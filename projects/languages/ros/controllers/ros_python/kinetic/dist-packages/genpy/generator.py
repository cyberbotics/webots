# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Library for Python message generation.

The structure of the serialization descends several levels of serializers:
 - msg_generator: generator for an individual msg file
  - serialize_fn_generator: generator for msg.serialize()
    - serializer_generator
      - field-type-specific serializers
                    raise MsgGenerationException("unknown file extension: %s"%f)

  - deserialize_fn_generator: generator for msg.deserialize()
    - serializer_generator
      - field-type-specific serializers
"""

from __future__ import print_function

import errno
import os
import keyword
import itertools
import sys
import traceback
import struct

import genmsg
import genmsg.msgs
import genmsg.msg_loader
import genmsg.gentools

from genmsg import InvalidMsgSpec, MsgContext, MsgSpec, MsgGenerationException
from genmsg.base import log

from . base import is_simple, SIMPLE_TYPES, SIMPLE_TYPES_DICT
from . generate_numpy import unpack_numpy, pack_numpy, NUMPY_DTYPE
from . generate_struct import reduce_pattern, serialize, \
     int32_pack, int32_unpack, pack, pack2, unpack, unpack2, compute_struct_pattern, \
     clear_patterns, add_pattern, get_patterns

# indent width
INDENT = '  '

def get_registered_ex(msg_context, type_):
    """
    wrapper for get_registered that wraps unknown types with a MsgGenerationException
    :param type_: ROS message type, ``str``
    """
    try:
        return msg_context.get_registered(type_)
    except:
        raise MsgGenerationException("Unknown type [%s]. Please check that the manifest.xml correctly declares dependencies."%type_)

################################################################################
# Special type handling for ROS builtin types that are not primitives

class Special:

    def __init__(self, constructor, post_deserialize, import_str):
        """
        :param constructor: expression to instantiate new type instance for deserialization, ``str``
        :param post_Deserialize: format string for expression to evaluate on type instance after deserialization is complete., ``str``
          variable name will be passed in as the single argument to format string.
        :param import_str: import to include if type is present, ``str``
        """
        self.constructor = constructor
        self.post_deserialize = post_deserialize
        self.import_str = import_str

    def get_post_deserialize(self, varname):
        """
        :returns: Post-deserialization code to executed (unindented) or
          ``None`` if no post-deserialization is required, ``str``
        """
        if self.post_deserialize:
            return self.post_deserialize%varname
        else:
            return None

_SPECIAL_TYPES = {
    genmsg.HEADER:   Special('std_msgs.msg._Header.Header()',     None, 'import std_msgs.msg'),
    genmsg.TIME:     Special('genpy.Time()',     '%s.canon()', 'import genpy'),
    genmsg.DURATION: Special('genpy.Duration()', '%s.canon()', 'import genpy'),
    }

def is_special(type_):
    """
    :returns: ``True` if *type_* is a special type (i.e. builtin represented as a class instead of a primitive), ``bool``
    """
    return type_ in _SPECIAL_TYPES

def get_special(type_):
    """
    :returns: special type handler for *type_* or ``None``, ``Special``
    """
    return _SPECIAL_TYPES.get(type_, None)

################################################################################
# utilities

# #671
def default_value(msg_context, field_type, default_package):
    """
    Compute default value for field_type

    :param default_package: default package, ``str``
    :param field_type: ROS .msg field type, ``str``
    :returns: default value encoded in Python string representation, ``str``
    """
    if field_type in ['byte', 'int8', 'int16', 'int32', 'int64',\
                          'char', 'uint8', 'uint16', 'uint32', 'uint64']:
        return '0'
    elif field_type in ['float32', 'float64']:
        return '0.'
    elif field_type == 'string':
        return "''"
    elif field_type == 'bool':
        return 'False'
    elif field_type.endswith(']'): # array type
        base_type, is_array, array_len = genmsg.msgs.parse_type(field_type)
        if base_type in ['char', 'uint8']:
            # strings, char[], and uint8s are all optimized to be strings
            if array_len is not None:
                return r"b'\0'*%s"%array_len
            else:
                return "b''"
        elif array_len is None: #var-length
            return '[]'
        else:
            # fixed-length
            def_val = default_value(msg_context, base_type, default_package)
            if base_type in [
                'byte', 'int8', 'int16', 'int32', 'int64', 'uint16', 'uint32',
                'uint64', 'float32', 'float64', 'string', 'bool'
            ]:  # fill primitive values
                return '[' + def_val + '] * ' + str(array_len)
            else:  # fill values with distinct instances
                def_val = default_value(msg_context, base_type, default_package)
                return '[' + def_val + ' for _ in range(' + str(array_len) + ')]'
    else:
        return compute_constructor(msg_context, default_package, field_type)

def flatten(msg_context, msg):
    """
    Flattens the msg spec so that embedded message fields become
    direct references. The resulting MsgSpec isn't a true/legal
    :class:`MsgSpec` and should only be used for serializer generation.
    :param msg: MsgSpec to flatten
    :returns: flattened MsgSpec message
    """
    new_types = []
    new_names = []
    for t, n in zip(msg.types, msg.names):
        # Parse type to make sure we don't flatten an array
        msg_type, is_array, _ = genmsg.msgs.parse_type(t)
        #flatten embedded types - note: bug #59
        if not is_array and msg_context.is_registered(t):
            msg_spec = flatten(msg_context, msg_context.get_registered(t))
            new_types.extend(msg_spec.types)
            for n2 in msg_spec.names:
                new_names.append(n+'.'+n2)
        else:
            #I'm not sure if it's a performance win to flatten fixed-length arrays
            #as you get n __getitems__ method calls vs. a single *array call
            new_types.append(t)
            new_names.append(n)
    return MsgSpec(new_types, new_names, msg.constants, msg.text, msg.full_name)

def make_python_safe(spec):
    """
    Remap field/constant names in spec to avoid collision with Python reserved words.

    :param spec: msg spec to map to new, python-safe field names, ``MsgSpec``
    :returns: python-safe message specification, ``MsgSpec``
    """
    new_c = [genmsg.Constant(c.type, _remap_reserved(c.name), c.val, c.val_text) for c in spec.constants]
    return MsgSpec(spec.types, [_remap_reserved(n) for n in spec.names], new_c, spec.text, spec.full_name)

def _remap_reserved(field_name):
    """
    Map field_name to a python-safe representation, if necessary
    :param field_name: msg field name, ``str``
    :returns: remapped name, ``str``
    """
    # include 'self' as well because we are within a class instance
    idx = field_name.rfind('.')
    if idx > 0:
        prefix = field_name[:idx+1]
        sub_field_name = field_name[idx+1:]
    else:
        prefix = ''
        sub_field_name = field_name
        
    if sub_field_name in keyword.kwlist + ['self']:
        sub_field_name = sub_field_name + "_"
    return prefix + sub_field_name

################################################################################
# (de)serialization routines

def compute_post_deserialize(type_, varname):
    """
    Compute post-deserialization code for type_, if necessary
    :returns: code to execute post-deserialization (unindented), or None if not necessary. ``str``
    """
    s = get_special(type_)
    if s is not None:
        return s.get_post_deserialize(varname)

def compute_constructor(msg_context, package, type_):
    """
    Compute python constructor expression for specified message type implementation
    :param package str: package that type is being imported into. Used
        to resolve type_ if package is not specified. ``str``
    :param type_: message type, ``str``
    """
    if is_special(type_):
        return get_special(type_).constructor
    elif genmsg.msgs.bare_msg_type(type_) != type_:
        # array or other weird type
        return None
    else:
        base_pkg, base_type_ = compute_pkg_type(package, type_)
        if not msg_context.is_registered("%s/%s"%(base_pkg,base_type_)):
            return None
        else:
            return '%s.msg.%s()'%(base_pkg, base_type_)

def compute_pkg_type(package, type_):
    """
    :param package: package that type is being imported into, ``str``
    :param type: message type (package resource name), ``str``
    :returns: python package and type name, ``(str, str)``
    """
    splits = type_.split(genmsg.SEP)
    if len(splits) == 1:
        return package, splits[0]
    elif len(splits) == 2:
        return tuple(splits)
    else:
        raise MsgGenerationException("illegal message type: %s"%type_)

def compute_import(msg_context, package, type_):
    """
    Compute python import statement for specified message type implementation
    :param package: package that type is being imported into, ``str``
    :param type_: message type (package resource name), ``str``
    :returns: list of import statements (no newline) required to use type_ from package, ``[str]``
    """
    # orig_base_type is the unresolved type
    orig_base_type = genmsg.msgs.bare_msg_type(type_) # strip array-suffix
    # resolve orig_base_type based on the current package context.
    # base_type is the resolved type stripped of any package name.
    # pkg is the actual package of type_.
    pkg, base_type = compute_pkg_type(package, orig_base_type)
    full_msg_type = "%s/%s"%(pkg, base_type) # compute fully-qualified type
    # important: have to do is_builtin check first. We do this check
    # against the unresolved type builtins/specials are never
    # relative. This requires some special handling for Header, which has
    # two names (Header and std_msgs/Header).
    if genmsg.msgs.is_builtin(orig_base_type) or \
           genmsg.msgs.is_header_type(orig_base_type):
        # of the builtin types, only special types require import
        # handling. we switch to base_type as special types do not
        # include package names.
        if is_special(base_type):
            retval = [get_special(base_type).import_str]
        else:
            retval = []
    elif not msg_context.is_registered(full_msg_type):
        retval = []
    else:
        retval = ['import %s.msg'%pkg]
        iter_types = get_registered_ex(msg_context, full_msg_type).types
        for t in iter_types:
            assert t != full_msg_type, "msg [%s] has circular self-dependencies"%(full_msg_type)
            full_sub_type = "%s/%s"%(package, t)
            log("compute_import", full_msg_type, package, t)
            sub = compute_import(msg_context, package, t)
            retval.extend([x for x in sub if not x in retval])
    return retval

def compute_full_text_escaped(msg_context, spec):
    """
    Same as genmsg.compute_full_text, except that the
    resulting text is escaped to be safe for Python's triple-quote string
    quoting

    :param get_deps_dict: dictionary returned by load_dependencies call, ``dict``
    :returns: concatenated text for msg/srv file and embedded msg/srv types. Text will be escaped for triple-quote, ``str``
    """
    msg_definition = genmsg.compute_full_text(msg_context, spec)
    msg_definition = msg_definition.replace('"""', r'\"\"\"')
    return msg_definition

################################################################################
# (De)serialization generators

_serial_context = ''
_context_stack = []

_counter = 0
def next_var():
    # we could optimize this by reusing vars once the context is popped
    global _counter
    _counter += 1
    return '_v%s'%_counter

def reset_var():
    global _counter
    _counter = 0

def push_context(context):
    """
    Push new variable context onto context stack.  The context stack
    manages field-reference context for serialization, e.g. 'self.foo'
    vs. 'self.bar.foo' vs. 'var.foo'
    """
    global _serial_context, _context_stack
    _context_stack.append(_serial_context)
    _serial_context = context

def pop_context():
    """
    Pop variable context from context stack.  The context stack manages
    field-reference context for serialization, e.g. 'self.foo'
    vs. 'self.bar.foo' vs. 'var.foo'
    """
    global _serial_context
    _serial_context = _context_stack.pop()

# These are the workhorses of the message generation. The generators
# are implemented as iterators, where each iteration value is a line
# of Python code. The generators will invoke underlying generators,
# using the context stack to manage any changes in variable-naming, so
# that code can be reused as much as possible.

def len_serializer_generator(var, is_string, serialize):
    """
    Generator for array-length serialization (32-bit, little-endian unsigned integer)
    :param var: variable name, ``str``
    :param is_string: if True, variable is a string type, ``bool``
    :param serialize bool: if True, generate code for
      serialization. Other, generate code for deserialization, ``bool``
    """
    if serialize:
        yield "length = len(%s)"%var
        # NOTE: it's more difficult to save a call to struct.pack with
        # the array length as we are already using *array_val to pass
        # into struct.pack as *args. Although it's possible that
        # Python is not optimizing it, it is potentially worse for
        # performance to attempt to combine
        if not is_string:
            yield int32_pack("length")
    else:
        yield "start = end"
        yield "end += 4"
        yield int32_unpack('length', 'str[start:end]') #4 = struct.calcsize('<i')

def string_serializer_generator(package, type_, name, serialize):
    """
    Generator for string types. similar to arrays, but with more
    efficient call to struct.pack.

    :param name: spec field name, ``str``
    :param serialize: if ``True``, generate code for
      serialization. Other, generate code for deserialization, ``bool``
    """
    # don't optimize in deserialization case as assignment doesn't
    # work
    if _serial_context and serialize:
        # optimize as string serialization accesses field twice
        yield "_x = %s%s"%(_serial_context, name)
        var = "_x"
    else:
        var = _serial_context+name

    # the length generator is a noop if serialize is True as we
    # optimize the serialization call.
    base_type, is_array, array_len = genmsg.msgs.parse_type(type_)
    # - don't serialize length for fixed-length arrays of bytes
    if base_type not in ['uint8', 'char'] or array_len is None:
        for y in len_serializer_generator(var, True, serialize):
            yield y #serialize string length

    if serialize:
        #serialize length and string together

        #check to see if its a uint8/byte type, in which case we need to convert to string before serializing
        base_type, is_array, array_len = genmsg.msgs.parse_type(type_)
        if base_type in ['uint8', 'char']:
            yield "# - if encoded as a list instead, serialize as bytes instead of string"
            if array_len is None:
                yield "if type(%s) in [list, tuple]:"%var
                yield INDENT+pack2("'<I%sB'%length", "length, *%s"%var)
                yield "else:"
                yield INDENT+pack2("'<I%ss'%length", "length, %s"%var)
            else:
                yield "if type(%s) in [list, tuple]:"%var
                yield INDENT+pack('%sB'%array_len, "*%s"%var)
                yield "else:"
                yield INDENT+pack('%ss'%array_len, var)
        else:
            # FIXME: for py3k, this needs to be w/ encode(), but this interferes with actual byte data
            yield "if python3 or type(%s) == unicode:"%(var)
            yield INDENT+"%s = %s.encode('utf-8')"%(var,var) #For unicode-strings in Python2, encode using utf-8
            yield INDENT+"length = len(%s)"%(var) # Update the length after utf-8 conversion

            yield pack2("'<I%ss'%length", "length, %s"%var)
    else:
        yield "start = end"
        if array_len is not None:
            yield "end += %s" % array_len
            yield "%s = str[start:end]" % var
        else:
            yield "end += length"
            if base_type in ['uint8', 'char']:
                yield "%s = str[start:end]" % (var)
            else:
                yield "if python3:"
                yield INDENT+"%s = str[start:end].decode('utf-8')" % (var) #If messages are python3-decode back to unicode
                yield "else:"
                yield INDENT+"%s = str[start:end]" % (var)


def array_serializer_generator(msg_context, package, type_, name, serialize, is_numpy):
    """
    Generator for array types

    :raises: :exc:`MsgGenerationException` If array spec is invalid
    """
    base_type, is_array, array_len = genmsg.msgs.parse_type(type_)
    if not is_array:
        raise MsgGenerationException("Invalid array spec: %s"%type_)
    var_length = array_len is None

    # handle fixed-size byte arrays could be slightly more efficient
    # as we recalculated the length in the generated code.
    if base_type in ['char', 'uint8']: #treat unsigned int8 arrays as string type
        for y in string_serializer_generator(package, type_, name, serialize):
            yield y
        return

    var = _serial_context+name
    # yield length serialization, if necessary
    if var_length:
        for y in len_serializer_generator(var, False, serialize):
            yield y #serialize array length
        length = None
    else:
        length = array_len

    #optimization for simple arrays
    if is_simple(base_type):
        if var_length:
            pattern = compute_struct_pattern([base_type])
            yield "pattern = '<%%s%s'%%length"%pattern
            if serialize:
                if is_numpy:
                    yield pack_numpy(var)
                else:
                    yield pack2('pattern', "*"+var)
            else:
                yield "start = end"
                yield "end += struct.calcsize(pattern)"
                if is_numpy:
                    dtype = NUMPY_DTYPE[base_type]
                    yield unpack_numpy(var, 'length', dtype, 'str[start:end]')
                else:
                    yield unpack2(var, 'pattern', 'str[start:end]')
        else:
            pattern = "%s%s"%(length, compute_struct_pattern([base_type]))
            if serialize:
                if is_numpy:
                    yield pack_numpy(var)
                else:
                    yield pack(pattern, "*"+var)
            else:
                yield "start = end"
                yield "end += %s"%struct.calcsize('<%s'%pattern)
                if is_numpy:
                    dtype = NUMPY_DTYPE[base_type]
                    yield unpack_numpy(var, length, dtype, 'str[start:end]')
                else:
                    yield unpack(var, pattern, 'str[start:end]')
        if not serialize and base_type == 'bool':
            # convert uint8 to bool
            if base_type == 'bool':
                yield "%s = map(bool, %s)"%(var, var)

    else:
        #generic recursive serializer
        #NOTE: this is functionally equivalent to the is_registered branch of complex_serializer_generator

        # choose a unique temporary variable for iterating
        loop_var = 'val%s'%len(_context_stack)

        # compute the variable context and factory to use
        if base_type == 'string':
            push_context('')
            factory = string_serializer_generator(package, base_type, loop_var, serialize)
        else:
            push_context('%s.'%loop_var)
            factory = serializer_generator(msg_context, make_python_safe(get_registered_ex(msg_context, base_type)), serialize, is_numpy)

        if serialize:
            yield 'for %s in %s:'%(loop_var, var)
        else:
            yield '%s = []'%var
            if var_length:
                yield 'for i in range(0, length):'
            else:
                yield 'for i in range(0, %s):'%length
            if base_type != 'string':
                yield INDENT + '%s = %s'%(loop_var, compute_constructor(msg_context, package, base_type))
        for y in factory:
            yield INDENT + y
        if not serialize:
            yield INDENT + '%s.append(%s)'%(var, loop_var)
        pop_context()

def complex_serializer_generator(msg_context, package, type_, name, serialize, is_numpy):
    """
    Generator for serializing complex type

    :param serialize: if True, generate serialization
      code. Otherwise, deserialization code. ``bool``
    :param is_numpy: if True, generate serializer code for numpy
      datatypes instead of Python lists, ``bool``
    :raises: MsgGenerationException If type is not a valid
    """
    # ordering of these statements is important as we mutate the type
    # string we are checking throughout. parse_type strips array
    # brackets, then we check for the 'complex' builtin types (string,
    # time, duration, Header), then we canonicalize it to an embedded
    # message type.
    _, is_array, _ = genmsg.msgs.parse_type(type_)

    #Array
    if is_array:
        for y in array_serializer_generator(msg_context, package, type_, name, serialize, is_numpy):
            yield y
    #Embedded Message
    elif type_ == 'string':
        for y in string_serializer_generator(package, type_, name, serialize):
            yield y
    else:
        if not is_special(type_):
            # canonicalize type
            pkg, base_type = compute_pkg_type(package, type_)
            type_ = "%s/%s"%(pkg, base_type)
        if msg_context.is_registered(type_):
            # descend data structure ####################
            ctx_var = next_var()
            yield "%s = %s"%(ctx_var, _serial_context+name)
            push_context(ctx_var+'.')
            # unoptimized code
            #push_context(_serial_context+name+'.')
            for y in serializer_generator(msg_context, make_python_safe(get_registered_ex(msg_context, type_)), serialize, is_numpy):
                yield y #recurs on subtype
            pop_context()
        else:
            #Invalid
            raise MsgGenerationException("Unknown type: %s. Package context is %s"%(type_, package))

def simple_serializer_generator(msg_context, spec, start, end, serialize): #primitives that can be handled with struct
    """
    Generator (de)serialization code for multiple fields from spec

    :param spec: :class:`genmsg.MsgSpec`
    :param start: first field to serialize, ``int``
    :param end: last field to serialize, ``int``
    """
    # optimize member var access
    if end - start > 1 and _serial_context.endswith('.'):
        yield '_x = '+_serial_context[:-1]
        vars_ = '_x.' + (', _x.').join(spec.names[start:end])
    else:
        vars_ = _serial_context + (', '+_serial_context).join(spec.names[start:end])

    pattern = compute_struct_pattern(spec.types[start:end])
    if serialize:
        yield pack(pattern, vars_)
    else:
        yield "start = end"
        yield "end += %s"%struct.calcsize('<%s'%reduce_pattern(pattern))
        yield unpack('(%s,)'%vars_, pattern, 'str[start:end]')

        # convert uint8 to bool. this doesn't add much value as Python
        # equality test on a field will return that True == 1, but I
        # want to be consistent with bool
        bool_vars = [(f, t) for f, t in zip(spec.names[start:end], spec.types[start:end]) if t == 'bool']
        for f, t in bool_vars:
            #TODO: could optimize this as well
            var = _serial_context+f
            yield "%s = bool(%s)"%(var, var)

def serializer_generator(msg_context, spec, serialize, is_numpy):
    """
    Python generator that yields un-indented python code for
    (de)serializing MsgSpec. The code this yields is meant to be
    included in a class method and cannot be used
    standalone. serialize_fn_generator and deserialize_fn_generator
    wrap method to provide appropriate class field initializations.

    :param serialize: if True, yield serialization
      code. Otherwise, yield deserialization code. ``bool``
    :param is_numpy: if True, generate serializer code for numpy datatypes instead of Python lists. ``bool``
    """
    # Break spec into chunks of simple (primitives) vs. complex (arrays, etc...)
    # Simple types are batch serialized using the python struct module.
    # Complex types are individually serialized
    if spec is None:
        raise MsgGenerationException("spec is none")
    names, types = spec.names, spec.types
    if serialize and not len(names): #Empty
        yield "pass"
        return

    _max_chunk = 255
    # iterate through types. whenever we encounter a non-simple type,
    # yield serializer for any simple types we've encountered until
    # then, then yield the complex type serializer
    curr = 0
    for (i, full_type) in enumerate(types):
        if not is_simple(full_type):
            if i != curr: #yield chunk of simples
                for _start in range(curr, i, _max_chunk):
                    _end = min(_start + _max_chunk, i)
                    for y in simple_serializer_generator(msg_context, spec, _start, _end, serialize):
                        yield y
            curr = i+1
            for y in complex_serializer_generator(msg_context, spec.package, full_type, names[i], serialize, is_numpy):
                yield y
    if curr < len(types): #yield rest of simples
        for _start in range(curr, len(types), _max_chunk):
            _end = min(_start + _max_chunk, len(types))
            for y in simple_serializer_generator(msg_context, spec, _start, _end, serialize):
                yield y

def serialize_fn_generator(msg_context, spec, is_numpy=False):
    """
    generator for body of serialize() function
    :param is_numpy: if True, generate serializer code for numpy
      datatypes instead of Python lists, ``bool``
    """
    # method-var context #########
    yield "try:"
    push_context('self.')
    #NOTE: we flatten the spec for optimal serialization
    # #3741: make sure to have sub-messages python safe
    flattened = make_python_safe(flatten(msg_context, spec))
    for y in serializer_generator(msg_context, flattened, True, is_numpy):
        yield "  "+y
    pop_context()
    yield "except struct.error as se: self._check_types(struct.error(\"%s: '%s' when writing '%s'\" % (type(se), str(se), str(locals().get('_x', self)))))"
    yield "except TypeError as te: self._check_types(ValueError(\"%s: '%s' when writing '%s'\" % (type(te), str(te), str(locals().get('_x', self)))))"
    # done w/ method-var context #

def deserialize_fn_generator(msg_context, spec, is_numpy=False):
    """
    generator for body of deserialize() function
    :param is_numpy: if True, generate serializer code for numpy
      datatypes instead of Python lists, ``bool``
    """
    yield "try:"
    package = spec.package
    #Instantiate embedded type classes
    for type_, name in spec.fields():
        if msg_context.is_registered(type_):
            yield "  if self.%s is None:"%name
            yield "    self.%s = %s"%(name, compute_constructor(msg_context, package, type_))
    yield "  end = 0" #initialize var

    # method-var context #########
    push_context('self.')
    #NOTE: we flatten the spec for optimal serialization
    # #3741: make sure to have sub-messages python safe
    flattened = make_python_safe(flatten(msg_context, spec))
    for y in serializer_generator(msg_context, flattened, False, is_numpy):
        yield "  "+y
    pop_context()
    # done w/ method-var context #

    # generate post-deserialization code
    for type_, name in spec.fields():
        code = compute_post_deserialize(type_, "self.%s"%name)
        if code:
            yield "  %s"%code

    yield "  return self"
    yield "except struct.error as e:"
    yield "  raise genpy.DeserializationError(e) #most likely buffer underfill"

def msg_generator(msg_context, spec, search_path):
    """
    Python code generator for .msg files. Generates a Python from a
     :class:`genmsg.MsgSpec`.

    :param spec: parsed .msg :class:`genmsg.MsgSpec` instance
    :param search_path: dictionary mapping message namespaces to a directory locations
    """

    # #2990: have to compute md5sum before any calls to make_python_safe

    # generate dependencies dictionary. omit files calculation as we
    # rely on in-memory MsgSpecs instead so that we can generate code
    # for older versions of msg files
    try:
        genmsg.msg_loader.load_depends(msg_context, spec, search_path)
    except InvalidMsgSpec as e:
        raise MsgGenerationException("Cannot generate .msg for %s/%s: %s"%(package, name, str(e)))
    md5sum = genmsg.compute_md5(msg_context, spec)

    # remap spec names to be Python-safe
    spec = make_python_safe(spec)
    spec_names = spec.names

    # #1807 : this will be much cleaner when msggenerator library is
    # rewritten to not use globals
    clear_patterns()

    yield '# This Python file uses the following encoding: utf-8'
    yield '"""autogenerated by genpy from %s.msg. Do not edit."""'%spec.full_name
    yield 'import sys'
    yield 'python3 = True if sys.hexversion > 0x03000000 else False'
    yield 'import genpy\nimport struct\n'
    import_strs = []
    for t in spec.types:
        import_strs.extend(compute_import(msg_context, spec.package, t))
    import_strs = set(import_strs)
    for i in import_strs:
        if i:
            yield i

    yield ''

    fulltype = spec.full_name
    name = spec.short_name

    #Yield data class first, e.g. Point2D
    yield 'class %s(genpy.Message):'%spec.short_name
    yield '  _md5sum = "%s"'%(md5sum)
    yield '  _type = "%s"'%(fulltype)
    yield '  _has_header = %s #flag to mark the presence of a Header object'%spec.has_header()
    
    full_text = compute_full_text_escaped(msg_context, spec)
    # escape trailing double-quote, unless already escaped, before wrapping in """
    if full_text.endswith('"') and not full_text.endswith(r'\"'):
        full_text = full_text[:-1] + r'\"'
    yield '  _full_text = """%s"""'%full_text

    if spec.constants:
        yield '  # Pseudo-constants'
        for c in spec.constants:
            if c.type == 'string':
                val = c.val
                if '"' in val and "'" in val:
                    # crude escaping of \ and "
                    escaped = c.val.replace('\\', '\\\\')
                    escaped = escaped.replace('\"', '\\"')
                    yield '  %s = "%s"'%(c.name, escaped)
                elif '"' in val: #use raw encoding for prettiness
                    yield "  %s = r'%s'"%(c.name, val)
                elif "'" in val: #use raw encoding for prettiness
                    yield '  %s = r"%s"'%(c.name, val)
                else:
                    yield "  %s = '%s'"%(c.name, val)
            else:
                yield '  %s = %s'%(c.name, c.val)
        yield ''

    if len(spec_names):
        yield "  __slots__ = ['"+"','".join(spec_names)+"']"
        yield "  _slot_types = ['"+"','".join(spec.types)+"']"
    else:
        yield "  __slots__ = []"
        yield "  _slot_types = []"

    yield """
  def __init__(self, *args, **kwds):
    \"\"\"
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       %s

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    \"\"\"
    if args or kwds:
      super(%s, self).__init__(*args, **kwds)"""%(','.join(spec_names), name)

    if len(spec_names):
        yield "      #message fields cannot be None, assign default values for those that are"
        for (t, s) in zip(spec.types, spec_names):
            yield "      if self.%s is None:"%s
            yield "        self.%s = %s"%(s, default_value(msg_context, t, spec.package))
    if len(spec_names) > 0:
      yield "    else:"
      for (t, s) in zip(spec.types, spec_names):
          yield "      self.%s = %s"%(s, default_value(msg_context, t, spec.package))

    yield """
  def _get_types(self):
    \"\"\"
    internal API method
    \"\"\"
    return self._slot_types

  def serialize(self, buff):
    \"\"\"
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    \"\"\""""
    for y in serialize_fn_generator(msg_context, spec):
        yield "    "+ y
    yield """
  def deserialize(self, str):
    \"\"\"
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    \"\"\""""
    for y in deserialize_fn_generator(msg_context, spec):
        yield "    " + y
    yield ""

    yield """
  def serialize_numpy(self, buff, numpy):
    \"\"\"
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    \"\"\""""
    for y in serialize_fn_generator(msg_context, spec, is_numpy=True):
        yield "    "+ y
    yield """
  def deserialize_numpy(self, str, numpy):
    \"\"\"
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    \"\"\""""
    for y in deserialize_fn_generator(msg_context, spec, is_numpy=True):
        yield "    " + y
    yield ""


    # #1807 : this will be much cleaner when msggenerator library is
    # rewritten to not use globals
    yield '_struct_I = genpy.struct_I'
    yield 'def _get_struct_I():'
    yield '    global _struct_I'
    yield '    return _struct_I'
    patterns = get_patterns()
    for p in set(patterns):
        # I patterns are already optimized
        if p == 'I':
            continue
        var_name = '_struct_%s'%(p.replace('<',''))
        yield '%s = None' % var_name
        yield 'def _get%s():' % var_name
        yield '    global %s' % var_name
        yield '    if %s is None:' % var_name
        yield '        %s = struct.Struct("<%s")' % (var_name, p)
        yield '    return %s' % var_name
    clear_patterns()

def srv_generator(msg_context, spec, search_path):
    for mspec in (spec.request, spec.response):
        for l in msg_generator(msg_context, mspec, search_path):
            yield l

    name = spec.short_name
    req, resp = ["%s%s"%(name, suff) for suff in ['Request', 'Response']]

    fulltype = spec.full_name

    genmsg.msg_loader.load_depends(msg_context, spec, search_path)
    md5 = genmsg.compute_md5(msg_context, spec)

    yield "class %s(object):"%name
    yield "  _type          = '%s'"%fulltype
    yield "  _md5sum = '%s'"%md5
    yield "  _request_class  = %s"%req
    yield "  _response_class = %s"%resp

def _module_name(type_name):
    """
    :param type_name str: Name of message type sans package,
      e.g. 'String'
    :returns str: name of python module for auto-generated code
    """
    return "_"+type_name

def compute_resource_name(filename, ext):
    """
    Convert resource filename to ROS resource name
    :param filename str: path to .msg/.srv file
    :returns str: name of ROS resource
    """
    return os.path.basename(filename)[:-len(ext)]

def compute_outfile_name(outdir, infile_name, ext):
    """
    :param outdir str: path to directory that files are generated to
    :returns str: output file path based on input file name and output directory
    """
    # Use leading _ so that module name does not collide with message name. It also
    # makes it more clear that the .py file should not be imported directly
    return os.path.join(outdir, _module_name(compute_resource_name(infile_name, ext))+".py")

class Generator(object):

    def __init__(self, what, ext, spec_loader_fn, generator_fn):
        self.what = what
        self.ext = ext
        self.spec_loader_fn = spec_loader_fn
        self.generator_fn = generator_fn

    def generate(self, msg_context, full_type, f, outdir, search_path):
        try:
            # you can't just check first... race condition
            os.makedirs(outdir)
        except OSError as e:
            if e.errno != errno.EEXIST:
                raise
        # generate message files for request/response
        spec = self.spec_loader_fn(msg_context, f, full_type)
        outfile = compute_outfile_name(outdir, os.path.basename(f), self.ext)
        with open(outfile, 'w') as f:
            for l in self.generator_fn(msg_context, spec, search_path):
                f.write(l+'\n')
        return outfile

    def generate_messages(self, package, package_files, outdir, search_path):
        """
        :returns: return code, ``int``
        """
        if not genmsg.is_legal_resource_base_name(package):
            raise MsgGenerationException("\nERROR: package name '%s' is illegal and cannot be used in message generation.\nPlease see http://ros.org/wiki/Names"%(package))

        # package/src/package/msg for messages, packages/src/package/srv for services
        msg_context = MsgContext.create_default()
        retcode = 0
        for f in package_files:
            try:
                f = os.path.abspath(f)
                infile_name = os.path.basename(f)
                full_type = genmsg.gentools.compute_full_type_name(package, infile_name);
                outfile = self.generate(msg_context, full_type, f, outdir, search_path) #actual generation
            except Exception as e:
                if not isinstance(e, MsgGenerationException) and not isinstance(e, genmsg.msgs.InvalidMsgSpec):
                    traceback.print_exc()
                print("\nERROR: Unable to generate %s for package '%s': while processing '%s': %s\n"%(self.what, package, f, e), file=sys.stderr)
                retcode = 1 #flag error
        return retcode

class SrvGenerator(Generator):

    def __init__(self):
        super(SrvGenerator, self).__init__('services', genmsg.EXT_SRV,
                                           genmsg.msg_loader.load_srv_from_file,
                                           srv_generator)

class MsgGenerator(Generator):
    """
    Generates Python message code for all messages in a
    package. See genutil.Generator. In order to generator code for a
    single .msg file, see msg_generator.
    """
    def __init__(self):
        super(MsgGenerator, self).__init__('messages', genmsg.EXT_MSG,
                                           genmsg.msg_loader.load_msg_from_file,
                                           msg_generator)

