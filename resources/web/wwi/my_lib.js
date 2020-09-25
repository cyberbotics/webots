

// The Module object: Our interface to the outside world. We import
// and export values on it. There are various ways Module can be used:
// 1. Not defined. We create it here
// 2. A function parameter, function(Module) { ..generated code.. }
// 3. pre-run appended it, var Module = {}; ..generated code..
// 4. External script tag defines var Module.
// We need to check if Module already exists (e.g. case 3 above).
// Substitution will be replaced with actual code on later stage of the build,
// this way Closure Compiler will not mangle it (e.g. case 4. above).
// Note that if you want to run closure, and also to use Module
// after the generated code, you will need to define   var Module = {};
// before the code. Then that object will be used in the code, and you
// can continue to use Module afterwards as well.
var Module = typeof Module !== 'undefined' ? Module : {};



// --pre-jses are emitted after the Module integration code, so that they can
// refer to Module (if they choose; they can also define Module)
// {{PRE_JSES}}

// Sometimes an existing Module object exists with properties
// meant to overwrite the default module functionality. Here
// we collect those properties and reapply _after_ we configure
// the current environment's defaults to avoid having to be so
// defensive during initialization.
var moduleOverrides = {};
var key;
for (key in Module) {
  if (Module.hasOwnProperty(key)) {
    moduleOverrides[key] = Module[key];
  }
}

var arguments_ = [];
var thisProgram = './this.program';
var quit_ = function(status, toThrow) {
  throw toThrow;
};

// Determine the runtime environment we are in. You can customize this by
// setting the ENVIRONMENT setting at compile time (see settings.js).

var ENVIRONMENT_IS_WEB = false;
var ENVIRONMENT_IS_WORKER = false;
var ENVIRONMENT_IS_NODE = false;
var ENVIRONMENT_IS_SHELL = false;
ENVIRONMENT_IS_WEB = typeof window === 'object';
ENVIRONMENT_IS_WORKER = typeof importScripts === 'function';
// N.b. Electron.js environment is simultaneously a NODE-environment, but
// also a web environment.
ENVIRONMENT_IS_NODE = typeof process === 'object' && typeof process.versions === 'object' && typeof process.versions.node === 'string';
ENVIRONMENT_IS_SHELL = !ENVIRONMENT_IS_WEB && !ENVIRONMENT_IS_NODE && !ENVIRONMENT_IS_WORKER;

if (Module['ENVIRONMENT']) {
  throw new Error('Module.ENVIRONMENT has been deprecated. To force the environment, use the ENVIRONMENT compile-time option (for example, -s ENVIRONMENT=web or -s ENVIRONMENT=node)');
}



// `/` should be present at the end if `scriptDirectory` is not empty
var scriptDirectory = '';
function locateFile(path) {
  if (Module['locateFile']) {
    return Module['locateFile'](path, scriptDirectory);
  }
  return scriptDirectory + path;
}

// Hooks that are implemented differently in different runtime environments.
var read_,
    readAsync,
    readBinary,
    setWindowTitle;

var nodeFS;
var nodePath;

if (ENVIRONMENT_IS_NODE) {
  if (ENVIRONMENT_IS_WORKER) {
    scriptDirectory = require('path').dirname(scriptDirectory) + '/';
  } else {
    scriptDirectory = __dirname + '/';
  }




read_ = function shell_read(filename, binary) {
  var ret = tryParseAsDataURI(filename);
  if (ret) {
    return binary ? ret : ret.toString();
  }
  if (!nodeFS) nodeFS = require('fs');
  if (!nodePath) nodePath = require('path');
  filename = nodePath['normalize'](filename);
  return nodeFS['readFileSync'](filename, binary ? null : 'utf8');
};

readBinary = function readBinary(filename) {
  var ret = read_(filename, true);
  if (!ret.buffer) {
    ret = new Uint8Array(ret);
  }
  assert(ret.buffer);
  return ret;
};



  if (process['argv'].length > 1) {
    thisProgram = process['argv'][1].replace(/\\/g, '/');
  }

  arguments_ = process['argv'].slice(2);

  if (typeof module !== 'undefined') {
    module['exports'] = Module;
  }

  process['on']('uncaughtException', function(ex) {
    // suppress ExitStatus exceptions from showing an error
    if (!(ex instanceof ExitStatus)) {
      throw ex;
    }
  });

  process['on']('unhandledRejection', abort);

  quit_ = function(status) {
    process['exit'](status);
  };

  Module['inspect'] = function () { return '[Emscripten Module object]'; };



} else
if (ENVIRONMENT_IS_SHELL) {


  if (typeof read != 'undefined') {
    read_ = function shell_read(f) {
      var data = tryParseAsDataURI(f);
      if (data) {
        return intArrayToString(data);
      }
      return read(f);
    };
  }

  readBinary = function readBinary(f) {
    var data;
    data = tryParseAsDataURI(f);
    if (data) {
      return data;
    }
    if (typeof readbuffer === 'function') {
      return new Uint8Array(readbuffer(f));
    }
    data = read(f, 'binary');
    assert(typeof data === 'object');
    return data;
  };

  if (typeof scriptArgs != 'undefined') {
    arguments_ = scriptArgs;
  } else if (typeof arguments != 'undefined') {
    arguments_ = arguments;
  }

  if (typeof quit === 'function') {
    quit_ = function(status) {
      quit(status);
    };
  }

  if (typeof print !== 'undefined') {
    // Prefer to use print/printErr where they exist, as they usually work better.
    if (typeof console === 'undefined') console = /** @type{!Console} */({});
    console.log = /** @type{!function(this:Console, ...*): undefined} */ (print);
    console.warn = console.error = /** @type{!function(this:Console, ...*): undefined} */ (typeof printErr !== 'undefined' ? printErr : print);
  }


} else

// Note that this includes Node.js workers when relevant (pthreads is enabled).
// Node.js workers are detected as a combination of ENVIRONMENT_IS_WORKER and
// ENVIRONMENT_IS_NODE.
if (ENVIRONMENT_IS_WEB || ENVIRONMENT_IS_WORKER) {
  if (ENVIRONMENT_IS_WORKER) { // Check worker, not web, since window could be polyfilled
    scriptDirectory = self.location.href;
  } else if (document.currentScript) { // web
    scriptDirectory = document.currentScript.src;
  }
  // blob urls look like blob:http://site.com/etc/etc and we cannot infer anything from them.
  // otherwise, slice off the final part of the url to find the script directory.
  // if scriptDirectory does not contain a slash, lastIndexOf will return -1,
  // and scriptDirectory will correctly be replaced with an empty string.
  if (scriptDirectory.indexOf('blob:') !== 0) {
    scriptDirectory = scriptDirectory.substr(0, scriptDirectory.lastIndexOf('/')+1);
  } else {
    scriptDirectory = '';
  }


  // Differentiate the Web Worker from the Node Worker case, as reading must
  // be done differently.
  {




  read_ = function shell_read(url) {
    try {
      var xhr = new XMLHttpRequest();
      xhr.open('GET', url, false);
      xhr.send(null);
      return xhr.responseText;
    } catch (err) {
      var data = tryParseAsDataURI(url);
      if (data) {
        return intArrayToString(data);
      }
      throw err;
    }
  };

  if (ENVIRONMENT_IS_WORKER) {
    readBinary = function readBinary(url) {
      try {
        var xhr = new XMLHttpRequest();
        xhr.open('GET', url, false);
        xhr.responseType = 'arraybuffer';
        xhr.send(null);
        return new Uint8Array(/** @type{!ArrayBuffer} */(xhr.response));
      } catch (err) {
        var data = tryParseAsDataURI(url);
        if (data) {
          return data;
        }
        throw err;
      }
    };
  }

  readAsync = function readAsync(url, onload, onerror) {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', url, true);
    xhr.responseType = 'arraybuffer';
    xhr.onload = function xhr_onload() {
      if (xhr.status == 200 || (xhr.status == 0 && xhr.response)) { // file URLs can return 0
        onload(xhr.response);
        return;
      }
      var data = tryParseAsDataURI(url);
      if (data) {
        onload(data.buffer);
        return;
      }
      onerror();
    };
    xhr.onerror = onerror;
    xhr.send(null);
  };




  }

  setWindowTitle = function(title) { document.title = title };
} else
{
  throw new Error('environment detection error');
}


// Set up the out() and err() hooks, which are how we can print to stdout or
// stderr, respectively.
var out = Module['print'] || console.log.bind(console);
var err = Module['printErr'] || console.warn.bind(console);

// Merge back in the overrides
for (key in moduleOverrides) {
  if (moduleOverrides.hasOwnProperty(key)) {
    Module[key] = moduleOverrides[key];
  }
}
// Free the object hierarchy contained in the overrides, this lets the GC
// reclaim data used e.g. in memoryInitializerRequest, which is a large typed array.
moduleOverrides = null;

// Emit code to handle expected values on the Module object. This applies Module.x
// to the proper local x. This has two benefits: first, we only emit it if it is
// expected to arrive, and second, by using a local everywhere else that can be
// minified.
if (Module['arguments']) arguments_ = Module['arguments'];if (!Object.getOwnPropertyDescriptor(Module, 'arguments')) Object.defineProperty(Module, 'arguments', { configurable: true, get: function() { abort('Module.arguments has been replaced with plain arguments_ (the initial value can be provided on Module, but after startup the value is only looked for on a local variable of that name)') } });
if (Module['thisProgram']) thisProgram = Module['thisProgram'];if (!Object.getOwnPropertyDescriptor(Module, 'thisProgram')) Object.defineProperty(Module, 'thisProgram', { configurable: true, get: function() { abort('Module.thisProgram has been replaced with plain thisProgram (the initial value can be provided on Module, but after startup the value is only looked for on a local variable of that name)') } });
if (Module['quit']) quit_ = Module['quit'];if (!Object.getOwnPropertyDescriptor(Module, 'quit')) Object.defineProperty(Module, 'quit', { configurable: true, get: function() { abort('Module.quit has been replaced with plain quit_ (the initial value can be provided on Module, but after startup the value is only looked for on a local variable of that name)') } });

// perform assertions in shell.js after we set up out() and err(), as otherwise if an assertion fails it cannot print the message
// Assertions on removed incoming Module JS APIs.
assert(typeof Module['memoryInitializerPrefixURL'] === 'undefined', 'Module.memoryInitializerPrefixURL option was removed, use Module.locateFile instead');
assert(typeof Module['pthreadMainPrefixURL'] === 'undefined', 'Module.pthreadMainPrefixURL option was removed, use Module.locateFile instead');
assert(typeof Module['cdInitializerPrefixURL'] === 'undefined', 'Module.cdInitializerPrefixURL option was removed, use Module.locateFile instead');
assert(typeof Module['filePackagePrefixURL'] === 'undefined', 'Module.filePackagePrefixURL option was removed, use Module.locateFile instead');
assert(typeof Module['read'] === 'undefined', 'Module.read option was removed (modify read_ in JS)');
assert(typeof Module['readAsync'] === 'undefined', 'Module.readAsync option was removed (modify readAsync in JS)');
assert(typeof Module['readBinary'] === 'undefined', 'Module.readBinary option was removed (modify readBinary in JS)');
assert(typeof Module['setWindowTitle'] === 'undefined', 'Module.setWindowTitle option was removed (modify setWindowTitle in JS)');
assert(typeof Module['TOTAL_MEMORY'] === 'undefined', 'Module.TOTAL_MEMORY has been renamed Module.INITIAL_MEMORY');
if (!Object.getOwnPropertyDescriptor(Module, 'read')) Object.defineProperty(Module, 'read', { configurable: true, get: function() { abort('Module.read has been replaced with plain read_ (the initial value can be provided on Module, but after startup the value is only looked for on a local variable of that name)') } });
if (!Object.getOwnPropertyDescriptor(Module, 'readAsync')) Object.defineProperty(Module, 'readAsync', { configurable: true, get: function() { abort('Module.readAsync has been replaced with plain readAsync (the initial value can be provided on Module, but after startup the value is only looked for on a local variable of that name)') } });
if (!Object.getOwnPropertyDescriptor(Module, 'readBinary')) Object.defineProperty(Module, 'readBinary', { configurable: true, get: function() { abort('Module.readBinary has been replaced with plain readBinary (the initial value can be provided on Module, but after startup the value is only looked for on a local variable of that name)') } });
if (!Object.getOwnPropertyDescriptor(Module, 'setWindowTitle')) Object.defineProperty(Module, 'setWindowTitle', { configurable: true, get: function() { abort('Module.setWindowTitle has been replaced with plain setWindowTitle (the initial value can be provided on Module, but after startup the value is only looked for on a local variable of that name)') } });
var IDBFS = 'IDBFS is no longer included by default; build with -lidbfs.js';
var PROXYFS = 'PROXYFS is no longer included by default; build with -lproxyfs.js';
var WORKERFS = 'WORKERFS is no longer included by default; build with -lworkerfs.js';
var NODEFS = 'NODEFS is no longer included by default; build with -lnodefs.js';






// {{PREAMBLE_ADDITIONS}}

var STACK_ALIGN = 16;

function alignMemory(size, factor) {
  if (!factor) factor = STACK_ALIGN; // stack alignment (16-byte) by default
  return Math.ceil(size / factor) * factor;
}

function getNativeTypeSize(type) {
  switch (type) {
    case 'i1': case 'i8': return 1;
    case 'i16': return 2;
    case 'i32': return 4;
    case 'i64': return 8;
    case 'float': return 4;
    case 'double': return 8;
    default: {
      if (type[type.length-1] === '*') {
        return 4; // A pointer
      } else if (type[0] === 'i') {
        var bits = Number(type.substr(1));
        assert(bits % 8 === 0, 'getNativeTypeSize invalid bits ' + bits + ', type ' + type);
        return bits / 8;
      } else {
        return 0;
      }
    }
  }
}

function warnOnce(text) {
  if (!warnOnce.shown) warnOnce.shown = {};
  if (!warnOnce.shown[text]) {
    warnOnce.shown[text] = 1;
    err(text);
  }
}





// Wraps a JS function as a wasm function with a given signature.
function convertJsFunctionToWasm(func, sig) {
  return func;
}

var freeTableIndexes = [];

// Weak map of functions in the table to their indexes, created on first use.
var functionsInTableMap;

// Add a wasm function to the table.
function addFunctionWasm(func, sig) {
  var table = wasmTable;

  // Check if the function is already in the table, to ensure each function
  // gets a unique index. First, create the map if this is the first use.
  if (!functionsInTableMap) {
    functionsInTableMap = new WeakMap();
    for (var i = 0; i < table.length; i++) {
      var item = table.get(i);
      // Ignore null values.
      if (item) {
        functionsInTableMap.set(item, i);
      }
    }
  }
  if (functionsInTableMap.has(func)) {
    return functionsInTableMap.get(func);
  }

  // It's not in the table, add it now.


  var ret;
  // Reuse a free index if there is one, otherwise grow.
  if (freeTableIndexes.length) {
    ret = freeTableIndexes.pop();
  } else {
    ret = table.length;
    // Grow the table
    try {
      table.grow(1);
    } catch (err) {
      if (!(err instanceof RangeError)) {
        throw err;
      }
      throw 'Unable to grow wasm table. Set ALLOW_TABLE_GROWTH.';
    }
  }

  // Set the new value.
  try {
    // Attempting to call this with JS function will cause of table.set() to fail
    table.set(ret, func);
  } catch (err) {
    if (!(err instanceof TypeError)) {
      throw err;
    }
    assert(typeof sig !== 'undefined', 'Missing signature argument to addFunction');
    var wrapped = convertJsFunctionToWasm(func, sig);
    table.set(ret, wrapped);
  }

  functionsInTableMap.set(func, ret);

  return ret;
}

function removeFunctionWasm(index) {
  functionsInTableMap.delete(wasmTable.get(index));
  freeTableIndexes.push(index);
}

// 'sig' parameter is required for the llvm backend but only when func is not
// already a WebAssembly function.
function addFunction(func, sig) {
  assert(typeof func !== 'undefined');

  return addFunctionWasm(func, sig);
}

function removeFunction(index) {
  removeFunctionWasm(index);
}









function makeBigInt(low, high, unsigned) {
  return unsigned ? ((+((low>>>0)))+((+((high>>>0)))*4294967296.0)) : ((+((low>>>0)))+((+((high|0)))*4294967296.0));
}

var tempRet0 = 0;

var setTempRet0 = function(value) {
  tempRet0 = value;
};

var getTempRet0 = function() {
  return tempRet0;
};

function getCompilerSetting(name) {
  throw 'You must build with -s RETAIN_COMPILER_SETTINGS=1 for getCompilerSetting or emscripten_get_compiler_setting to work';
}

// The address globals begin at. Very low in memory, for code size and optimization opportunities.
// Above 0 is static memory, starting with globals.
// Then the stack.
// Then 'dynamic' memory for sbrk.
var GLOBAL_BASE = 1024;





// === Preamble library stuff ===

// Documentation for the public APIs defined in this file must be updated in:
//    site/source/docs/api_reference/preamble.js.rst
// A prebuilt local version of the documentation is available at:
//    site/build/text/docs/api_reference/preamble.js.txt
// You can also build docs locally as HTML or other formats in site/
// An online HTML version (which may be of a different version of Emscripten)
//    is up at http://kripken.github.io/emscripten-site/docs/api_reference/preamble.js.html


var wasmBinary;if (Module['wasmBinary']) wasmBinary = Module['wasmBinary'];if (!Object.getOwnPropertyDescriptor(Module, 'wasmBinary')) Object.defineProperty(Module, 'wasmBinary', { configurable: true, get: function() { abort('Module.wasmBinary has been replaced with plain wasmBinary (the initial value can be provided on Module, but after startup the value is only looked for on a local variable of that name)') } });
var noExitRuntime;if (Module['noExitRuntime']) noExitRuntime = Module['noExitRuntime'];if (!Object.getOwnPropertyDescriptor(Module, 'noExitRuntime')) Object.defineProperty(Module, 'noExitRuntime', { configurable: true, get: function() { abort('Module.noExitRuntime has been replaced with plain noExitRuntime (the initial value can be provided on Module, but after startup the value is only looked for on a local variable of that name)') } });




// wasm2js.js - enough of a polyfill for the WebAssembly object so that we can load
// wasm2js code that way.

// Emit "var WebAssembly" if definitely using wasm2js. Otherwise, in MAYBE_WASM2JS
// mode, we can't use a "var" since it would prevent normal wasm from working.
/** @suppress{const} */
var
WebAssembly = {
  // Note that we do not use closure quoting (this['buffer'], etc.) on these
  // functions, as they are just meant for internal use. In other words, this is
  // not a fully general polyfill.
  Memory: function(opts) {
    this.buffer = new ArrayBuffer(opts['initial'] * 65536);
    this.grow = function(amount) {
      var oldBuffer = this.buffer;
      var ret = __growWasmMemory(amount);
      assert(this.buffer !== oldBuffer); // the call should have updated us
      return ret;
    };
  },

  // Table is not a normal constructor and instead returns the array object.
  // That lets us use the length property automatically, which is simpler and
  // smaller (but instanceof will not report that an instance of Table is an
  // instance of this function).
  Table: /** @constructor */ function(opts) {
    var ret = new Array(opts['initial']);
    ret.grow = function(by) {
      abort('Unable to grow wasm table. Build with ALLOW_TABLE_GROWTH.')
    };
    ret.set = function(i, func) {
      ret[i] = func;
    };
    ret.get = function(i) {
      return ret[i];
    };
    return ret;
  },

  Module: function(binary) {
    // TODO: use the binary and info somehow - right now the wasm2js output is embedded in
    // the main JS
  },

  Instance: function(module, info) {
    // TODO: use the module and info somehow - right now the wasm2js output is embedded in
    // the main JS
    // This will be replaced by the actual wasm2js code.
    this.exports = (
function instantiate(asmLibraryArg, wasmMemory, wasmTable) {


  var scratchBuffer = new ArrayBuffer(16);
  var i32ScratchView = new Int32Array(scratchBuffer);
  var f32ScratchView = new Float32Array(scratchBuffer);
  var f64ScratchView = new Float64Array(scratchBuffer);
  
  function wasm2js_scratch_load_i32(index) {
    return i32ScratchView[index];
  }
      
  function wasm2js_scratch_store_i32(index, value) {
    i32ScratchView[index] = value;
  }
      
  function wasm2js_scratch_load_f64() {
    return f64ScratchView[0];
  }
      
  function wasm2js_scratch_store_f64(value) {
    f64ScratchView[0] = value;
  }
      
function asmFunc(global, env, buffer) {
 var memory = env.memory;
 var FUNCTION_TABLE = wasmTable;
 var HEAP8 = new global.Int8Array(buffer);
 var HEAP16 = new global.Int16Array(buffer);
 var HEAP32 = new global.Int32Array(buffer);
 var HEAPU8 = new global.Uint8Array(buffer);
 var HEAPU16 = new global.Uint16Array(buffer);
 var HEAPU32 = new global.Uint32Array(buffer);
 var HEAPF32 = new global.Float32Array(buffer);
 var HEAPF64 = new global.Float64Array(buffer);
 var Math_imul = global.Math.imul;
 var Math_fround = global.Math.fround;
 var Math_abs = global.Math.abs;
 var Math_clz32 = global.Math.clz32;
 var Math_min = global.Math.min;
 var Math_max = global.Math.max;
 var Math_floor = global.Math.floor;
 var Math_ceil = global.Math.ceil;
 var Math_sqrt = global.Math.sqrt;
 var abort = env.abort;
 var nan = global.NaN;
 var infinity = global.Infinity;
 var fimport$0 = env._embind_register_enum;
 var fimport$1 = env._embind_register_enum_value;
 var fimport$2 = env._embind_register_class;
 var fimport$3 = env._embind_register_class_property;
 var fimport$4 = env._embind_register_function;
 var fimport$5 = env.__assert_fail;
 var fimport$6 = env._embind_register_class_constructor;
 var fimport$7 = env._embind_register_class_function;
 var fimport$8 = env._embind_register_void;
 var fimport$9 = env._embind_register_bool;
 var fimport$10 = env._embind_register_std_string;
 var fimport$11 = env._embind_register_std_wstring;
 var fimport$12 = env._embind_register_emval;
 var fimport$13 = env._embind_register_integer;
 var fimport$14 = env._embind_register_float;
 var fimport$15 = env._embind_register_memory_view;
 var fimport$16 = env.abort;
 var fimport$17 = env.emscripten_resize_heap;
 var fimport$18 = env.emscripten_memcpy_big;
 var global$0 = 5248128;
 var global$1 = 5240;
 var i64toi32_i32$HIGH_BITS = 0;
 // EMSCRIPTEN_START_FUNCS
;
 function $0() {
  $144();
  $196();
  $325();
 }
 
 function $1() {
  FUNCTION_TABLE[1 | 0](4712) | 0;
  return;
 }
 
 function $2($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $10_1 = 0, $14_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  $10_1 = $3_1 + 8 | 0;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $14_1 = HEAP32[($3_1 + 12 | 0) >> 2] | 0;
  $3($10_1 | 0, 1024 | 0) | 0;
  $4($4($4($10_1 | 0, 1030 | 0, 0 | 0) | 0 | 0, 1035 | 0, 1 | 0) | 0 | 0, 1042 | 0, 2 | 0) | 0;
  global$0 = $3_1 + 16 | 0;
  return $14_1 | 0;
 }
 
 function $3($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  var $4_1 = 0, $7_1 = 0;
  $4_1 = global$0 - 16 | 0;
  global$0 = $4_1;
  HEAP32[($4_1 + 12 | 0) >> 2] = $0_1;
  HEAP32[($4_1 + 8 | 0) >> 2] = $1_1;
  $7_1 = HEAP32[($4_1 + 12 | 0) >> 2] | 0;
  fimport$0($5() | 0 | 0, HEAP32[($4_1 + 8 | 0) >> 2] | 0 | 0, 4 | 0, 0 & 1 | 0 | 0);
  global$0 = $4_1 + 16 | 0;
  return $7_1 | 0;
 }
 
 function $4($0_1, $1_1, $2_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  var $5_1 = 0, $6_1 = 0;
  $5_1 = global$0 - 16 | 0;
  global$0 = $5_1;
  HEAP32[($5_1 + 12 | 0) >> 2] = $0_1;
  HEAP32[($5_1 + 8 | 0) >> 2] = $1_1;
  HEAP32[($5_1 + 4 | 0) >> 2] = $2_1;
  $6_1 = HEAP32[($5_1 + 12 | 0) >> 2] | 0;
  fimport$1($5() | 0 | 0, HEAP32[($5_1 + 8 | 0) >> 2] | 0 | 0, HEAP32[($5_1 + 4 | 0) >> 2] | 0 | 0);
  global$0 = $5_1 + 16 | 0;
  return $6_1 | 0;
 }
 
 function $5() {
  return $71() | 0 | 0;
 }
 
 function $6($0_1, $1_1, $2_1, $3_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  $7($0_1 | 0, $1_1 | 0, $2_1 | 0, $3_1 | 0);
  return;
 }
 
 function $7($0_1, $1_1, $2_1, $3_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  var $6_1 = 0;
  $6_1 = global$0 - 16 | 0;
  global$0 = $6_1;
  HEAP32[($6_1 + 12 | 0) >> 2] = $1_1;
  HEAP32[($6_1 + 8 | 0) >> 2] = $2_1;
  HEAP32[($6_1 + 4 | 0) >> 2] = $3_1;
  $8($0_1 | 0, HEAP32[($6_1 + 12 | 0) >> 2] | 0 | 0, HEAP32[($6_1 + 8 | 0) >> 2] | 0 | 0, HEAP32[($6_1 + 4 | 0) >> 2] | 0 | 0);
  global$0 = $6_1 + 16 | 0;
  return;
 }
 
 function $8($0_1, $1_1, $2_1, $3_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  var $6_1 = 0, $7_1 = 0, $12_1 = 0, $16_1 = 0, $11_1 = 0, $8_1 = 0, $19_1 = 0, $15_1 = 0, $25_1 = 0, $28_1 = 0, $62_1 = Math_fround(0), $63_1 = Math_fround(0), $64_1 = Math_fround(0), $65_1 = Math_fround(0), $66_1 = Math_fround(0), $67_1 = Math_fround(0), $69_1 = Math_fround(0), $71_1 = Math_fround(0), $73_1 = Math_fround(0), $75_1 = Math_fround(0), $77_1 = Math_fround(0), $78_1 = Math_fround(0);
  $6_1 = global$0 - 96 | 0;
  global$0 = $6_1;
  $7_1 = 2;
  $8_1 = 3;
  $11_1 = $6_1 + 72 | 0;
  $12_1 = 1;
  $15_1 = $6_1 + 8 | 0;
  $16_1 = 0;
  $19_1 = $6_1 + 40 | 0;
  $25_1 = $6_1 + 24 | 0;
  $28_1 = $6_1 + 56 | 0;
  HEAP32[($6_1 + 92 | 0) >> 2] = $1_1;
  HEAP32[($6_1 + 88 | 0) >> 2] = $2_1;
  HEAP32[($6_1 + 84 | 0) >> 2] = $3_1;
  $72($28_1 | 0, HEAP32[($6_1 + 88 | 0) >> 2] | 0 | 0, HEAP32[($6_1 + 92 | 0) >> 2] | 0 | 0);
  $73($11_1 | 0, $28_1 | 0);
  $74($25_1 | 0, $11_1 | 0, HEAP32[($6_1 + 84 | 0) >> 2] | 0 | 0);
  $73($19_1 | 0, $25_1 | 0);
  $74($15_1 | 0, $19_1 | 0, $11_1 | 0);
  HEAPF32[($6_1 + 4 | 0) >> 2] = Math_fround(1.0);
  $75($0_1 | 0, $6_1 + 4 | 0 | 0) | 0;
  $62_1 = Math_fround(HEAPF32[($6_1 + 40 | 0) >> 2]);
  HEAPF32[($77($76($0_1 | 0, $16_1 | 0) | 0 | 0, $16_1 | 0) | 0) >> 2] = $62_1;
  $63_1 = Math_fround(HEAPF32[($6_1 + 44 | 0) >> 2]);
  HEAPF32[($77($76($0_1 | 0, $12_1 | 0) | 0 | 0, $16_1 | 0) | 0) >> 2] = $63_1;
  $64_1 = Math_fround(HEAPF32[($6_1 + 48 | 0) >> 2]);
  HEAPF32[($77($76($0_1 | 0, $7_1 | 0) | 0 | 0, $16_1 | 0) | 0) >> 2] = $64_1;
  $65_1 = Math_fround(HEAPF32[($6_1 + 8 | 0) >> 2]);
  HEAPF32[($77($76($0_1 | 0, $16_1 | 0) | 0 | 0, $12_1 | 0) | 0) >> 2] = $65_1;
  $66_1 = Math_fround(HEAPF32[($6_1 + 12 | 0) >> 2]);
  HEAPF32[($77($76($0_1 | 0, $12_1 | 0) | 0 | 0, $12_1 | 0) | 0) >> 2] = $66_1;
  $67_1 = Math_fround(HEAPF32[($6_1 + 16 | 0) >> 2]);
  HEAPF32[($77($76($0_1 | 0, $7_1 | 0) | 0 | 0, $12_1 | 0) | 0) >> 2] = $67_1;
  $69_1 = Math_fround(-Math_fround(HEAPF32[($6_1 + 72 | 0) >> 2]));
  HEAPF32[($77($76($0_1 | 0, $16_1 | 0) | 0 | 0, $7_1 | 0) | 0) >> 2] = $69_1;
  $71_1 = Math_fround(-Math_fround(HEAPF32[($6_1 + 76 | 0) >> 2]));
  HEAPF32[($77($76($0_1 | 0, $12_1 | 0) | 0 | 0, $7_1 | 0) | 0) >> 2] = $71_1;
  $73_1 = Math_fround(-Math_fround(HEAPF32[($6_1 + 80 | 0) >> 2]));
  HEAPF32[($77($76($0_1 | 0, $7_1 | 0) | 0 | 0, $7_1 | 0) | 0) >> 2] = $73_1;
  $75_1 = Math_fround(-Math_fround($11($19_1 | 0, HEAP32[($6_1 + 92 | 0) >> 2] | 0 | 0)));
  HEAPF32[($77($76($0_1 | 0, $8_1 | 0) | 0 | 0, $16_1 | 0) | 0) >> 2] = $75_1;
  $77_1 = Math_fround(-Math_fround($11($15_1 | 0, HEAP32[($6_1 + 92 | 0) >> 2] | 0 | 0)));
  HEAPF32[($77($76($0_1 | 0, $8_1 | 0) | 0 | 0, $12_1 | 0) | 0) >> 2] = $77_1;
  $78_1 = Math_fround($11($11_1 | 0, HEAP32[($6_1 + 92 | 0) >> 2] | 0 | 0));
  HEAPF32[($77($76($0_1 | 0, $8_1 | 0) | 0 | 0, $7_1 | 0) | 0) >> 2] = $78_1;
  global$0 = $6_1 + 96 | 0;
  return;
 }
 
 function $9($0_1) {
  $0_1 = $0_1 | 0;
  return Math_fround(Math_fround($10($0_1 | 0)));
 }
 
 function $10($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $9_1 = Math_fround(0);
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $9_1 = Math_fround($12(Math_fround(Math_fround($11(HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0)))));
  global$0 = $3_1 + 16 | 0;
  return Math_fround($9_1);
 }
 
 function $11($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  var $4_1 = 0, $9_1 = Math_fround(0);
  $4_1 = global$0 - 16 | 0;
  global$0 = $4_1;
  HEAP32[($4_1 + 12 | 0) >> 2] = $0_1;
  HEAP32[($4_1 + 8 | 0) >> 2] = $1_1;
  $9_1 = Math_fround($88(HEAP32[($4_1 + 12 | 0) >> 2] | 0 | 0, HEAP32[($4_1 + 8 | 0) >> 2] | 0 | 0));
  global$0 = $4_1 + 16 | 0;
  return Math_fround($9_1);
 }
 
 function $12($0_1) {
  $0_1 = Math_fround($0_1);
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  HEAPF32[($3_1 + 12 | 0) >> 2] = $0_1;
  return Math_fround(Math_fround(Math_sqrt(Math_fround(HEAPF32[($3_1 + 12 | 0) >> 2]))));
 }
 
 function $13() {
  FUNCTION_TABLE[2 | 0](4713) | 0;
  return;
 }
 
 function $14($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $18_1 = 0, $19_1 = 0, $11_1 = 0, $17_1 = 0, $24_1 = 0, $25_1 = 0, $26_1 = 0, $29_1 = 0, $30_1 = 0, $31_1 = 0, $36_1 = 0, $42_1 = 0, $43_1 = 0, $44_1 = 0, $47_1 = 0, $52_1 = 0, $56_1 = 0, $59_1 = 0, $60_1 = 0, $61_1 = 0, $62_1 = 0, $64_1 = 0, $65_1 = 0, $67_1 = 0, $68_1 = 0, $70_1 = 0, $71_1 = 0, $72_1 = 0, $77_1 = 0, $80_1 = 0, $82_1 = 0, $83_1 = 0, $84_1 = 0, $85_1 = 0, $87_1 = 0, $88_1 = 0, $89_1 = 0, $90_1 = 0, $95_1 = 0, $96_1 = 0, $97_1 = 0, $99_1 = 0, $100_1 = 0, $101_1 = 0, $102_1 = 0, $109_1 = 0, $110_1 = 0, $111_1 = 0, $112_1 = 0, $114_1 = 0, $115_1 = 0, $117_1 = 0, $118_1 = 0, $120_1 = 0, $121_1 = 0, $122_1 = 0, $127_1 = 0, $130_1 = 0, $132_1 = 0, $133_1 = 0, $134_1 = 0, $135_1 = 0, $137_1 = 0, $138_1 = 0, $139_1 = 0, $140_1 = 0, $145_1 = 0, $146_1 = 0, $147_1 = 0, $148_1 = 0, $150_1 = 0, $151_1 = 0, $152_1 = 0, $153_1 = 0, $158_1 = 0, $159_1 = 0, $160_1 = 0, $162_1 = 0, $163_1 = 0, $164_1 = 0, $165_1 = 0, $172_1 = 0, $173_1 = 0, $174_1 = 0, $175_1 = 0, $177_1 = 0, $178_1 = 0, $180_1 = 0, $181_1 = 0, $183_1 = 0, $184_1 = 0, $185_1 = 0;
  $3_1 = global$0 - 352 | 0;
  global$0 = $3_1;
  $11_1 = $3_1 + 8 | 0;
  $17_1 = $3_1 + 280 | 0;
  $18_1 = 8;
  $19_1 = 9;
  $24_1 = $3_1 + 252 | 0;
  $25_1 = 4;
  $26_1 = 1058;
  $29_1 = $3_1 + 224 | 0;
  $30_1 = 0;
  $31_1 = 1056;
  $36_1 = $3_1 + 16 | 0;
  $42_1 = $3_1 + 132 | 0;
  $43_1 = 14;
  $44_1 = 15;
  $47_1 = $3_1 + 104 | 0;
  $52_1 = $3_1 + 24 | 0;
  HEAP32[($3_1 + 28 | 0) >> 2] = $0_1;
  $56_1 = HEAP32[($3_1 + 28 | 0) >> 2] | 0;
  HEAP32[($3_1 + 52 | 0) >> 2] = $52_1;
  HEAP32[($3_1 + 48 | 0) >> 2] = 1048;
  $30();
  HEAP32[($3_1 + 44 | 0) >> 2] = 19;
  HEAP32[($3_1 + 40 | 0) >> 2] = $31() | 0;
  HEAP32[($3_1 + 36 | 0) >> 2] = $32() | 0;
  HEAP32[($3_1 + 32 | 0) >> 2] = 18;
  $59_1 = $33() | 0;
  $60_1 = $34() | 0;
  $61_1 = $35() | 0;
  $62_1 = $36() | 0;
  HEAP32[($3_1 + 56 | 0) >> 2] = HEAP32[($3_1 + 44 | 0) >> 2] | 0;
  $64_1 = $37() | 0;
  $65_1 = HEAP32[($3_1 + 44 | 0) >> 2] | 0;
  HEAP32[($3_1 + 60 | 0) >> 2] = HEAP32[($3_1 + 40 | 0) >> 2] | 0;
  $67_1 = $38() | 0;
  $68_1 = HEAP32[($3_1 + 40 | 0) >> 2] | 0;
  HEAP32[($3_1 + 64 | 0) >> 2] = HEAP32[($3_1 + 36 | 0) >> 2] | 0;
  $70_1 = $38() | 0;
  $71_1 = HEAP32[($3_1 + 36 | 0) >> 2] | 0;
  $72_1 = HEAP32[($3_1 + 48 | 0) >> 2] | 0;
  HEAP32[($3_1 + 68 | 0) >> 2] = HEAP32[($3_1 + 32 | 0) >> 2] | 0;
  fimport$2($59_1 | 0, $60_1 | 0, $61_1 | 0, $62_1 | 0, $64_1 | 0, $65_1 | 0, $67_1 | 0, $68_1 | 0, $70_1 | 0, $71_1 | 0, $72_1 | 0, $39() | 0 | 0, HEAP32[($3_1 + 32 | 0) >> 2] | 0 | 0);
  HEAP32[($3_1 + 72 | 0) >> 2] = $52_1;
  HEAP32[($3_1 + 80 | 0) >> 2] = HEAP32[($3_1 + 72 | 0) >> 2] | 0;
  HEAP32[($3_1 + 76 | 0) >> 2] = 17;
  $77_1 = HEAP32[($3_1 + 80 | 0) >> 2] | 0;
  $40(HEAP32[($3_1 + 76 | 0) >> 2] | 0 | 0);
  HEAP32[($3_1 + 84 | 0) >> 2] = $77_1;
  HEAP32[($3_1 + 92 | 0) >> 2] = HEAP32[($3_1 + 84 | 0) >> 2] | 0;
  HEAP32[($3_1 + 88 | 0) >> 2] = 16;
  $80_1 = HEAP32[($3_1 + 92 | 0) >> 2] | 0;
  $41(HEAP32[($3_1 + 88 | 0) >> 2] | 0 | 0);
  HEAP32[($3_1 + 112 | 0) >> 2] = $80_1;
  HEAP32[($3_1 + 108 | 0) >> 2] = $31_1;
  HEAP32[($3_1 + 104 | 0) >> 2] = $30_1;
  $82_1 = HEAP32[($3_1 + 112 | 0) >> 2] | 0;
  HEAP32[($3_1 + 100 | 0) >> 2] = $44_1;
  HEAP32[($3_1 + 96 | 0) >> 2] = $43_1;
  $83_1 = $33() | 0;
  $84_1 = HEAP32[($3_1 + 108 | 0) >> 2] | 0;
  $85_1 = $42() | 0;
  HEAP32[($3_1 + 116 | 0) >> 2] = HEAP32[($3_1 + 100 | 0) >> 2] | 0;
  $87_1 = $43() | 0;
  $88_1 = HEAP32[($3_1 + 100 | 0) >> 2] | 0;
  $89_1 = $44($47_1 | 0) | 0;
  $90_1 = $42() | 0;
  HEAP32[($3_1 + 120 | 0) >> 2] = HEAP32[($3_1 + 96 | 0) >> 2] | 0;
  fimport$3($83_1 | 0, $84_1 | 0, $85_1 | 0, $87_1 | 0, $88_1 | 0, $89_1 | 0, $90_1 | 0, $45() | 0 | 0, HEAP32[($3_1 + 96 | 0) >> 2] | 0 | 0, $44($47_1 | 0) | 0 | 0);
  HEAP32[($3_1 + 140 | 0) >> 2] = $82_1;
  HEAP32[($3_1 + 136 | 0) >> 2] = $26_1;
  HEAP32[($3_1 + 132 | 0) >> 2] = $25_1;
  HEAP32[($3_1 + 128 | 0) >> 2] = $44_1;
  HEAP32[($3_1 + 124 | 0) >> 2] = $43_1;
  $95_1 = $33() | 0;
  $96_1 = HEAP32[($3_1 + 136 | 0) >> 2] | 0;
  $97_1 = $42() | 0;
  HEAP32[($3_1 + 144 | 0) >> 2] = HEAP32[($3_1 + 128 | 0) >> 2] | 0;
  $99_1 = $43() | 0;
  $100_1 = HEAP32[($3_1 + 128 | 0) >> 2] | 0;
  $101_1 = $44($42_1 | 0) | 0;
  $102_1 = $42() | 0;
  HEAP32[($3_1 + 148 | 0) >> 2] = HEAP32[($3_1 + 124 | 0) >> 2] | 0;
  fimport$3($95_1 | 0, $96_1 | 0, $97_1 | 0, $99_1 | 0, $100_1 | 0, $101_1 | 0, $102_1 | 0, $45() | 0 | 0, HEAP32[($3_1 + 124 | 0) >> 2] | 0 | 0, $44($42_1 | 0) | 0 | 0);
  HEAP32[($3_1 + 172 | 0) >> 2] = $36_1;
  HEAP32[($3_1 + 168 | 0) >> 2] = 1060;
  $46();
  HEAP32[($3_1 + 164 | 0) >> 2] = 13;
  HEAP32[($3_1 + 160 | 0) >> 2] = $47() | 0;
  HEAP32[($3_1 + 156 | 0) >> 2] = $48() | 0;
  HEAP32[($3_1 + 152 | 0) >> 2] = 12;
  $109_1 = $49() | 0;
  $110_1 = $50() | 0;
  $111_1 = $51() | 0;
  $112_1 = $36() | 0;
  HEAP32[($3_1 + 176 | 0) >> 2] = HEAP32[($3_1 + 164 | 0) >> 2] | 0;
  $114_1 = $37() | 0;
  $115_1 = HEAP32[($3_1 + 164 | 0) >> 2] | 0;
  HEAP32[($3_1 + 180 | 0) >> 2] = HEAP32[($3_1 + 160 | 0) >> 2] | 0;
  $117_1 = $38() | 0;
  $118_1 = HEAP32[($3_1 + 160 | 0) >> 2] | 0;
  HEAP32[($3_1 + 184 | 0) >> 2] = HEAP32[($3_1 + 156 | 0) >> 2] | 0;
  $120_1 = $38() | 0;
  $121_1 = HEAP32[($3_1 + 156 | 0) >> 2] | 0;
  $122_1 = HEAP32[($3_1 + 168 | 0) >> 2] | 0;
  HEAP32[($3_1 + 188 | 0) >> 2] = HEAP32[($3_1 + 152 | 0) >> 2] | 0;
  fimport$2($109_1 | 0, $110_1 | 0, $111_1 | 0, $112_1 | 0, $114_1 | 0, $115_1 | 0, $117_1 | 0, $118_1 | 0, $120_1 | 0, $121_1 | 0, $122_1 | 0, $39() | 0 | 0, HEAP32[($3_1 + 152 | 0) >> 2] | 0 | 0);
  HEAP32[($3_1 + 192 | 0) >> 2] = $36_1;
  HEAP32[($3_1 + 200 | 0) >> 2] = HEAP32[($3_1 + 192 | 0) >> 2] | 0;
  HEAP32[($3_1 + 196 | 0) >> 2] = 11;
  $127_1 = HEAP32[($3_1 + 200 | 0) >> 2] | 0;
  $52(HEAP32[($3_1 + 196 | 0) >> 2] | 0 | 0);
  HEAP32[($3_1 + 204 | 0) >> 2] = $127_1;
  HEAP32[($3_1 + 212 | 0) >> 2] = HEAP32[($3_1 + 204 | 0) >> 2] | 0;
  HEAP32[($3_1 + 208 | 0) >> 2] = 10;
  $130_1 = HEAP32[($3_1 + 212 | 0) >> 2] | 0;
  $53(HEAP32[($3_1 + 208 | 0) >> 2] | 0 | 0);
  HEAP32[($3_1 + 232 | 0) >> 2] = $130_1;
  HEAP32[($3_1 + 228 | 0) >> 2] = $31_1;
  HEAP32[($3_1 + 224 | 0) >> 2] = $30_1;
  $132_1 = HEAP32[($3_1 + 232 | 0) >> 2] | 0;
  HEAP32[($3_1 + 220 | 0) >> 2] = $19_1;
  HEAP32[($3_1 + 216 | 0) >> 2] = $18_1;
  $133_1 = $49() | 0;
  $134_1 = HEAP32[($3_1 + 228 | 0) >> 2] | 0;
  $135_1 = $42() | 0;
  HEAP32[($3_1 + 236 | 0) >> 2] = HEAP32[($3_1 + 220 | 0) >> 2] | 0;
  $137_1 = $43() | 0;
  $138_1 = HEAP32[($3_1 + 220 | 0) >> 2] | 0;
  $139_1 = $54($29_1 | 0) | 0;
  $140_1 = $42() | 0;
  HEAP32[($3_1 + 240 | 0) >> 2] = HEAP32[($3_1 + 216 | 0) >> 2] | 0;
  fimport$3($133_1 | 0, $134_1 | 0, $135_1 | 0, $137_1 | 0, $138_1 | 0, $139_1 | 0, $140_1 | 0, $45() | 0 | 0, HEAP32[($3_1 + 216 | 0) >> 2] | 0 | 0, $54($29_1 | 0) | 0 | 0);
  HEAP32[($3_1 + 260 | 0) >> 2] = $132_1;
  HEAP32[($3_1 + 256 | 0) >> 2] = $26_1;
  HEAP32[($3_1 + 252 | 0) >> 2] = $25_1;
  $145_1 = HEAP32[($3_1 + 260 | 0) >> 2] | 0;
  HEAP32[($3_1 + 248 | 0) >> 2] = $19_1;
  HEAP32[($3_1 + 244 | 0) >> 2] = $18_1;
  $146_1 = $49() | 0;
  $147_1 = HEAP32[($3_1 + 256 | 0) >> 2] | 0;
  $148_1 = $42() | 0;
  HEAP32[($3_1 + 264 | 0) >> 2] = HEAP32[($3_1 + 248 | 0) >> 2] | 0;
  $150_1 = $43() | 0;
  $151_1 = HEAP32[($3_1 + 248 | 0) >> 2] | 0;
  $152_1 = $54($24_1 | 0) | 0;
  $153_1 = $42() | 0;
  HEAP32[($3_1 + 268 | 0) >> 2] = HEAP32[($3_1 + 244 | 0) >> 2] | 0;
  fimport$3($146_1 | 0, $147_1 | 0, $148_1 | 0, $150_1 | 0, $151_1 | 0, $152_1 | 0, $153_1 | 0, $45() | 0 | 0, HEAP32[($3_1 + 244 | 0) >> 2] | 0 | 0, $54($24_1 | 0) | 0 | 0);
  HEAP32[($3_1 + 288 | 0) >> 2] = $145_1;
  HEAP32[($3_1 + 284 | 0) >> 2] = 1068;
  HEAP32[($3_1 + 280 | 0) >> 2] = 8;
  HEAP32[($3_1 + 276 | 0) >> 2] = $19_1;
  HEAP32[($3_1 + 272 | 0) >> 2] = $18_1;
  $158_1 = $49() | 0;
  $159_1 = HEAP32[($3_1 + 284 | 0) >> 2] | 0;
  $160_1 = $42() | 0;
  HEAP32[($3_1 + 292 | 0) >> 2] = HEAP32[($3_1 + 276 | 0) >> 2] | 0;
  $162_1 = $43() | 0;
  $163_1 = HEAP32[($3_1 + 276 | 0) >> 2] | 0;
  $164_1 = $54($17_1 | 0) | 0;
  $165_1 = $42() | 0;
  HEAP32[($3_1 + 296 | 0) >> 2] = HEAP32[($3_1 + 272 | 0) >> 2] | 0;
  fimport$3($158_1 | 0, $159_1 | 0, $160_1 | 0, $162_1 | 0, $163_1 | 0, $164_1 | 0, $165_1 | 0, $45() | 0 | 0, HEAP32[($3_1 + 272 | 0) >> 2] | 0 | 0, $54($17_1 | 0) | 0 | 0);
  HEAP32[($3_1 + 320 | 0) >> 2] = $11_1;
  HEAP32[($3_1 + 316 | 0) >> 2] = 1070;
  $55();
  HEAP32[($3_1 + 312 | 0) >> 2] = 7;
  HEAP32[($3_1 + 308 | 0) >> 2] = $56() | 0;
  HEAP32[($3_1 + 304 | 0) >> 2] = $57() | 0;
  HEAP32[($3_1 + 300 | 0) >> 2] = 6;
  $172_1 = $58() | 0;
  $173_1 = $59() | 0;
  $174_1 = $60() | 0;
  $175_1 = $36() | 0;
  HEAP32[($3_1 + 324 | 0) >> 2] = HEAP32[($3_1 + 312 | 0) >> 2] | 0;
  $177_1 = $37() | 0;
  $178_1 = HEAP32[($3_1 + 312 | 0) >> 2] | 0;
  HEAP32[($3_1 + 328 | 0) >> 2] = HEAP32[($3_1 + 308 | 0) >> 2] | 0;
  $180_1 = $38() | 0;
  $181_1 = HEAP32[($3_1 + 308 | 0) >> 2] | 0;
  HEAP32[($3_1 + 332 | 0) >> 2] = HEAP32[($3_1 + 304 | 0) >> 2] | 0;
  $183_1 = $38() | 0;
  $184_1 = HEAP32[($3_1 + 304 | 0) >> 2] | 0;
  $185_1 = HEAP32[($3_1 + 316 | 0) >> 2] | 0;
  HEAP32[($3_1 + 336 | 0) >> 2] = HEAP32[($3_1 + 300 | 0) >> 2] | 0;
  fimport$2($172_1 | 0, $173_1 | 0, $174_1 | 0, $175_1 | 0, $177_1 | 0, $178_1 | 0, $180_1 | 0, $181_1 | 0, $183_1 | 0, $184_1 | 0, $185_1 | 0, $39() | 0 | 0, HEAP32[($3_1 + 300 | 0) >> 2] | 0 | 0);
  HEAP32[($3_1 + 340 | 0) >> 2] = $11_1;
  HEAP32[($3_1 + 348 | 0) >> 2] = HEAP32[($3_1 + 340 | 0) >> 2] | 0;
  HEAP32[($3_1 + 344 | 0) >> 2] = 5;
  $61(HEAP32[($3_1 + 344 | 0) >> 2] | 0 | 0);
  $62(1075 | 0, 4 | 0);
  $63(1082 | 0, 3 | 0);
  global$0 = $3_1 + 352 | 0;
  return $56_1 | 0;
 }
 
 function $15($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $8_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $8_1 = $330(64 | 0) | 0;
  HEAPF32[($3_1 + 8 | 0) >> 2] = Math_fround(+HEAPF64[($101(HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0) | 0) >> 3]);
  $75($8_1 | 0, $3_1 + 8 | 0 | 0) | 0;
  global$0 = $3_1 + 16 | 0;
  return $8_1 | 0;
 }
 
 function $16($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $5_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $5_1 = HEAP32[($3_1 + 12 | 0) >> 2] | 0;
  label$1 : {
   if (($5_1 | 0) == (0 | 0) & 1 | 0) {
    break label$1
   }
   $331($5_1 | 0);
  }
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $17($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $5_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $5_1 = $129(HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0) | 0;
  global$0 = $3_1 + 16 | 0;
  return $5_1 | 0;
 }
 
 function $18($0_1, $1_1, $2_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = Math_fround($2_1);
  var $5_1 = 0, $13_1 = Math_fround(0);
  $5_1 = global$0 - 16 | 0;
  global$0 = $5_1;
  HEAP32[($5_1 + 12 | 0) >> 2] = $0_1;
  HEAP32[($5_1 + 8 | 0) >> 2] = $1_1;
  HEAPF32[($5_1 + 4 | 0) >> 2] = $2_1;
  $13_1 = Math_fround($111(Math_fround(Math_fround(HEAPF32[($5_1 + 4 | 0) >> 2]))));
  HEAPF32[((HEAP32[($5_1 + 8 | 0) >> 2] | 0) + (HEAP32[(HEAP32[($5_1 + 12 | 0) >> 2] | 0) >> 2] | 0) | 0) >> 2] = $13_1;
  global$0 = $5_1 + 16 | 0;
  return;
 }
 
 function $19($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  var $4_1 = 0, $11_1 = Math_fround(0);
  $4_1 = global$0 - 16 | 0;
  global$0 = $4_1;
  HEAP32[($4_1 + 12 | 0) >> 2] = $0_1;
  HEAP32[($4_1 + 8 | 0) >> 2] = $1_1;
  $11_1 = Math_fround($110((HEAP32[($4_1 + 8 | 0) >> 2] | 0) + (HEAP32[(HEAP32[($4_1 + 12 | 0) >> 2] | 0) >> 2] | 0) | 0 | 0));
  global$0 = $4_1 + 16 | 0;
  return Math_fround($11_1);
 }
 
 function $20($0_1, $1_1, $2_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  var $5_1 = 0, $7_1 = 0;
  $5_1 = global$0 - 16 | 0;
  global$0 = $5_1;
  HEAP32[($5_1 + 12 | 0) >> 2] = $0_1;
  HEAP32[($5_1 + 8 | 0) >> 2] = $1_1;
  HEAP32[($5_1 + 4 | 0) >> 2] = $2_1;
  $7_1 = $330(12 | 0) | 0;
  $123($7_1 | 0, $101(HEAP32[($5_1 + 12 | 0) >> 2] | 0 | 0) | 0 | 0, $101(HEAP32[($5_1 + 8 | 0) >> 2] | 0 | 0) | 0 | 0, $101(HEAP32[($5_1 + 4 | 0) >> 2] | 0 | 0) | 0 | 0) | 0;
  global$0 = $5_1 + 16 | 0;
  return $7_1 | 0;
 }
 
 function $21() {
  var $1_1 = 0;
  $1_1 = $330(12 | 0) | 0;
  $117($1_1 | 0) | 0;
  return $1_1 | 0;
 }
 
 function $22($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $5_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $5_1 = HEAP32[($3_1 + 12 | 0) >> 2] | 0;
  label$1 : {
   if (($5_1 | 0) == (0 | 0) & 1 | 0) {
    break label$1
   }
   $331($5_1 | 0);
  }
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $23($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $5_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $5_1 = $113(HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0) | 0;
  global$0 = $3_1 + 16 | 0;
  return $5_1 | 0;
 }
 
 function $24($0_1, $1_1, $2_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = Math_fround($2_1);
  var $5_1 = 0, $13_1 = Math_fround(0);
  $5_1 = global$0 - 16 | 0;
  global$0 = $5_1;
  HEAP32[($5_1 + 12 | 0) >> 2] = $0_1;
  HEAP32[($5_1 + 8 | 0) >> 2] = $1_1;
  HEAPF32[($5_1 + 4 | 0) >> 2] = $2_1;
  $13_1 = Math_fround($111(Math_fround(Math_fround(HEAPF32[($5_1 + 4 | 0) >> 2]))));
  HEAPF32[((HEAP32[($5_1 + 8 | 0) >> 2] | 0) + (HEAP32[(HEAP32[($5_1 + 12 | 0) >> 2] | 0) >> 2] | 0) | 0) >> 2] = $13_1;
  global$0 = $5_1 + 16 | 0;
  return;
 }
 
 function $25($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  var $4_1 = 0, $11_1 = Math_fround(0);
  $4_1 = global$0 - 16 | 0;
  global$0 = $4_1;
  HEAP32[($4_1 + 12 | 0) >> 2] = $0_1;
  HEAP32[($4_1 + 8 | 0) >> 2] = $1_1;
  $11_1 = Math_fround($110((HEAP32[($4_1 + 8 | 0) >> 2] | 0) + (HEAP32[(HEAP32[($4_1 + 12 | 0) >> 2] | 0) >> 2] | 0) | 0 | 0));
  global$0 = $4_1 + 16 | 0;
  return Math_fround($11_1);
 }
 
 function $26($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  var $4_1 = 0, $6_1 = 0;
  $4_1 = global$0 - 16 | 0;
  global$0 = $4_1;
  HEAP32[($4_1 + 12 | 0) >> 2] = $0_1;
  HEAP32[($4_1 + 8 | 0) >> 2] = $1_1;
  $6_1 = $330(8 | 0) | 0;
  $102($6_1 | 0, $101(HEAP32[($4_1 + 12 | 0) >> 2] | 0 | 0) | 0 | 0, $101(HEAP32[($4_1 + 8 | 0) >> 2] | 0 | 0) | 0 | 0) | 0;
  global$0 = $4_1 + 16 | 0;
  return $6_1 | 0;
 }
 
 function $27() {
  var $1_1 = 0;
  $1_1 = $330(8 | 0) | 0;
  $95($1_1 | 0) | 0;
  return $1_1 | 0;
 }
 
 function $28($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $5_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $5_1 = HEAP32[($3_1 + 12 | 0) >> 2] | 0;
  label$1 : {
   if (($5_1 | 0) == (0 | 0) & 1 | 0) {
    break label$1
   }
   $331($5_1 | 0);
  }
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $29($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $5_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $5_1 = $91(HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0) | 0;
  global$0 = $3_1 + 16 | 0;
  return $5_1 | 0;
 }
 
 function $30() {
  return;
 }
 
 function $31() {
  return 0 | 0;
 }
 
 function $32() {
  return 0 | 0;
 }
 
 function $33() {
  return $92() | 0 | 0;
 }
 
 function $34() {
  return $93() | 0 | 0;
 }
 
 function $35() {
  return $94() | 0 | 0;
 }
 
 function $36() {
  return 0 | 0;
 }
 
 function $37() {
  return 1424 | 0;
 }
 
 function $38() {
  return 1427 | 0;
 }
 
 function $39() {
  return 1429 | 0;
 }
 
 function $40($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $6_1 = 0, $8_1 = 0, $9_1 = 0, $10_1 = 0;
  $3_1 = global$0 - 32 | 0;
  global$0 = $3_1;
  $6_1 = $3_1 + 16 | 0;
  HEAP32[($3_1 + 24 | 0) >> 2] = $0_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = 20;
  $8_1 = $33() | 0;
  $9_1 = $97($6_1 | 0) | 0;
  $10_1 = $98($6_1 | 0) | 0;
  HEAP32[($3_1 + 28 | 0) >> 2] = HEAP32[($3_1 + 12 | 0) >> 2] | 0;
  fimport$6($8_1 | 0, $9_1 | 0, $10_1 | 0, $37() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0, HEAP32[($3_1 + 24 | 0) >> 2] | 0 | 0);
  global$0 = $3_1 + 32 | 0;
  return;
 }
 
 function $41($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $6_1 = 0, $8_1 = 0, $9_1 = 0, $10_1 = 0;
  $3_1 = global$0 - 32 | 0;
  global$0 = $3_1;
  $6_1 = $3_1 + 16 | 0;
  HEAP32[($3_1 + 24 | 0) >> 2] = $0_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = 21;
  $8_1 = $33() | 0;
  $9_1 = $104($6_1 | 0) | 0;
  $10_1 = $105($6_1 | 0) | 0;
  HEAP32[($3_1 + 28 | 0) >> 2] = HEAP32[($3_1 + 12 | 0) >> 2] | 0;
  fimport$6($8_1 | 0, $9_1 | 0, $10_1 | 0, $106() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0, HEAP32[($3_1 + 24 | 0) >> 2] | 0 | 0);
  global$0 = $3_1 + 32 | 0;
  return;
 }
 
 function $42() {
  return $112() | 0 | 0;
 }
 
 function $43() {
  return 1453 | 0;
 }
 
 function $44($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $5_1 = 0, $8_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $5_1 = $330(4 | 0) | 0;
  HEAP32[$5_1 >> 2] = HEAP32[(HEAP32[($3_1 + 12 | 0) >> 2] | 0) >> 2] | 0;
  HEAP32[($3_1 + 8 | 0) >> 2] = $5_1;
  $8_1 = HEAP32[($3_1 + 8 | 0) >> 2] | 0;
  global$0 = $3_1 + 16 | 0;
  return $8_1 | 0;
 }
 
 function $45() {
  return 1457 | 0;
 }
 
 function $46() {
  return;
 }
 
 function $47() {
  return 0 | 0;
 }
 
 function $48() {
  return 0 | 0;
 }
 
 function $49() {
  return $114() | 0 | 0;
 }
 
 function $50() {
  return $115() | 0 | 0;
 }
 
 function $51() {
  return $116() | 0 | 0;
 }
 
 function $52($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $6_1 = 0, $8_1 = 0, $9_1 = 0, $10_1 = 0;
  $3_1 = global$0 - 32 | 0;
  global$0 = $3_1;
  $6_1 = $3_1 + 16 | 0;
  HEAP32[($3_1 + 24 | 0) >> 2] = $0_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = 22;
  $8_1 = $49() | 0;
  $9_1 = $119($6_1 | 0) | 0;
  $10_1 = $120($6_1 | 0) | 0;
  HEAP32[($3_1 + 28 | 0) >> 2] = HEAP32[($3_1 + 12 | 0) >> 2] | 0;
  fimport$6($8_1 | 0, $9_1 | 0, $10_1 | 0, $37() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0, HEAP32[($3_1 + 24 | 0) >> 2] | 0 | 0);
  global$0 = $3_1 + 32 | 0;
  return;
 }
 
 function $53($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $6_1 = 0, $8_1 = 0, $9_1 = 0, $10_1 = 0;
  $3_1 = global$0 - 32 | 0;
  global$0 = $3_1;
  $6_1 = $3_1 + 16 | 0;
  HEAP32[($3_1 + 24 | 0) >> 2] = $0_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = 23;
  $8_1 = $49() | 0;
  $9_1 = $125($6_1 | 0) | 0;
  $10_1 = $126($6_1 | 0) | 0;
  HEAP32[($3_1 + 28 | 0) >> 2] = HEAP32[($3_1 + 12 | 0) >> 2] | 0;
  fimport$6($8_1 | 0, $9_1 | 0, $10_1 | 0, $127() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0, HEAP32[($3_1 + 24 | 0) >> 2] | 0 | 0);
  global$0 = $3_1 + 32 | 0;
  return;
 }
 
 function $54($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $5_1 = 0, $8_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $5_1 = $330(4 | 0) | 0;
  HEAP32[$5_1 >> 2] = HEAP32[(HEAP32[($3_1 + 12 | 0) >> 2] | 0) >> 2] | 0;
  HEAP32[($3_1 + 8 | 0) >> 2] = $5_1;
  $8_1 = HEAP32[($3_1 + 8 | 0) >> 2] | 0;
  global$0 = $3_1 + 16 | 0;
  return $8_1 | 0;
 }
 
 function $55() {
  return;
 }
 
 function $56() {
  return 0 | 0;
 }
 
 function $57() {
  return 0 | 0;
 }
 
 function $58() {
  return $130() | 0 | 0;
 }
 
 function $59() {
  return $131() | 0 | 0;
 }
 
 function $60() {
  return $132() | 0 | 0;
 }
 
 function $61($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $6_1 = 0, $8_1 = 0, $9_1 = 0, $10_1 = 0;
  $3_1 = global$0 - 32 | 0;
  global$0 = $3_1;
  $6_1 = $3_1 + 16 | 0;
  HEAP32[($3_1 + 24 | 0) >> 2] = $0_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = 24;
  $8_1 = $58() | 0;
  $9_1 = $134($6_1 | 0) | 0;
  $10_1 = $135($6_1 | 0) | 0;
  HEAP32[($3_1 + 28 | 0) >> 2] = HEAP32[($3_1 + 12 | 0) >> 2] | 0;
  fimport$6($8_1 | 0, $9_1 | 0, $10_1 | 0, $136() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0, HEAP32[($3_1 + 24 | 0) >> 2] | 0 | 0);
  global$0 = $3_1 + 32 | 0;
  return;
 }
 
 function $62($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  var $4_1 = 0, $7_1 = 0, $9_1 = 0, $10_1 = 0, $11_1 = 0;
  $4_1 = global$0 - 32 | 0;
  global$0 = $4_1;
  $7_1 = $4_1 + 16 | 0;
  HEAP32[($4_1 + 24 | 0) >> 2] = $0_1;
  HEAP32[($4_1 + 20 | 0) >> 2] = $1_1;
  HEAP32[($4_1 + 12 | 0) >> 2] = 25;
  $9_1 = HEAP32[($4_1 + 24 | 0) >> 2] | 0;
  $10_1 = $65($7_1 | 0) | 0;
  $11_1 = $66($7_1 | 0) | 0;
  HEAP32[($4_1 + 28 | 0) >> 2] = HEAP32[($4_1 + 12 | 0) >> 2] | 0;
  fimport$4($9_1 | 0, $10_1 | 0, $11_1 | 0, $43() | 0 | 0, HEAP32[($4_1 + 12 | 0) >> 2] | 0 | 0, HEAP32[($4_1 + 20 | 0) >> 2] | 0 | 0);
  global$0 = $4_1 + 32 | 0;
  return;
 }
 
 function $63($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  var $4_1 = 0, $7_1 = 0, $9_1 = 0, $10_1 = 0, $11_1 = 0;
  $4_1 = global$0 - 32 | 0;
  global$0 = $4_1;
  $7_1 = $4_1 + 16 | 0;
  HEAP32[($4_1 + 24 | 0) >> 2] = $0_1;
  HEAP32[($4_1 + 20 | 0) >> 2] = $1_1;
  HEAP32[($4_1 + 12 | 0) >> 2] = 26;
  $9_1 = HEAP32[($4_1 + 24 | 0) >> 2] | 0;
  $10_1 = $68($7_1 | 0) | 0;
  $11_1 = $69($7_1 | 0) | 0;
  HEAP32[($4_1 + 28 | 0) >> 2] = HEAP32[($4_1 + 12 | 0) >> 2] | 0;
  fimport$4($9_1 | 0, $10_1 | 0, $11_1 | 0, $70() | 0 | 0, HEAP32[($4_1 + 12 | 0) >> 2] | 0 | 0, HEAP32[($4_1 + 20 | 0) >> 2] | 0 | 0);
  global$0 = $4_1 + 32 | 0;
  return;
 }
 
 function $64($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  var $4_1 = 0, i64toi32_i32$0 = 0, i64toi32_i32$2 = 0, i64toi32_i32$1 = 0, $11_1 = 0, $15_1 = 0, $8_1 = 0, $56_1 = 0, $86_1 = 0, $33_1 = Math_fround(0);
  $4_1 = global$0 - 48 | 0;
  global$0 = $4_1;
  HEAP32[($4_1 + 44 | 0) >> 2] = $0_1;
  HEAP32[($4_1 + 40 | 0) >> 2] = $1_1;
  $8_1 = HEAP32[($4_1 + 44 | 0) >> 2] | 0;
  i64toi32_i32$2 = $139(HEAP32[($4_1 + 40 | 0) >> 2] | 0 | 0) | 0;
  i64toi32_i32$0 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$1 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $56_1 = i64toi32_i32$0;
  i64toi32_i32$0 = $4_1 + 24 | 0;
  HEAP32[i64toi32_i32$0 >> 2] = $56_1;
  HEAP32[(i64toi32_i32$0 + 4 | 0) >> 2] = i64toi32_i32$1;
  $11_1 = 8;
  HEAP32[(i64toi32_i32$0 + $11_1 | 0) >> 2] = HEAP32[(i64toi32_i32$2 + $11_1 | 0) >> 2] | 0;
  $15_1 = 8;
  HEAP32[(($4_1 + 8 | 0) + $15_1 | 0) >> 2] = HEAP32[(($4_1 + 24 | 0) + $15_1 | 0) >> 2] | 0;
  i64toi32_i32$2 = $4_1;
  i64toi32_i32$1 = HEAP32[($4_1 + 24 | 0) >> 2] | 0;
  i64toi32_i32$0 = HEAP32[($4_1 + 28 | 0) >> 2] | 0;
  $86_1 = i64toi32_i32$1;
  i64toi32_i32$1 = $4_1;
  HEAP32[($4_1 + 8 | 0) >> 2] = $86_1;
  HEAP32[($4_1 + 12 | 0) >> 2] = i64toi32_i32$0;
  HEAPF32[($4_1 + 36 | 0) >> 2] = Math_fround(FUNCTION_TABLE[$8_1 | 0]($4_1 + 8 | 0));
  $33_1 = Math_fround($110($4_1 + 36 | 0 | 0));
  global$0 = $4_1 + 48 | 0;
  return Math_fround($33_1);
 }
 
 function $65($0_1) {
  $0_1 = $0_1 | 0;
  HEAP32[((global$0 - 16 | 0) + 12 | 0) >> 2] = $0_1;
  return 2 | 0;
 }
 
 function $66($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $4_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $4_1 = $140() | 0;
  global$0 = $3_1 + 16 | 0;
  return $4_1 | 0;
 }
 
 function $67($0_1, $1_1, $2_1, $3_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  var $6_1 = 0, i64toi32_i32$0 = 0, i64toi32_i32$1 = 0, i64toi32_i32$2 = 0, $35_1 = 0, $19_1 = 0, $25_1 = 0, $31_1 = 0, $16_1 = 0, $107_1 = 0, $125_1 = 0, $143_1 = 0, $173_1 = 0, $193_1 = 0, $210_1 = 0, $64_1 = 0;
  $6_1 = global$0 - 176 | 0;
  global$0 = $6_1;
  HEAP32[($6_1 + 172 | 0) >> 2] = $0_1;
  HEAP32[($6_1 + 168 | 0) >> 2] = $1_1;
  HEAP32[($6_1 + 164 | 0) >> 2] = $2_1;
  HEAP32[($6_1 + 160 | 0) >> 2] = $3_1;
  $16_1 = HEAP32[($6_1 + 172 | 0) >> 2] | 0;
  i64toi32_i32$2 = $139(HEAP32[($6_1 + 168 | 0) >> 2] | 0 | 0) | 0;
  i64toi32_i32$0 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$1 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $107_1 = i64toi32_i32$0;
  i64toi32_i32$0 = $6_1 + 80 | 0;
  HEAP32[i64toi32_i32$0 >> 2] = $107_1;
  HEAP32[(i64toi32_i32$0 + 4 | 0) >> 2] = i64toi32_i32$1;
  $19_1 = 8;
  HEAP32[(i64toi32_i32$0 + $19_1 | 0) >> 2] = HEAP32[(i64toi32_i32$2 + $19_1 | 0) >> 2] | 0;
  i64toi32_i32$2 = $139(HEAP32[($6_1 + 164 | 0) >> 2] | 0 | 0) | 0;
  i64toi32_i32$1 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$0 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $125_1 = i64toi32_i32$1;
  i64toi32_i32$1 = $6_1 + 64 | 0;
  HEAP32[i64toi32_i32$1 >> 2] = $125_1;
  HEAP32[(i64toi32_i32$1 + 4 | 0) >> 2] = i64toi32_i32$0;
  $25_1 = 8;
  HEAP32[(i64toi32_i32$1 + $25_1 | 0) >> 2] = HEAP32[(i64toi32_i32$2 + $25_1 | 0) >> 2] | 0;
  i64toi32_i32$2 = $139(HEAP32[($6_1 + 160 | 0) >> 2] | 0 | 0) | 0;
  i64toi32_i32$0 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$1 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $143_1 = i64toi32_i32$0;
  i64toi32_i32$0 = $6_1 + 48 | 0;
  HEAP32[i64toi32_i32$0 >> 2] = $143_1;
  HEAP32[(i64toi32_i32$0 + 4 | 0) >> 2] = i64toi32_i32$1;
  $31_1 = 8;
  HEAP32[(i64toi32_i32$0 + $31_1 | 0) >> 2] = HEAP32[(i64toi32_i32$2 + $31_1 | 0) >> 2] | 0;
  $35_1 = 8;
  HEAP32[(($6_1 + 32 | 0) + $35_1 | 0) >> 2] = HEAP32[(($6_1 + 80 | 0) + $35_1 | 0) >> 2] | 0;
  i64toi32_i32$2 = $6_1;
  i64toi32_i32$1 = HEAP32[($6_1 + 80 | 0) >> 2] | 0;
  i64toi32_i32$0 = HEAP32[($6_1 + 84 | 0) >> 2] | 0;
  $173_1 = i64toi32_i32$1;
  i64toi32_i32$1 = $6_1;
  HEAP32[($6_1 + 32 | 0) >> 2] = $173_1;
  HEAP32[($6_1 + 36 | 0) >> 2] = i64toi32_i32$0;
  HEAP32[(($6_1 + 16 | 0) + $35_1 | 0) >> 2] = HEAP32[(($6_1 + 64 | 0) + $35_1 | 0) >> 2] | 0;
  i64toi32_i32$2 = $6_1;
  i64toi32_i32$0 = HEAP32[($6_1 + 64 | 0) >> 2] | 0;
  i64toi32_i32$1 = HEAP32[($6_1 + 68 | 0) >> 2] | 0;
  $193_1 = i64toi32_i32$0;
  i64toi32_i32$0 = $6_1;
  HEAP32[($6_1 + 16 | 0) >> 2] = $193_1;
  HEAP32[($6_1 + 20 | 0) >> 2] = i64toi32_i32$1;
  HEAP32[($6_1 + $35_1 | 0) >> 2] = HEAP32[(($6_1 + 48 | 0) + $35_1 | 0) >> 2] | 0;
  i64toi32_i32$2 = $6_1;
  i64toi32_i32$1 = HEAP32[($6_1 + 48 | 0) >> 2] | 0;
  i64toi32_i32$0 = HEAP32[($6_1 + 52 | 0) >> 2] | 0;
  $210_1 = i64toi32_i32$1;
  i64toi32_i32$1 = $6_1;
  HEAP32[$6_1 >> 2] = $210_1;
  HEAP32[($6_1 + 4 | 0) >> 2] = i64toi32_i32$0;
  FUNCTION_TABLE[$16_1 | 0]($6_1 + 96 | 0, $6_1 + 32 | 0, $6_1 + 16 | 0, $6_1);
  $64_1 = $141($6_1 + 96 | 0 | 0) | 0;
  global$0 = $6_1 + 176 | 0;
  return $64_1 | 0;
 }
 
 function $68($0_1) {
  $0_1 = $0_1 | 0;
  HEAP32[((global$0 - 16 | 0) + 12 | 0) >> 2] = $0_1;
  return 4 | 0;
 }
 
 function $69($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $4_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $4_1 = $142() | 0;
  global$0 = $3_1 + 16 | 0;
  return $4_1 | 0;
 }
 
 function $70() {
  return 1840 | 0;
 }
 
 function $71() {
  return 1096 | 0;
 }
 
 function $72($0_1, $1_1, $2_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  var $5_1 = 0;
  $5_1 = global$0 - 32 | 0;
  global$0 = $5_1;
  HEAP32[($5_1 + 28 | 0) >> 2] = $1_1;
  HEAP32[($5_1 + 24 | 0) >> 2] = $2_1;
  HEAPF32[($5_1 + 20 | 0) >> 2] = Math_fround(Math_fround(HEAPF32[(HEAP32[($5_1 + 28 | 0) >> 2] | 0) >> 2]) - Math_fround(HEAPF32[(HEAP32[($5_1 + 24 | 0) >> 2] | 0) >> 2]));
  HEAPF32[($5_1 + 16 | 0) >> 2] = Math_fround(Math_fround(HEAPF32[((HEAP32[($5_1 + 28 | 0) >> 2] | 0) + 4 | 0) >> 2]) - Math_fround(HEAPF32[((HEAP32[($5_1 + 24 | 0) >> 2] | 0) + 4 | 0) >> 2]));
  HEAPF32[($5_1 + 12 | 0) >> 2] = Math_fround(Math_fround(HEAPF32[((HEAP32[($5_1 + 28 | 0) >> 2] | 0) + 8 | 0) >> 2]) - Math_fround(HEAPF32[((HEAP32[($5_1 + 24 | 0) >> 2] | 0) + 8 | 0) >> 2]));
  $80($0_1 | 0, $5_1 + 20 | 0 | 0, $5_1 + 16 | 0 | 0, $5_1 + 12 | 0 | 0) | 0;
  global$0 = $5_1 + 32 | 0;
  return;
 }
 
 function $73($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  var $4_1 = 0, $8_1 = 0;
  $4_1 = global$0 - 16 | 0;
  global$0 = $4_1;
  HEAP32[($4_1 + 12 | 0) >> 2] = $1_1;
  $8_1 = HEAP32[($4_1 + 12 | 0) >> 2] | 0;
  HEAPF32[($4_1 + 8 | 0) >> 2] = Math_fround($78(Math_fround(Math_fround($11(HEAP32[($4_1 + 12 | 0) >> 2] | 0 | 0, HEAP32[($4_1 + 12 | 0) >> 2] | 0 | 0)))));
  $79($0_1 | 0, $8_1 | 0, $4_1 + 8 | 0 | 0);
  global$0 = $4_1 + 16 | 0;
  return;
 }
 
 function $74($0_1, $1_1, $2_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  var $5_1 = 0;
  $5_1 = global$0 - 32 | 0;
  global$0 = $5_1;
  HEAP32[($5_1 + 28 | 0) >> 2] = $1_1;
  HEAP32[($5_1 + 24 | 0) >> 2] = $2_1;
  HEAPF32[($5_1 + 20 | 0) >> 2] = Math_fround(Math_fround(Math_fround(HEAPF32[((HEAP32[($5_1 + 28 | 0) >> 2] | 0) + 4 | 0) >> 2]) * Math_fround(HEAPF32[((HEAP32[($5_1 + 24 | 0) >> 2] | 0) + 8 | 0) >> 2])) - Math_fround(Math_fround(HEAPF32[((HEAP32[($5_1 + 24 | 0) >> 2] | 0) + 4 | 0) >> 2]) * Math_fround(HEAPF32[((HEAP32[($5_1 + 28 | 0) >> 2] | 0) + 8 | 0) >> 2])));
  HEAPF32[($5_1 + 16 | 0) >> 2] = Math_fround(Math_fround(Math_fround(HEAPF32[((HEAP32[($5_1 + 28 | 0) >> 2] | 0) + 8 | 0) >> 2]) * Math_fround(HEAPF32[(HEAP32[($5_1 + 24 | 0) >> 2] | 0) >> 2])) - Math_fround(Math_fround(HEAPF32[((HEAP32[($5_1 + 24 | 0) >> 2] | 0) + 8 | 0) >> 2]) * Math_fround(HEAPF32[(HEAP32[($5_1 + 28 | 0) >> 2] | 0) >> 2])));
  HEAPF32[($5_1 + 12 | 0) >> 2] = Math_fround(Math_fround(Math_fround(HEAPF32[(HEAP32[($5_1 + 28 | 0) >> 2] | 0) >> 2]) * Math_fround(HEAPF32[((HEAP32[($5_1 + 24 | 0) >> 2] | 0) + 4 | 0) >> 2])) - Math_fround(Math_fround(HEAPF32[(HEAP32[($5_1 + 24 | 0) >> 2] | 0) >> 2]) * Math_fround(HEAPF32[((HEAP32[($5_1 + 28 | 0) >> 2] | 0) + 4 | 0) >> 2])));
  $80($0_1 | 0, $5_1 + 20 | 0 | 0, $5_1 + 16 | 0 | 0, $5_1 + 12 | 0 | 0) | 0;
  global$0 = $5_1 + 32 | 0;
  return;
 }
 
 function $75($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  var i64toi32_i32$2 = 0, i64toi32_i32$0 = 0, i64toi32_i32$1 = 0, $4_1 = 0, $18_1 = 0, $5_1 = 0, $8_1 = 0, $9_1 = 0, $11_1 = 0, $21_1 = 0, $24_1 = 0, $27_1 = 0, $29_1 = 0, $34_1 = 0, $35_1 = 0, $40_1 = 0, $41_1 = 0, $46_1 = 0, $47_1 = 0, $7_1 = 0, $125_1 = 0, $135_1 = 0, $152_1 = 0, $162_1 = 0, $179_1 = 0, $189_1 = 0, $206_1 = 0, $216_1 = 0, $50_1 = 0;
  $4_1 = global$0 - 80 | 0;
  global$0 = $4_1;
  HEAP32[($4_1 + 72 | 0) >> 2] = $0_1;
  HEAP32[($4_1 + 68 | 0) >> 2] = $1_1;
  $5_1 = HEAP32[($4_1 + 72 | 0) >> 2] | 0;
  HEAP32[($4_1 + 76 | 0) >> 2] = $5_1;
  $7_1 = $5_1 + 64 | 0;
  $8_1 = $5_1;
  label$1 : while (1) {
   $9_1 = $8_1;
   $81($9_1 | 0) | 0;
   $11_1 = $9_1 + 16 | 0;
   $8_1 = $11_1;
   if (!(($11_1 | 0) == ($7_1 | 0) & 1 | 0)) {
    continue label$1
   }
   break label$1;
  };
  $18_1 = 0;
  $21_1 = $4_1 + 16 | 0;
  $24_1 = $4_1 + 32 | 0;
  $27_1 = $4_1 + 48 | 0;
  $82($27_1 | 0, Math_fround(Math_fround(HEAPF32[(HEAP32[($4_1 + 68 | 0) >> 2] | 0) >> 2])), $18_1 | 0, $18_1 | 0, $18_1 | 0) | 0;
  i64toi32_i32$2 = $27_1;
  i64toi32_i32$0 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$1 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $125_1 = i64toi32_i32$0;
  i64toi32_i32$0 = $5_1;
  HEAP32[i64toi32_i32$0 >> 2] = $125_1;
  HEAP32[(i64toi32_i32$0 + 4 | 0) >> 2] = i64toi32_i32$1;
  $29_1 = 8;
  i64toi32_i32$2 = i64toi32_i32$2 + $29_1 | 0;
  i64toi32_i32$1 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$0 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $135_1 = i64toi32_i32$1;
  i64toi32_i32$1 = $5_1 + $29_1 | 0;
  HEAP32[i64toi32_i32$1 >> 2] = $135_1;
  HEAP32[(i64toi32_i32$1 + 4 | 0) >> 2] = i64toi32_i32$0;
  $83($24_1 | 0, $18_1 | 0, Math_fround(Math_fround(HEAPF32[(HEAP32[($4_1 + 68 | 0) >> 2] | 0) >> 2])), $18_1 | 0, $18_1 | 0) | 0;
  $34_1 = $5_1 + 16 | 0;
  i64toi32_i32$2 = $24_1;
  i64toi32_i32$0 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$1 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $152_1 = i64toi32_i32$0;
  i64toi32_i32$0 = $34_1;
  HEAP32[i64toi32_i32$0 >> 2] = $152_1;
  HEAP32[(i64toi32_i32$0 + 4 | 0) >> 2] = i64toi32_i32$1;
  $35_1 = 8;
  i64toi32_i32$2 = i64toi32_i32$2 + $35_1 | 0;
  i64toi32_i32$1 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$0 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $162_1 = i64toi32_i32$1;
  i64toi32_i32$1 = $34_1 + $35_1 | 0;
  HEAP32[i64toi32_i32$1 >> 2] = $162_1;
  HEAP32[(i64toi32_i32$1 + 4 | 0) >> 2] = i64toi32_i32$0;
  $84($21_1 | 0, $18_1 | 0, $18_1 | 0, Math_fround(Math_fround(HEAPF32[(HEAP32[($4_1 + 68 | 0) >> 2] | 0) >> 2])), $18_1 | 0) | 0;
  $40_1 = $5_1 + 32 | 0;
  i64toi32_i32$2 = $21_1;
  i64toi32_i32$0 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$1 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $179_1 = i64toi32_i32$0;
  i64toi32_i32$0 = $40_1;
  HEAP32[i64toi32_i32$0 >> 2] = $179_1;
  HEAP32[(i64toi32_i32$0 + 4 | 0) >> 2] = i64toi32_i32$1;
  $41_1 = 8;
  i64toi32_i32$2 = i64toi32_i32$2 + $41_1 | 0;
  i64toi32_i32$1 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$0 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $189_1 = i64toi32_i32$1;
  i64toi32_i32$1 = $40_1 + $41_1 | 0;
  HEAP32[i64toi32_i32$1 >> 2] = $189_1;
  HEAP32[(i64toi32_i32$1 + 4 | 0) >> 2] = i64toi32_i32$0;
  $85($4_1 | 0, $18_1 | 0, $18_1 | 0, $18_1 | 0, Math_fround(Math_fround(HEAPF32[(HEAP32[($4_1 + 68 | 0) >> 2] | 0) >> 2]))) | 0;
  $46_1 = $5_1 + 48 | 0;
  i64toi32_i32$2 = $4_1;
  i64toi32_i32$0 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$1 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $206_1 = i64toi32_i32$0;
  i64toi32_i32$0 = $46_1;
  HEAP32[i64toi32_i32$0 >> 2] = $206_1;
  HEAP32[(i64toi32_i32$0 + 4 | 0) >> 2] = i64toi32_i32$1;
  $47_1 = 8;
  i64toi32_i32$2 = i64toi32_i32$2 + $47_1 | 0;
  i64toi32_i32$1 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$0 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $216_1 = i64toi32_i32$1;
  i64toi32_i32$1 = $46_1 + $47_1 | 0;
  HEAP32[i64toi32_i32$1 >> 2] = $216_1;
  HEAP32[(i64toi32_i32$1 + 4 | 0) >> 2] = i64toi32_i32$0;
  $50_1 = HEAP32[($4_1 + 76 | 0) >> 2] | 0;
  global$0 = $4_1 + 80 | 0;
  return $50_1 | 0;
 }
 
 function $76($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  var $4_1 = 0, $5_1 = 0, $20_1 = 0;
  $4_1 = global$0 - 16 | 0;
  global$0 = $4_1;
  HEAP32[($4_1 + 12 | 0) >> 2] = $0_1;
  HEAP32[($4_1 + 8 | 0) >> 2] = $1_1;
  $5_1 = HEAP32[($4_1 + 12 | 0) >> 2] | 0;
  label$1 : {
   if ((HEAP32[($4_1 + 8 | 0) >> 2] | 0 | 0) < ($86($5_1 | 0) | 0 | 0) & 1 | 0) {
    break label$1
   }
   fimport$5(1104 | 0, 1123 | 0, 356 | 0, 1154 | 0);
   abort();
  }
  $20_1 = $5_1 + ((HEAP32[($4_1 + 8 | 0) >> 2] | 0) << 4 | 0) | 0;
  global$0 = $4_1 + 16 | 0;
  return $20_1 | 0;
 }
 
 function $77($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  var $4_1 = 0, $6_1 = 0, $27_1 = 0;
  $4_1 = global$0 - 16 | 0;
  global$0 = $4_1;
  HEAP32[($4_1 + 12 | 0) >> 2] = $0_1;
  HEAP32[($4_1 + 8 | 0) >> 2] = $1_1;
  $6_1 = HEAP32[($4_1 + 12 | 0) >> 2] | 0;
  label$1 : {
   label$2 : {
    if (!((HEAP32[($4_1 + 8 | 0) >> 2] | 0 | 0) >= (0 | 0) & 1 | 0)) {
     break label$2
    }
    if ((HEAP32[($4_1 + 8 | 0) >> 2] | 0 | 0) < ($87($6_1 | 0) | 0 | 0) & 1 | 0) {
     break label$1
    }
   }
   fimport$5(1165 | 0, 1250 | 0, 237 | 0, 1154 | 0);
   abort();
  }
  $27_1 = $6_1 + ((HEAP32[($4_1 + 8 | 0) >> 2] | 0) << 2 | 0) | 0;
  global$0 = $4_1 + 16 | 0;
  return $27_1 | 0;
 }
 
 function $78($0_1) {
  $0_1 = Math_fround($0_1);
  var $3_1 = 0, $9_1 = Math_fround(0);
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAPF32[($3_1 + 12 | 0) >> 2] = $0_1;
  $9_1 = Math_fround(Math_fround(1.0) / Math_fround($12(Math_fround(Math_fround(HEAPF32[($3_1 + 12 | 0) >> 2])))));
  global$0 = $3_1 + 16 | 0;
  return Math_fround($9_1);
 }
 
 function $79($0_1, $1_1, $2_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  var $5_1 = 0;
  $5_1 = global$0 - 32 | 0;
  global$0 = $5_1;
  HEAP32[($5_1 + 28 | 0) >> 2] = $1_1;
  HEAP32[($5_1 + 24 | 0) >> 2] = $2_1;
  HEAPF32[($5_1 + 20 | 0) >> 2] = Math_fround(Math_fround(HEAPF32[(HEAP32[($5_1 + 28 | 0) >> 2] | 0) >> 2]) * Math_fround(HEAPF32[(HEAP32[($5_1 + 24 | 0) >> 2] | 0) >> 2]));
  HEAPF32[($5_1 + 16 | 0) >> 2] = Math_fround(Math_fround(HEAPF32[((HEAP32[($5_1 + 28 | 0) >> 2] | 0) + 4 | 0) >> 2]) * Math_fround(HEAPF32[(HEAP32[($5_1 + 24 | 0) >> 2] | 0) >> 2]));
  HEAPF32[($5_1 + 12 | 0) >> 2] = Math_fround(Math_fround(HEAPF32[((HEAP32[($5_1 + 28 | 0) >> 2] | 0) + 8 | 0) >> 2]) * Math_fround(HEAPF32[(HEAP32[($5_1 + 24 | 0) >> 2] | 0) >> 2]));
  $80($0_1 | 0, $5_1 + 20 | 0 | 0, $5_1 + 16 | 0 | 0, $5_1 + 12 | 0 | 0) | 0;
  global$0 = $5_1 + 32 | 0;
  return;
 }
 
 function $80($0_1, $1_1, $2_1, $3_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  var $6_1 = 0, $7_1 = 0;
  $6_1 = global$0 - 16 | 0;
  HEAP32[($6_1 + 12 | 0) >> 2] = $0_1;
  HEAP32[($6_1 + 8 | 0) >> 2] = $1_1;
  HEAP32[($6_1 + 4 | 0) >> 2] = $2_1;
  HEAP32[$6_1 >> 2] = $3_1;
  $7_1 = HEAP32[($6_1 + 12 | 0) >> 2] | 0;
  HEAPF32[$7_1 >> 2] = Math_fround(HEAPF32[(HEAP32[($6_1 + 8 | 0) >> 2] | 0) >> 2]);
  HEAPF32[($7_1 + 4 | 0) >> 2] = Math_fround(HEAPF32[(HEAP32[($6_1 + 4 | 0) >> 2] | 0) >> 2]);
  HEAPF32[($7_1 + 8 | 0) >> 2] = Math_fround(HEAPF32[(HEAP32[$6_1 >> 2] | 0) >> 2]);
  return $7_1 | 0;
 }
 
 function $81($0_1) {
  $0_1 = $0_1 | 0;
  var $5_1 = 0, $6_1 = Math_fround(0), $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  $6_1 = Math_fround(0 | 0);
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $5_1 = HEAP32[($3_1 + 12 | 0) >> 2] | 0;
  HEAPF32[$5_1 >> 2] = $6_1;
  HEAPF32[($5_1 + 4 | 0) >> 2] = $6_1;
  HEAPF32[($5_1 + 8 | 0) >> 2] = $6_1;
  HEAPF32[($5_1 + 12 | 0) >> 2] = $6_1;
  return $5_1 | 0;
 }
 
 function $82($0_1, $1_1, $2_1, $3_1, $4_1) {
  $0_1 = $0_1 | 0;
  $1_1 = Math_fround($1_1);
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  $4_1 = $4_1 | 0;
  var $7_1 = 0, $8_1 = 0;
  $7_1 = global$0 - 32 | 0;
  HEAP32[($7_1 + 28 | 0) >> 2] = $0_1;
  HEAPF32[($7_1 + 24 | 0) >> 2] = $1_1;
  HEAP32[($7_1 + 20 | 0) >> 2] = $2_1;
  HEAP32[($7_1 + 16 | 0) >> 2] = $3_1;
  HEAP32[($7_1 + 12 | 0) >> 2] = $4_1;
  $8_1 = HEAP32[($7_1 + 28 | 0) >> 2] | 0;
  HEAPF32[$8_1 >> 2] = Math_fround(HEAPF32[($7_1 + 24 | 0) >> 2]);
  HEAPF32[($8_1 + 4 | 0) >> 2] = Math_fround(HEAP32[($7_1 + 20 | 0) >> 2] | 0 | 0);
  HEAPF32[($8_1 + 8 | 0) >> 2] = Math_fround(HEAP32[($7_1 + 16 | 0) >> 2] | 0 | 0);
  HEAPF32[($8_1 + 12 | 0) >> 2] = Math_fround(HEAP32[($7_1 + 12 | 0) >> 2] | 0 | 0);
  return $8_1 | 0;
 }
 
 function $83($0_1, $1_1, $2_1, $3_1, $4_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = Math_fround($2_1);
  $3_1 = $3_1 | 0;
  $4_1 = $4_1 | 0;
  var $7_1 = 0, $8_1 = 0;
  $7_1 = global$0 - 32 | 0;
  HEAP32[($7_1 + 28 | 0) >> 2] = $0_1;
  HEAP32[($7_1 + 24 | 0) >> 2] = $1_1;
  HEAPF32[($7_1 + 20 | 0) >> 2] = $2_1;
  HEAP32[($7_1 + 16 | 0) >> 2] = $3_1;
  HEAP32[($7_1 + 12 | 0) >> 2] = $4_1;
  $8_1 = HEAP32[($7_1 + 28 | 0) >> 2] | 0;
  HEAPF32[$8_1 >> 2] = Math_fround(HEAP32[($7_1 + 24 | 0) >> 2] | 0 | 0);
  HEAPF32[($8_1 + 4 | 0) >> 2] = Math_fround(HEAPF32[($7_1 + 20 | 0) >> 2]);
  HEAPF32[($8_1 + 8 | 0) >> 2] = Math_fround(HEAP32[($7_1 + 16 | 0) >> 2] | 0 | 0);
  HEAPF32[($8_1 + 12 | 0) >> 2] = Math_fround(HEAP32[($7_1 + 12 | 0) >> 2] | 0 | 0);
  return $8_1 | 0;
 }
 
 function $84($0_1, $1_1, $2_1, $3_1, $4_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = Math_fround($3_1);
  $4_1 = $4_1 | 0;
  var $7_1 = 0, $8_1 = 0;
  $7_1 = global$0 - 32 | 0;
  HEAP32[($7_1 + 28 | 0) >> 2] = $0_1;
  HEAP32[($7_1 + 24 | 0) >> 2] = $1_1;
  HEAP32[($7_1 + 20 | 0) >> 2] = $2_1;
  HEAPF32[($7_1 + 16 | 0) >> 2] = $3_1;
  HEAP32[($7_1 + 12 | 0) >> 2] = $4_1;
  $8_1 = HEAP32[($7_1 + 28 | 0) >> 2] | 0;
  HEAPF32[$8_1 >> 2] = Math_fround(HEAP32[($7_1 + 24 | 0) >> 2] | 0 | 0);
  HEAPF32[($8_1 + 4 | 0) >> 2] = Math_fround(HEAP32[($7_1 + 20 | 0) >> 2] | 0 | 0);
  HEAPF32[($8_1 + 8 | 0) >> 2] = Math_fround(HEAPF32[($7_1 + 16 | 0) >> 2]);
  HEAPF32[($8_1 + 12 | 0) >> 2] = Math_fround(HEAP32[($7_1 + 12 | 0) >> 2] | 0 | 0);
  return $8_1 | 0;
 }
 
 function $85($0_1, $1_1, $2_1, $3_1, $4_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  $4_1 = Math_fround($4_1);
  var $7_1 = 0, $8_1 = 0;
  $7_1 = global$0 - 32 | 0;
  HEAP32[($7_1 + 28 | 0) >> 2] = $0_1;
  HEAP32[($7_1 + 24 | 0) >> 2] = $1_1;
  HEAP32[($7_1 + 20 | 0) >> 2] = $2_1;
  HEAP32[($7_1 + 16 | 0) >> 2] = $3_1;
  HEAPF32[($7_1 + 12 | 0) >> 2] = $4_1;
  $8_1 = HEAP32[($7_1 + 28 | 0) >> 2] | 0;
  HEAPF32[$8_1 >> 2] = Math_fround(HEAP32[($7_1 + 24 | 0) >> 2] | 0 | 0);
  HEAPF32[($8_1 + 4 | 0) >> 2] = Math_fround(HEAP32[($7_1 + 20 | 0) >> 2] | 0 | 0);
  HEAPF32[($8_1 + 8 | 0) >> 2] = Math_fround(HEAP32[($7_1 + 16 | 0) >> 2] | 0 | 0);
  HEAPF32[($8_1 + 12 | 0) >> 2] = Math_fround(HEAPF32[($7_1 + 12 | 0) >> 2]);
  return $8_1 | 0;
 }
 
 function $86($0_1) {
  $0_1 = $0_1 | 0;
  HEAP32[((global$0 - 16 | 0) + 12 | 0) >> 2] = $0_1;
  return 4 | 0;
 }
 
 function $87($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $5_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $5_1 = $89(HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0) | 0;
  global$0 = $3_1 + 16 | 0;
  return $5_1 | 0;
 }
 
 function $88($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  var $4_1 = 0, $16_1 = Math_fround(0);
  $4_1 = global$0 - 32 | 0;
  global$0 = $4_1;
  HEAP32[($4_1 + 28 | 0) >> 2] = $0_1;
  HEAP32[($4_1 + 24 | 0) >> 2] = $1_1;
  $90($4_1 + 8 | 0 | 0, HEAP32[($4_1 + 28 | 0) >> 2] | 0 | 0, HEAP32[($4_1 + 24 | 0) >> 2] | 0 | 0);
  $16_1 = Math_fround(Math_fround(Math_fround(HEAPF32[($4_1 + 8 | 0) >> 2]) + Math_fround(HEAPF32[($4_1 + 12 | 0) >> 2])) + Math_fround(HEAPF32[($4_1 + 16 | 0) >> 2]));
  global$0 = $4_1 + 32 | 0;
  return Math_fround($16_1);
 }
 
 function $89($0_1) {
  $0_1 = $0_1 | 0;
  HEAP32[((global$0 - 16 | 0) + 12 | 0) >> 2] = $0_1;
  return 4 | 0;
 }
 
 function $90($0_1, $1_1, $2_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  var $5_1 = 0;
  $5_1 = global$0 - 32 | 0;
  global$0 = $5_1;
  HEAP32[($5_1 + 28 | 0) >> 2] = $1_1;
  HEAP32[($5_1 + 24 | 0) >> 2] = $2_1;
  HEAPF32[($5_1 + 20 | 0) >> 2] = Math_fround(Math_fround(HEAPF32[(HEAP32[($5_1 + 28 | 0) >> 2] | 0) >> 2]) * Math_fround(HEAPF32[(HEAP32[($5_1 + 24 | 0) >> 2] | 0) >> 2]));
  HEAPF32[($5_1 + 16 | 0) >> 2] = Math_fround(Math_fround(HEAPF32[((HEAP32[($5_1 + 28 | 0) >> 2] | 0) + 4 | 0) >> 2]) * Math_fround(HEAPF32[((HEAP32[($5_1 + 24 | 0) >> 2] | 0) + 4 | 0) >> 2]));
  HEAPF32[($5_1 + 12 | 0) >> 2] = Math_fround(Math_fround(HEAPF32[((HEAP32[($5_1 + 28 | 0) >> 2] | 0) + 8 | 0) >> 2]) * Math_fround(HEAPF32[((HEAP32[($5_1 + 24 | 0) >> 2] | 0) + 8 | 0) >> 2]));
  $80($0_1 | 0, $5_1 + 20 | 0 | 0, $5_1 + 16 | 0 | 0, $5_1 + 12 | 0 | 0) | 0;
  global$0 = $5_1 + 32 | 0;
  return;
 }
 
 function $91($0_1) {
  $0_1 = $0_1 | 0;
  HEAP32[((global$0 - 16 | 0) + 12 | 0) >> 2] = $0_1;
  return 1312 | 0;
 }
 
 function $92() {
  return 1312 | 0;
 }
 
 function $93() {
  return 1356 | 0;
 }
 
 function $94() {
  return 1408 | 0;
 }
 
 function $95($0_1) {
  $0_1 = $0_1 | 0;
  var $5_1 = 0, $3_1 = 0, $6_1 = Math_fround(0);
  $3_1 = global$0 - 16 | 0;
  $6_1 = Math_fround(0 | 0);
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $5_1 = HEAP32[($3_1 + 12 | 0) >> 2] | 0;
  HEAPF32[$5_1 >> 2] = $6_1;
  HEAPF32[($5_1 + 4 | 0) >> 2] = $6_1;
  return $5_1 | 0;
 }
 
 function $96($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $6_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $6_1 = $99(FUNCTION_TABLE[HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0]() | 0 | 0) | 0;
  global$0 = $3_1 + 16 | 0;
  return $6_1 | 0;
 }
 
 function $97($0_1) {
  $0_1 = $0_1 | 0;
  HEAP32[((global$0 - 16 | 0) + 12 | 0) >> 2] = $0_1;
  return 1 | 0;
 }
 
 function $98($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $4_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $4_1 = $100() | 0;
  global$0 = $3_1 + 16 | 0;
  return $4_1 | 0;
 }
 
 function $99($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  return HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0;
 }
 
 function $100() {
  return 1432 | 0;
 }
 
 function $101($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  return HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0;
 }
 
 function $102($0_1, $1_1, $2_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  var $5_1 = 0, $6_1 = 0;
  $5_1 = global$0 - 16 | 0;
  HEAP32[($5_1 + 12 | 0) >> 2] = $0_1;
  HEAP32[($5_1 + 8 | 0) >> 2] = $1_1;
  HEAP32[($5_1 + 4 | 0) >> 2] = $2_1;
  $6_1 = HEAP32[($5_1 + 12 | 0) >> 2] | 0;
  HEAPF32[$6_1 >> 2] = Math_fround(+HEAPF64[(HEAP32[($5_1 + 8 | 0) >> 2] | 0) >> 3]);
  HEAPF32[($6_1 + 4 | 0) >> 2] = Math_fround(+HEAPF64[(HEAP32[($5_1 + 4 | 0) >> 2] | 0) >> 3]);
  return $6_1 | 0;
 }
 
 function $103($0_1, $1_1, $2_1) {
  $0_1 = $0_1 | 0;
  $1_1 = +$1_1;
  $2_1 = +$2_1;
  var $5_1 = 0, $12_1 = 0, $14_1 = 0;
  $5_1 = global$0 - 48 | 0;
  global$0 = $5_1;
  HEAP32[($5_1 + 44 | 0) >> 2] = $0_1;
  HEAPF64[($5_1 + 32 | 0) >> 3] = $1_1;
  HEAPF64[($5_1 + 24 | 0) >> 3] = $2_1;
  $12_1 = HEAP32[($5_1 + 44 | 0) >> 2] | 0;
  HEAPF64[($5_1 + 16 | 0) >> 3] = +$107(+(+HEAPF64[($5_1 + 32 | 0) >> 3]));
  HEAPF64[($5_1 + 8 | 0) >> 3] = +$107(+(+HEAPF64[($5_1 + 24 | 0) >> 3]));
  $14_1 = $99(FUNCTION_TABLE[$12_1 | 0]($5_1 + 16 | 0, $5_1 + 8 | 0) | 0 | 0) | 0;
  global$0 = $5_1 + 48 | 0;
  return $14_1 | 0;
 }
 
 function $104($0_1) {
  $0_1 = $0_1 | 0;
  HEAP32[((global$0 - 16 | 0) + 12 | 0) >> 2] = $0_1;
  return 3 | 0;
 }
 
 function $105($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $4_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $4_1 = $108() | 0;
  global$0 = $3_1 + 16 | 0;
  return $4_1 | 0;
 }
 
 function $106() {
  return 1448 | 0;
 }
 
 function $107($0_1) {
  $0_1 = +$0_1;
  var $3_1 = 0, $7_1 = 0.0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAPF64[($3_1 + 8 | 0) >> 3] = $0_1;
  $7_1 = +$109(+(+HEAPF64[($3_1 + 8 | 0) >> 3]));
  global$0 = $3_1 + 16 | 0;
  return +$7_1;
 }
 
 function $108() {
  return 1436 | 0;
 }
 
 function $109($0_1) {
  $0_1 = +$0_1;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  HEAPF64[($3_1 + 8 | 0) >> 3] = $0_1;
  return +(+HEAPF64[($3_1 + 8 | 0) >> 3]);
 }
 
 function $110($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  return Math_fround(Math_fround(HEAPF32[(HEAP32[($3_1 + 12 | 0) >> 2] | 0) >> 2]));
 }
 
 function $111($0_1) {
  $0_1 = Math_fround($0_1);
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  HEAPF32[($3_1 + 12 | 0) >> 2] = $0_1;
  return Math_fround(Math_fround(HEAPF32[($3_1 + 12 | 0) >> 2]));
 }
 
 function $112() {
  return 4364 | 0;
 }
 
 function $113($0_1) {
  $0_1 = $0_1 | 0;
  HEAP32[((global$0 - 16 | 0) + 12 | 0) >> 2] = $0_1;
  return 1496 | 0;
 }
 
 function $114() {
  return 1496 | 0;
 }
 
 function $115() {
  return 1540 | 0;
 }
 
 function $116() {
  return 1592 | 0;
 }
 
 function $117($0_1) {
  $0_1 = $0_1 | 0;
  var $5_1 = 0, $6_1 = Math_fround(0), $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  $6_1 = Math_fround(0 | 0);
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $5_1 = HEAP32[($3_1 + 12 | 0) >> 2] | 0;
  HEAPF32[$5_1 >> 2] = $6_1;
  HEAPF32[($5_1 + 4 | 0) >> 2] = $6_1;
  HEAPF32[($5_1 + 8 | 0) >> 2] = $6_1;
  return $5_1 | 0;
 }
 
 function $118($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $6_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $6_1 = $121(FUNCTION_TABLE[HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0]() | 0 | 0) | 0;
  global$0 = $3_1 + 16 | 0;
  return $6_1 | 0;
 }
 
 function $119($0_1) {
  $0_1 = $0_1 | 0;
  HEAP32[((global$0 - 16 | 0) + 12 | 0) >> 2] = $0_1;
  return 1 | 0;
 }
 
 function $120($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $4_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $4_1 = $122() | 0;
  global$0 = $3_1 + 16 | 0;
  return $4_1 | 0;
 }
 
 function $121($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  return HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0;
 }
 
 function $122() {
  return 1608 | 0;
 }
 
 function $123($0_1, $1_1, $2_1, $3_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  var $6_1 = 0, $7_1 = 0;
  $6_1 = global$0 - 16 | 0;
  HEAP32[($6_1 + 12 | 0) >> 2] = $0_1;
  HEAP32[($6_1 + 8 | 0) >> 2] = $1_1;
  HEAP32[($6_1 + 4 | 0) >> 2] = $2_1;
  HEAP32[$6_1 >> 2] = $3_1;
  $7_1 = HEAP32[($6_1 + 12 | 0) >> 2] | 0;
  HEAPF32[$7_1 >> 2] = Math_fround(+HEAPF64[(HEAP32[($6_1 + 8 | 0) >> 2] | 0) >> 3]);
  HEAPF32[($7_1 + 4 | 0) >> 2] = Math_fround(+HEAPF64[(HEAP32[($6_1 + 4 | 0) >> 2] | 0) >> 3]);
  HEAPF32[($7_1 + 8 | 0) >> 2] = Math_fround(+HEAPF64[(HEAP32[$6_1 >> 2] | 0) >> 3]);
  return $7_1 | 0;
 }
 
 function $124($0_1, $1_1, $2_1, $3_1) {
  $0_1 = $0_1 | 0;
  $1_1 = +$1_1;
  $2_1 = +$2_1;
  $3_1 = +$3_1;
  var $6_1 = 0, $16_1 = 0, $18_1 = 0;
  $6_1 = global$0 - 64 | 0;
  global$0 = $6_1;
  HEAP32[($6_1 + 60 | 0) >> 2] = $0_1;
  HEAPF64[($6_1 + 48 | 0) >> 3] = $1_1;
  HEAPF64[($6_1 + 40 | 0) >> 3] = $2_1;
  HEAPF64[($6_1 + 32 | 0) >> 3] = $3_1;
  $16_1 = HEAP32[($6_1 + 60 | 0) >> 2] | 0;
  HEAPF64[($6_1 + 24 | 0) >> 3] = +$107(+(+HEAPF64[($6_1 + 48 | 0) >> 3]));
  HEAPF64[($6_1 + 16 | 0) >> 3] = +$107(+(+HEAPF64[($6_1 + 40 | 0) >> 3]));
  HEAPF64[($6_1 + 8 | 0) >> 3] = +$107(+(+HEAPF64[($6_1 + 32 | 0) >> 3]));
  $18_1 = $121(FUNCTION_TABLE[$16_1 | 0]($6_1 + 24 | 0, $6_1 + 16 | 0, $6_1 + 8 | 0) | 0 | 0) | 0;
  global$0 = $6_1 + 64 | 0;
  return $18_1 | 0;
 }
 
 function $125($0_1) {
  $0_1 = $0_1 | 0;
  HEAP32[((global$0 - 16 | 0) + 12 | 0) >> 2] = $0_1;
  return 4 | 0;
 }
 
 function $126($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $4_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $4_1 = $128() | 0;
  global$0 = $3_1 + 16 | 0;
  return $4_1 | 0;
 }
 
 function $127() {
  return 1632 | 0;
 }
 
 function $128() {
  return 1616 | 0;
 }
 
 function $129($0_1) {
  $0_1 = $0_1 | 0;
  HEAP32[((global$0 - 16 | 0) + 12 | 0) >> 2] = $0_1;
  return 1676 | 0;
 }
 
 function $130() {
  return 1676 | 0;
 }
 
 function $131() {
  return 1720 | 0;
 }
 
 function $132() {
  return 1776 | 0;
 }
 
 function $133($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = +$1_1;
  var $4_1 = 0, $8_1 = 0, $10_1 = 0;
  $4_1 = global$0 - 32 | 0;
  global$0 = $4_1;
  HEAP32[($4_1 + 28 | 0) >> 2] = $0_1;
  HEAPF64[($4_1 + 16 | 0) >> 3] = $1_1;
  $8_1 = HEAP32[($4_1 + 28 | 0) >> 2] | 0;
  HEAPF64[($4_1 + 8 | 0) >> 3] = +$107(+(+HEAPF64[($4_1 + 16 | 0) >> 3]));
  $10_1 = $137(FUNCTION_TABLE[$8_1 | 0]($4_1 + 8 | 0) | 0 | 0) | 0;
  global$0 = $4_1 + 32 | 0;
  return $10_1 | 0;
 }
 
 function $134($0_1) {
  $0_1 = $0_1 | 0;
  HEAP32[((global$0 - 16 | 0) + 12 | 0) >> 2] = $0_1;
  return 2 | 0;
 }
 
 function $135($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $4_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $4_1 = $138() | 0;
  global$0 = $3_1 + 16 | 0;
  return $4_1 | 0;
 }
 
 function $136() {
  return 1800 | 0;
 }
 
 function $137($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  return HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0;
 }
 
 function $138() {
  return 1792 | 0;
 }
 
 function $139($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  return HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0;
 }
 
 function $140() {
  return 1804 | 0;
 }
 
 function $141($0_1) {
  $0_1 = $0_1 | 0;
  var i64toi32_i32$0 = 0, i64toi32_i32$1 = 0, i64toi32_i32$2 = 0, $5_1 = 0, $7_1 = 0, $3_1 = 0, $8_1 = 0, $11_1 = 0, $14_1 = 0, $17_1 = 0, $20_1 = 0, $23_1 = 0, $26_1 = 0, $55_1 = 0, $65_1 = 0, $75_1 = 0, $85_1 = 0, $95_1 = 0, $105_1 = 0, $115_1 = 0, $125_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $5_1 = $330(64 | 0) | 0;
  $7_1 = $143(HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0) | 0;
  i64toi32_i32$2 = $7_1;
  i64toi32_i32$0 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$1 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $55_1 = i64toi32_i32$0;
  i64toi32_i32$0 = $5_1;
  HEAP32[i64toi32_i32$0 >> 2] = $55_1;
  HEAP32[(i64toi32_i32$0 + 4 | 0) >> 2] = i64toi32_i32$1;
  $8_1 = 56;
  i64toi32_i32$2 = i64toi32_i32$2 + $8_1 | 0;
  i64toi32_i32$1 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$0 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $65_1 = i64toi32_i32$1;
  i64toi32_i32$1 = $5_1 + $8_1 | 0;
  HEAP32[i64toi32_i32$1 >> 2] = $65_1;
  HEAP32[(i64toi32_i32$1 + 4 | 0) >> 2] = i64toi32_i32$0;
  $11_1 = 48;
  i64toi32_i32$2 = $7_1 + $11_1 | 0;
  i64toi32_i32$0 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$1 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $75_1 = i64toi32_i32$0;
  i64toi32_i32$0 = $5_1 + $11_1 | 0;
  HEAP32[i64toi32_i32$0 >> 2] = $75_1;
  HEAP32[(i64toi32_i32$0 + 4 | 0) >> 2] = i64toi32_i32$1;
  $14_1 = 40;
  i64toi32_i32$2 = $7_1 + $14_1 | 0;
  i64toi32_i32$1 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$0 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $85_1 = i64toi32_i32$1;
  i64toi32_i32$1 = $5_1 + $14_1 | 0;
  HEAP32[i64toi32_i32$1 >> 2] = $85_1;
  HEAP32[(i64toi32_i32$1 + 4 | 0) >> 2] = i64toi32_i32$0;
  $17_1 = 32;
  i64toi32_i32$2 = $7_1 + $17_1 | 0;
  i64toi32_i32$0 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$1 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $95_1 = i64toi32_i32$0;
  i64toi32_i32$0 = $5_1 + $17_1 | 0;
  HEAP32[i64toi32_i32$0 >> 2] = $95_1;
  HEAP32[(i64toi32_i32$0 + 4 | 0) >> 2] = i64toi32_i32$1;
  $20_1 = 24;
  i64toi32_i32$2 = $7_1 + $20_1 | 0;
  i64toi32_i32$1 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$0 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $105_1 = i64toi32_i32$1;
  i64toi32_i32$1 = $5_1 + $20_1 | 0;
  HEAP32[i64toi32_i32$1 >> 2] = $105_1;
  HEAP32[(i64toi32_i32$1 + 4 | 0) >> 2] = i64toi32_i32$0;
  $23_1 = 16;
  i64toi32_i32$2 = $7_1 + $23_1 | 0;
  i64toi32_i32$0 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$1 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $115_1 = i64toi32_i32$0;
  i64toi32_i32$0 = $5_1 + $23_1 | 0;
  HEAP32[i64toi32_i32$0 >> 2] = $115_1;
  HEAP32[(i64toi32_i32$0 + 4 | 0) >> 2] = i64toi32_i32$1;
  $26_1 = 8;
  i64toi32_i32$2 = $7_1 + $26_1 | 0;
  i64toi32_i32$1 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$0 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $125_1 = i64toi32_i32$1;
  i64toi32_i32$1 = $5_1 + $26_1 | 0;
  HEAP32[i64toi32_i32$1 >> 2] = $125_1;
  HEAP32[(i64toi32_i32$1 + 4 | 0) >> 2] = i64toi32_i32$0;
  global$0 = $3_1 + 16 | 0;
  return $5_1 | 0;
 }
 
 function $142() {
  return 1824 | 0;
 }
 
 function $143($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  return HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0;
 }
 
 function $144() {
  $1();
  $13();
  return;
 }
 
 function $145($0_1, $1_1, $2_1, $3_1) {
  $0_1 = $0_1 | 0;
  $1_1 = +$1_1;
  $2_1 = +$2_1;
  $3_1 = +$3_1;
  var $6_1 = 0, $8_1 = 0;
  $6_1 = global$0 - 32 | 0;
  HEAP32[($6_1 + 28 | 0) >> 2] = $0_1;
  HEAPF64[($6_1 + 16 | 0) >> 3] = $1_1;
  HEAPF64[($6_1 + 8 | 0) >> 3] = $2_1;
  HEAPF64[$6_1 >> 3] = $3_1;
  $8_1 = HEAP32[($6_1 + 28 | 0) >> 2] | 0;
  HEAPF64[($8_1 + 8 | 0) >> 3] = +HEAPF64[($6_1 + 16 | 0) >> 3];
  HEAPF64[($8_1 + 16 | 0) >> 3] = +HEAPF64[($6_1 + 8 | 0) >> 3];
  HEAPF64[($8_1 + 24 | 0) >> 3] = +HEAPF64[$6_1 >> 3];
  HEAP8[$8_1 >> 0] = 1;
  return $8_1 | 0;
 }
 
 function $146($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  return (HEAPU8[(HEAP32[($3_1 + 12 | 0) >> 2] | 0) >> 0] | 0) & 1 | 0 | 0;
 }
 
 function $147($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  return +(+HEAPF64[((HEAP32[($3_1 + 12 | 0) >> 2] | 0) + 8 | 0) >> 3]);
 }
 
 function $148($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  return +(+HEAPF64[((HEAP32[($3_1 + 12 | 0) >> 2] | 0) + 16 | 0) >> 3]);
 }
 
 function $149($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  return +(+HEAPF64[((HEAP32[($3_1 + 12 | 0) >> 2] | 0) + 24 | 0) >> 3]);
 }
 
 function $150($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = +$1_1;
  var $4_1 = 0, $15_1 = 0.0;
  $4_1 = global$0 - 16 | 0;
  global$0 = $4_1;
  HEAP32[($4_1 + 12 | 0) >> 2] = $0_1;
  HEAPF64[$4_1 >> 3] = $1_1;
  label$1 : {
   label$2 : {
    if (!(+HEAPF64[$4_1 >> 3] < .0031308 & 1 | 0)) {
     break label$2
    }
    $15_1 = +HEAPF64[$4_1 >> 3] * 12.92;
    break label$1;
   }
   $15_1 = 1.055 * +$329(+(+HEAPF64[$4_1 >> 3]), +(.41666)) - .055;
  }
  global$0 = $4_1 + 16 | 0;
  return +$15_1;
 }
 
 function $151($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  var i64toi32_i32$0 = 0, i64toi32_i32$1 = 0, i64toi32_i32$2 = 0, $4_1 = 0, $7_1 = 0, $8_1 = 0, $9_1 = 0, $12_1 = 0, $15_1 = 0, $80_1 = 0, $90_1 = 0, $100_1 = 0, $110_1 = 0;
  $4_1 = global$0 - 32 | 0;
  global$0 = $4_1;
  HEAP32[($4_1 + 28 | 0) >> 2] = $1_1;
  $7_1 = HEAP32[($4_1 + 28 | 0) >> 2] | 0;
  HEAPF64[($4_1 + 16 | 0) >> 3] = +$150($7_1 | 0, +(+HEAPF64[($7_1 + 8 | 0) >> 3]));
  HEAPF64[($4_1 + 8 | 0) >> 3] = +$150($7_1 | 0, +(+HEAPF64[($7_1 + 16 | 0) >> 3]));
  HEAPF64[$4_1 >> 3] = +$150($7_1 | 0, +(+HEAPF64[($7_1 + 24 | 0) >> 3]));
  $8_1 = $330(32 | 0) | 0;
  FUNCTION_TABLE[27 | 0]($8_1, +HEAPF64[($4_1 + 16 | 0) >> 3], +HEAPF64[($4_1 + 8 | 0) >> 3], +HEAPF64[$4_1 >> 3]) | 0;
  i64toi32_i32$2 = $8_1;
  i64toi32_i32$0 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$1 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $80_1 = i64toi32_i32$0;
  i64toi32_i32$0 = $0_1;
  HEAP32[i64toi32_i32$0 >> 2] = $80_1;
  HEAP32[(i64toi32_i32$0 + 4 | 0) >> 2] = i64toi32_i32$1;
  $9_1 = 24;
  i64toi32_i32$2 = i64toi32_i32$2 + $9_1 | 0;
  i64toi32_i32$1 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$0 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $90_1 = i64toi32_i32$1;
  i64toi32_i32$1 = $0_1 + $9_1 | 0;
  HEAP32[i64toi32_i32$1 >> 2] = $90_1;
  HEAP32[(i64toi32_i32$1 + 4 | 0) >> 2] = i64toi32_i32$0;
  $12_1 = 16;
  i64toi32_i32$2 = $8_1 + $12_1 | 0;
  i64toi32_i32$0 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$1 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $100_1 = i64toi32_i32$0;
  i64toi32_i32$0 = $0_1 + $12_1 | 0;
  HEAP32[i64toi32_i32$0 >> 2] = $100_1;
  HEAP32[(i64toi32_i32$0 + 4 | 0) >> 2] = i64toi32_i32$1;
  $15_1 = 8;
  i64toi32_i32$2 = $8_1 + $15_1 | 0;
  i64toi32_i32$1 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$0 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $110_1 = i64toi32_i32$1;
  i64toi32_i32$1 = $0_1 + $15_1 | 0;
  HEAP32[i64toi32_i32$1 >> 2] = $110_1;
  HEAP32[(i64toi32_i32$1 + 4 | 0) >> 2] = i64toi32_i32$0;
  global$0 = $4_1 + 32 | 0;
  return;
 }
 
 function $152() {
  FUNCTION_TABLE[28 | 0](4714) | 0;
  return;
 }
 
 function $153($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, i64toi32_i32$0 = 0, i64toi32_i32$1 = 0, $35_1 = 0, $41_1 = 0, $4_1 = 0, $12_1 = 0, $13_1 = 0, $14_1 = 0, $15_1 = 0, $17_1 = 0, $18_1 = 0, $20_1 = 0, $21_1 = 0, $23_1 = 0, $24_1 = 0, $25_1 = 0, $33_1 = 0, $217_1 = 0, $37_1 = 0, $38_1 = 0, $40_1 = 0, $42_1 = 0, $43_1 = 0, $44_1 = 0, $46_1 = 0, $47_1 = 0, $48_1 = 0, $256_1 = 0, $279_1 = 0, $54_1 = 0, $55_1 = 0, $57_1 = 0, $58_1 = 0, $59_1 = 0, $60_1 = 0, $62_1 = 0, $63_1 = 0, $64_1 = 0, $318_1 = 0, $341_1 = 0, $70_1 = 0, $71_1 = 0, $73_1 = 0, $74_1 = 0, $75_1 = 0, $76_1 = 0, $78_1 = 0, $79_1 = 0, $80_1 = 0, $380_1 = 0, $403 = 0, $86_1 = 0, $87_1 = 0, $89_1 = 0, $91_1 = 0, $92_1 = 0, $93_1 = 0, $95_1 = 0, $96_1 = 0, $97_1 = 0, $442 = 0, $462 = 0, $101_1 = 0, $102_1 = 0, $104_1 = 0, $105_1 = 0, $488 = 0;
  $3_1 = global$0 - 368 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 88 | 0) >> 2] = $0_1;
  $4_1 = HEAP32[($3_1 + 88 | 0) >> 2] | 0;
  HEAP32[($3_1 + 112 | 0) >> 2] = $3_1 + 80 | 0;
  HEAP32[($3_1 + 108 | 0) >> 2] = 1846;
  $154();
  HEAP32[($3_1 + 104 | 0) >> 2] = 29;
  HEAP32[($3_1 + 100 | 0) >> 2] = $156() | 0;
  HEAP32[($3_1 + 96 | 0) >> 2] = $157() | 0;
  HEAP32[($3_1 + 92 | 0) >> 2] = 30;
  $12_1 = $159() | 0;
  $13_1 = $160() | 0;
  $14_1 = $161() | 0;
  $15_1 = $36() | 0;
  HEAP32[($3_1 + 116 | 0) >> 2] = HEAP32[($3_1 + 104 | 0) >> 2] | 0;
  $17_1 = $37() | 0;
  $18_1 = HEAP32[($3_1 + 104 | 0) >> 2] | 0;
  HEAP32[($3_1 + 120 | 0) >> 2] = HEAP32[($3_1 + 100 | 0) >> 2] | 0;
  $20_1 = $38() | 0;
  $21_1 = HEAP32[($3_1 + 100 | 0) >> 2] | 0;
  HEAP32[($3_1 + 124 | 0) >> 2] = HEAP32[($3_1 + 96 | 0) >> 2] | 0;
  $23_1 = $38() | 0;
  $24_1 = HEAP32[($3_1 + 96 | 0) >> 2] | 0;
  $25_1 = HEAP32[($3_1 + 108 | 0) >> 2] | 0;
  HEAP32[($3_1 + 128 | 0) >> 2] = HEAP32[($3_1 + 92 | 0) >> 2] | 0;
  fimport$2($12_1 | 0, $13_1 | 0, $14_1 | 0, $15_1 | 0, $17_1 | 0, $18_1 | 0, $20_1 | 0, $21_1 | 0, $23_1 | 0, $24_1 | 0, $25_1 | 0, $39() | 0 | 0, HEAP32[($3_1 + 92 | 0) >> 2] | 0 | 0);
  HEAP32[($3_1 + 132 | 0) >> 2] = $3_1 + 80 | 0;
  HEAP32[($3_1 + 140 | 0) >> 2] = HEAP32[($3_1 + 132 | 0) >> 2] | 0;
  HEAP32[($3_1 + 136 | 0) >> 2] = 31;
  $33_1 = HEAP32[($3_1 + 140 | 0) >> 2] | 0;
  $163(HEAP32[($3_1 + 136 | 0) >> 2] | 0 | 0);
  $35_1 = 0;
  HEAP32[($3_1 + 76 | 0) >> 2] = $35_1;
  HEAP32[($3_1 + 72 | 0) >> 2] = 32;
  i64toi32_i32$0 = HEAP32[($3_1 + 72 | 0) >> 2] | 0;
  i64toi32_i32$1 = HEAP32[($3_1 + 76 | 0) >> 2] | 0;
  $217_1 = i64toi32_i32$0;
  i64toi32_i32$0 = $3_1;
  HEAP32[($3_1 + 144 | 0) >> 2] = $217_1;
  HEAP32[($3_1 + 148 | 0) >> 2] = i64toi32_i32$1;
  $37_1 = HEAP32[($3_1 + 144 | 0) >> 2] | 0;
  $38_1 = HEAP32[($3_1 + 148 | 0) >> 2] | 0;
  HEAP32[($3_1 + 184 | 0) >> 2] = $33_1;
  HEAP32[($3_1 + 180 | 0) >> 2] = 1852;
  HEAP32[($3_1 + 172 | 0) >> 2] = $38_1;
  HEAP32[($3_1 + 168 | 0) >> 2] = $37_1;
  $40_1 = HEAP32[($3_1 + 184 | 0) >> 2] | 0;
  $41_1 = 33;
  HEAP32[($3_1 + 164 | 0) >> 2] = $41_1;
  $42_1 = $159() | 0;
  $43_1 = HEAP32[($3_1 + 180 | 0) >> 2] | 0;
  $44_1 = $165() | 0;
  HEAP32[($3_1 + 188 | 0) >> 2] = HEAP32[($3_1 + 164 | 0) >> 2] | 0;
  $46_1 = $166() | 0;
  $47_1 = HEAP32[($3_1 + 164 | 0) >> 2] | 0;
  $48_1 = HEAP32[($3_1 + 168 | 0) >> 2] | 0;
  HEAP32[($3_1 + 156 | 0) >> 2] = HEAP32[($3_1 + 172 | 0) >> 2] | 0;
  HEAP32[($3_1 + 152 | 0) >> 2] = $48_1;
  i64toi32_i32$1 = HEAP32[($3_1 + 152 | 0) >> 2] | 0;
  i64toi32_i32$0 = HEAP32[($3_1 + 156 | 0) >> 2] | 0;
  $256_1 = i64toi32_i32$1;
  i64toi32_i32$1 = $3_1;
  HEAP32[($3_1 + 24 | 0) >> 2] = $256_1;
  HEAP32[($3_1 + 28 | 0) >> 2] = i64toi32_i32$0;
  fimport$3($42_1 | 0, $43_1 | 0, $44_1 | 0, $46_1 | 0, $47_1 | 0, $167($3_1 + 24 | 0 | 0) | 0 | 0, $35_1 | 0, $35_1 | 0, $35_1 | 0, $35_1 | 0);
  HEAP32[($3_1 + 68 | 0) >> 2] = $35_1;
  HEAP32[($3_1 + 64 | 0) >> 2] = 34;
  i64toi32_i32$0 = HEAP32[($3_1 + 64 | 0) >> 2] | 0;
  i64toi32_i32$1 = HEAP32[($3_1 + 68 | 0) >> 2] | 0;
  $279_1 = i64toi32_i32$0;
  i64toi32_i32$0 = $3_1;
  HEAP32[($3_1 + 192 | 0) >> 2] = $279_1;
  HEAP32[($3_1 + 196 | 0) >> 2] = i64toi32_i32$1;
  $54_1 = HEAP32[($3_1 + 192 | 0) >> 2] | 0;
  $55_1 = HEAP32[($3_1 + 196 | 0) >> 2] | 0;
  HEAP32[($3_1 + 232 | 0) >> 2] = $40_1;
  HEAP32[($3_1 + 228 | 0) >> 2] = 1854;
  HEAP32[($3_1 + 220 | 0) >> 2] = $55_1;
  HEAP32[($3_1 + 216 | 0) >> 2] = $54_1;
  $57_1 = HEAP32[($3_1 + 232 | 0) >> 2] | 0;
  HEAP32[($3_1 + 212 | 0) >> 2] = $41_1;
  $58_1 = $159() | 0;
  $59_1 = HEAP32[($3_1 + 228 | 0) >> 2] | 0;
  $60_1 = $165() | 0;
  HEAP32[($3_1 + 236 | 0) >> 2] = HEAP32[($3_1 + 212 | 0) >> 2] | 0;
  $62_1 = $166() | 0;
  $63_1 = HEAP32[($3_1 + 212 | 0) >> 2] | 0;
  $64_1 = HEAP32[($3_1 + 216 | 0) >> 2] | 0;
  HEAP32[($3_1 + 204 | 0) >> 2] = HEAP32[($3_1 + 220 | 0) >> 2] | 0;
  HEAP32[($3_1 + 200 | 0) >> 2] = $64_1;
  i64toi32_i32$1 = HEAP32[($3_1 + 200 | 0) >> 2] | 0;
  i64toi32_i32$0 = HEAP32[($3_1 + 204 | 0) >> 2] | 0;
  $318_1 = i64toi32_i32$1;
  i64toi32_i32$1 = $3_1;
  HEAP32[($3_1 + 16 | 0) >> 2] = $318_1;
  HEAP32[($3_1 + 20 | 0) >> 2] = i64toi32_i32$0;
  fimport$3($58_1 | 0, $59_1 | 0, $60_1 | 0, $62_1 | 0, $63_1 | 0, $167($3_1 + 16 | 0 | 0) | 0 | 0, $35_1 | 0, $35_1 | 0, $35_1 | 0, $35_1 | 0);
  HEAP32[($3_1 + 60 | 0) >> 2] = $35_1;
  HEAP32[($3_1 + 56 | 0) >> 2] = 35;
  i64toi32_i32$0 = HEAP32[($3_1 + 56 | 0) >> 2] | 0;
  i64toi32_i32$1 = HEAP32[($3_1 + 60 | 0) >> 2] | 0;
  $341_1 = i64toi32_i32$0;
  i64toi32_i32$0 = $3_1;
  HEAP32[($3_1 + 240 | 0) >> 2] = $341_1;
  HEAP32[($3_1 + 244 | 0) >> 2] = i64toi32_i32$1;
  $70_1 = HEAP32[($3_1 + 240 | 0) >> 2] | 0;
  $71_1 = HEAP32[($3_1 + 244 | 0) >> 2] | 0;
  HEAP32[($3_1 + 280 | 0) >> 2] = $57_1;
  HEAP32[($3_1 + 276 | 0) >> 2] = 1856;
  HEAP32[($3_1 + 268 | 0) >> 2] = $71_1;
  HEAP32[($3_1 + 264 | 0) >> 2] = $70_1;
  $73_1 = HEAP32[($3_1 + 280 | 0) >> 2] | 0;
  HEAP32[($3_1 + 260 | 0) >> 2] = $41_1;
  $74_1 = $159() | 0;
  $75_1 = HEAP32[($3_1 + 276 | 0) >> 2] | 0;
  $76_1 = $165() | 0;
  HEAP32[($3_1 + 284 | 0) >> 2] = HEAP32[($3_1 + 260 | 0) >> 2] | 0;
  $78_1 = $166() | 0;
  $79_1 = HEAP32[($3_1 + 260 | 0) >> 2] | 0;
  $80_1 = HEAP32[($3_1 + 264 | 0) >> 2] | 0;
  HEAP32[($3_1 + 252 | 0) >> 2] = HEAP32[($3_1 + 268 | 0) >> 2] | 0;
  HEAP32[($3_1 + 248 | 0) >> 2] = $80_1;
  i64toi32_i32$1 = HEAP32[($3_1 + 248 | 0) >> 2] | 0;
  i64toi32_i32$0 = HEAP32[($3_1 + 252 | 0) >> 2] | 0;
  $380_1 = i64toi32_i32$1;
  i64toi32_i32$1 = $3_1;
  HEAP32[($3_1 + 8 | 0) >> 2] = $380_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = i64toi32_i32$0;
  fimport$3($74_1 | 0, $75_1 | 0, $76_1 | 0, $78_1 | 0, $79_1 | 0, $167($3_1 + 8 | 0 | 0) | 0 | 0, $35_1 | 0, $35_1 | 0, $35_1 | 0, $35_1 | 0);
  HEAP32[($3_1 + 52 | 0) >> 2] = $35_1;
  HEAP32[($3_1 + 48 | 0) >> 2] = 36;
  i64toi32_i32$0 = HEAP32[($3_1 + 48 | 0) >> 2] | 0;
  i64toi32_i32$1 = HEAP32[($3_1 + 52 | 0) >> 2] | 0;
  $403 = i64toi32_i32$0;
  i64toi32_i32$0 = $3_1;
  HEAP32[($3_1 + 288 | 0) >> 2] = $403;
  HEAP32[($3_1 + 292 | 0) >> 2] = i64toi32_i32$1;
  $86_1 = HEAP32[($3_1 + 288 | 0) >> 2] | 0;
  $87_1 = HEAP32[($3_1 + 292 | 0) >> 2] | 0;
  HEAP32[($3_1 + 328 | 0) >> 2] = $73_1;
  HEAP32[($3_1 + 324 | 0) >> 2] = 1858;
  HEAP32[($3_1 + 316 | 0) >> 2] = $87_1;
  HEAP32[($3_1 + 312 | 0) >> 2] = $86_1;
  $89_1 = HEAP32[($3_1 + 328 | 0) >> 2] | 0;
  HEAP32[($3_1 + 308 | 0) >> 2] = 37;
  $91_1 = $159() | 0;
  $92_1 = HEAP32[($3_1 + 324 | 0) >> 2] | 0;
  $93_1 = $169() | 0;
  HEAP32[($3_1 + 332 | 0) >> 2] = HEAP32[($3_1 + 308 | 0) >> 2] | 0;
  $95_1 = $170() | 0;
  $96_1 = HEAP32[($3_1 + 308 | 0) >> 2] | 0;
  $97_1 = HEAP32[($3_1 + 312 | 0) >> 2] | 0;
  HEAP32[($3_1 + 300 | 0) >> 2] = HEAP32[($3_1 + 316 | 0) >> 2] | 0;
  HEAP32[($3_1 + 296 | 0) >> 2] = $97_1;
  i64toi32_i32$1 = HEAP32[($3_1 + 296 | 0) >> 2] | 0;
  i64toi32_i32$0 = HEAP32[($3_1 + 300 | 0) >> 2] | 0;
  $442 = i64toi32_i32$1;
  i64toi32_i32$1 = $3_1;
  HEAP32[$3_1 >> 2] = $442;
  HEAP32[($3_1 + 4 | 0) >> 2] = i64toi32_i32$0;
  fimport$3($91_1 | 0, $92_1 | 0, $93_1 | 0, $95_1 | 0, $96_1 | 0, $171($3_1 | 0) | 0 | 0, $35_1 | 0, $35_1 | 0, $35_1 | 0, $35_1 | 0);
  HEAP32[($3_1 + 44 | 0) >> 2] = $35_1;
  HEAP32[($3_1 + 40 | 0) >> 2] = 38;
  i64toi32_i32$0 = HEAP32[($3_1 + 40 | 0) >> 2] | 0;
  i64toi32_i32$1 = HEAP32[($3_1 + 44 | 0) >> 2] | 0;
  $462 = i64toi32_i32$0;
  i64toi32_i32$0 = $3_1;
  HEAP32[($3_1 + 336 | 0) >> 2] = $462;
  HEAP32[($3_1 + 340 | 0) >> 2] = i64toi32_i32$1;
  $101_1 = HEAP32[($3_1 + 336 | 0) >> 2] | 0;
  $102_1 = HEAP32[($3_1 + 340 | 0) >> 2] | 0;
  HEAP32[($3_1 + 364 | 0) >> 2] = $89_1;
  HEAP32[($3_1 + 360 | 0) >> 2] = 1866;
  HEAP32[($3_1 + 356 | 0) >> 2] = $102_1;
  HEAP32[($3_1 + 352 | 0) >> 2] = $101_1;
  $104_1 = HEAP32[($3_1 + 360 | 0) >> 2] | 0;
  $105_1 = HEAP32[($3_1 + 352 | 0) >> 2] | 0;
  HEAP32[($3_1 + 348 | 0) >> 2] = HEAP32[($3_1 + 356 | 0) >> 2] | 0;
  HEAP32[($3_1 + 344 | 0) >> 2] = $105_1;
  i64toi32_i32$1 = HEAP32[($3_1 + 344 | 0) >> 2] | 0;
  i64toi32_i32$0 = HEAP32[($3_1 + 348 | 0) >> 2] | 0;
  $488 = i64toi32_i32$1;
  i64toi32_i32$1 = $3_1;
  HEAP32[($3_1 + 32 | 0) >> 2] = $488;
  HEAP32[($3_1 + 36 | 0) >> 2] = i64toi32_i32$0;
  $172($104_1 | 0, $3_1 + 32 | 0 | 0);
  global$0 = $3_1 + 368 | 0;
  return $4_1 | 0;
 }
 
 function $154() {
  return;
 }
 
 function $155($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $5_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $5_1 = $173(HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0) | 0;
  global$0 = $3_1 + 16 | 0;
  return $5_1 | 0;
 }
 
 function $156() {
  return 0 | 0;
 }
 
 function $157() {
  return 0 | 0;
 }
 
 function $158($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $5_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $5_1 = HEAP32[($3_1 + 12 | 0) >> 2] | 0;
  label$1 : {
   if (($5_1 | 0) == (0 | 0) & 1 | 0) {
    break label$1
   }
   $331($5_1 | 0);
  }
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $159() {
  return $174() | 0 | 0;
 }
 
 function $160() {
  return $175() | 0 | 0;
 }
 
 function $161() {
  return $176() | 0 | 0;
 }
 
 function $162($0_1, $1_1, $2_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  var $5_1 = 0, $8_1 = 0;
  $5_1 = global$0 - 16 | 0;
  global$0 = $5_1;
  HEAP32[($5_1 + 12 | 0) >> 2] = $0_1;
  HEAP32[($5_1 + 8 | 0) >> 2] = $1_1;
  HEAP32[($5_1 + 4 | 0) >> 2] = $2_1;
  $8_1 = $330(32 | 0) | 0;
  FUNCTION_TABLE[27 | 0]($8_1, +HEAPF64[($101(HEAP32[($5_1 + 12 | 0) >> 2] | 0 | 0) | 0) >> 3], +HEAPF64[($101(HEAP32[($5_1 + 8 | 0) >> 2] | 0 | 0) | 0) >> 3], +HEAPF64[($101(HEAP32[($5_1 + 4 | 0) >> 2] | 0 | 0) | 0) >> 3]) | 0;
  global$0 = $5_1 + 16 | 0;
  return $8_1 | 0;
 }
 
 function $163($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $6_1 = 0, $8_1 = 0, $9_1 = 0, $10_1 = 0;
  $3_1 = global$0 - 32 | 0;
  global$0 = $3_1;
  $6_1 = $3_1 + 16 | 0;
  HEAP32[($3_1 + 24 | 0) >> 2] = $0_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = 39;
  $8_1 = $159() | 0;
  $9_1 = $178($6_1 | 0) | 0;
  $10_1 = $179($6_1 | 0) | 0;
  HEAP32[($3_1 + 28 | 0) >> 2] = HEAP32[($3_1 + 12 | 0) >> 2] | 0;
  fimport$6($8_1 | 0, $9_1 | 0, $10_1 | 0, $127() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0, HEAP32[($3_1 + 24 | 0) >> 2] | 0 | 0);
  global$0 = $3_1 + 32 | 0;
  return;
 }
 
 function $164($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  var $4_1 = 0, $6_1 = 0, $7_1 = 0, $8_1 = 0, $11_1 = 0, $17_1 = 0, $23_1 = 0.0;
  $4_1 = global$0 - 16 | 0;
  global$0 = $4_1;
  HEAP32[($4_1 + 12 | 0) >> 2] = $0_1;
  HEAP32[($4_1 + 8 | 0) >> 2] = $1_1;
  $6_1 = HEAP32[($4_1 + 12 | 0) >> 2] | 0;
  $7_1 = HEAP32[($6_1 + 4 | 0) >> 2] | 0;
  $8_1 = HEAP32[$6_1 >> 2] | 0;
  $11_1 = (HEAP32[($4_1 + 8 | 0) >> 2] | 0) + ($7_1 >> 1 | 0) | 0;
  label$1 : {
   label$2 : {
    if (!($7_1 & 1 | 0)) {
     break label$2
    }
    $17_1 = HEAP32[((HEAP32[$11_1 >> 2] | 0) + $8_1 | 0) >> 2] | 0;
    break label$1;
   }
   $17_1 = $8_1;
  }
  HEAPF64[$4_1 >> 3] = +FUNCTION_TABLE[$17_1 | 0]($11_1);
  $23_1 = +$182($4_1 | 0);
  global$0 = $4_1 + 16 | 0;
  return +$23_1;
 }
 
 function $165() {
  return $183() | 0 | 0;
 }
 
 function $166() {
  return 1984 | 0;
 }
 
 function $167($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $7_1 = 0, $9_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  $7_1 = HEAP32[$0_1 >> 2] | 0;
  HEAP32[($3_1 + 12 | 0) >> 2] = HEAP32[($0_1 + 4 | 0) >> 2] | 0;
  HEAP32[($3_1 + 8 | 0) >> 2] = $7_1;
  $9_1 = $184($3_1 + 8 | 0 | 0) | 0;
  global$0 = $3_1 + 16 | 0;
  return $9_1 | 0;
 }
 
 function $168($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  var $4_1 = 0, $6_1 = 0, $7_1 = 0, $8_1 = 0, $11_1 = 0, $17_1 = 0, $24_1 = 0;
  $4_1 = global$0 - 16 | 0;
  global$0 = $4_1;
  HEAP32[($4_1 + 12 | 0) >> 2] = $0_1;
  HEAP32[($4_1 + 8 | 0) >> 2] = $1_1;
  $6_1 = HEAP32[($4_1 + 12 | 0) >> 2] | 0;
  $7_1 = HEAP32[($6_1 + 4 | 0) >> 2] | 0;
  $8_1 = HEAP32[$6_1 >> 2] | 0;
  $11_1 = (HEAP32[($4_1 + 8 | 0) >> 2] | 0) + ($7_1 >> 1 | 0) | 0;
  label$1 : {
   label$2 : {
    if (!($7_1 & 1 | 0)) {
     break label$2
    }
    $17_1 = HEAP32[((HEAP32[$11_1 >> 2] | 0) + $8_1 | 0) >> 2] | 0;
    break label$1;
   }
   $17_1 = $8_1;
  }
  $24_1 = ($185((FUNCTION_TABLE[$17_1 | 0]($11_1) | 0) & 1 | 0 | 0) | 0) & 1 | 0;
  global$0 = $4_1 + 16 | 0;
  return $24_1 | 0;
 }
 
 function $169() {
  return $186() | 0 | 0;
 }
 
 function $170() {
  return 1988 | 0;
 }
 
 function $171($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $7_1 = 0, $9_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  $7_1 = HEAP32[$0_1 >> 2] | 0;
  HEAP32[($3_1 + 12 | 0) >> 2] = HEAP32[($0_1 + 4 | 0) >> 2] | 0;
  HEAP32[($3_1 + 8 | 0) >> 2] = $7_1;
  $9_1 = $187($3_1 + 8 | 0 | 0) | 0;
  global$0 = $3_1 + 16 | 0;
  return $9_1 | 0;
 }
 
 function $172($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  var $4_1 = 0, $11_1 = 0, $13_1 = 0, $14_1 = 0, $15_1 = 0, $16_1 = 0, $17_1 = 0, $18_1 = 0;
  $4_1 = global$0 - 32 | 0;
  global$0 = $4_1;
  $11_1 = $4_1 + 8 | 0;
  $13_1 = HEAP32[$1_1 >> 2] | 0;
  $14_1 = HEAP32[($1_1 + 4 | 0) >> 2] | 0;
  HEAP32[($4_1 + 24 | 0) >> 2] = $0_1;
  HEAP32[($4_1 + 20 | 0) >> 2] = $14_1;
  HEAP32[($4_1 + 16 | 0) >> 2] = $13_1;
  HEAP32[($4_1 + 12 | 0) >> 2] = 40;
  $15_1 = $159() | 0;
  $16_1 = HEAP32[($4_1 + 24 | 0) >> 2] | 0;
  $17_1 = $189($11_1 | 0) | 0;
  $18_1 = $190($11_1 | 0) | 0;
  HEAP32[($4_1 + 28 | 0) >> 2] = HEAP32[($4_1 + 12 | 0) >> 2] | 0;
  fimport$7($15_1 | 0, $16_1 | 0, $17_1 | 0, $18_1 | 0, $170() | 0 | 0, HEAP32[($4_1 + 12 | 0) >> 2] | 0 | 0, $191($4_1 + 16 | 0 | 0) | 0 | 0, 0 | 0);
  global$0 = $4_1 + 32 | 0;
  return;
 }
 
 function $173($0_1) {
  $0_1 = $0_1 | 0;
  HEAP32[((global$0 - 16 | 0) + 12 | 0) >> 2] = $0_1;
  return 1896 | 0;
 }
 
 function $174() {
  return 1896 | 0;
 }
 
 function $175() {
  return 1912 | 0;
 }
 
 function $176() {
  return 1940 | 0;
 }
 
 function $177($0_1, $1_1, $2_1, $3_1) {
  $0_1 = $0_1 | 0;
  $1_1 = +$1_1;
  $2_1 = +$2_1;
  $3_1 = +$3_1;
  var $6_1 = 0, $16_1 = 0, $18_1 = 0;
  $6_1 = global$0 - 64 | 0;
  global$0 = $6_1;
  HEAP32[($6_1 + 60 | 0) >> 2] = $0_1;
  HEAPF64[($6_1 + 48 | 0) >> 3] = $1_1;
  HEAPF64[($6_1 + 40 | 0) >> 3] = $2_1;
  HEAPF64[($6_1 + 32 | 0) >> 3] = $3_1;
  $16_1 = HEAP32[($6_1 + 60 | 0) >> 2] | 0;
  HEAPF64[($6_1 + 24 | 0) >> 3] = +$107(+(+HEAPF64[($6_1 + 48 | 0) >> 3]));
  HEAPF64[($6_1 + 16 | 0) >> 3] = +$107(+(+HEAPF64[($6_1 + 40 | 0) >> 3]));
  HEAPF64[($6_1 + 8 | 0) >> 3] = +$107(+(+HEAPF64[($6_1 + 32 | 0) >> 3]));
  $18_1 = $180(FUNCTION_TABLE[$16_1 | 0]($6_1 + 24 | 0, $6_1 + 16 | 0, $6_1 + 8 | 0) | 0 | 0) | 0;
  global$0 = $6_1 + 64 | 0;
  return $18_1 | 0;
 }
 
 function $178($0_1) {
  $0_1 = $0_1 | 0;
  HEAP32[((global$0 - 16 | 0) + 12 | 0) >> 2] = $0_1;
  return 4 | 0;
 }
 
 function $179($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $4_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $4_1 = $181() | 0;
  global$0 = $3_1 + 16 | 0;
  return $4_1 | 0;
 }
 
 function $180($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  return HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0;
 }
 
 function $181() {
  return 1968 | 0;
 }
 
 function $182($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  return +(+HEAPF64[(HEAP32[($3_1 + 12 | 0) >> 2] | 0) >> 3]);
 }
 
 function $183() {
  return 4376 | 0;
 }
 
 function $184($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $5_1 = 0, $7_1 = 0, $8_1 = 0, $10_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $5_1 = $330(8 | 0) | 0;
  $7_1 = HEAP32[($3_1 + 12 | 0) >> 2] | 0;
  $8_1 = HEAP32[$7_1 >> 2] | 0;
  HEAP32[($5_1 + 4 | 0) >> 2] = HEAP32[($7_1 + 4 | 0) >> 2] | 0;
  HEAP32[$5_1 >> 2] = $8_1;
  HEAP32[($3_1 + 8 | 0) >> 2] = $5_1;
  $10_1 = HEAP32[($3_1 + 8 | 0) >> 2] | 0;
  global$0 = $3_1 + 16 | 0;
  return $10_1 | 0;
 }
 
 function $185($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  HEAP8[($3_1 + 15 | 0) >> 0] = $0_1;
  return (HEAPU8[($3_1 + 15 | 0) >> 0] | 0) & 1 | 0 | 0;
 }
 
 function $186() {
  return 4244 | 0;
 }
 
 function $187($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $5_1 = 0, $7_1 = 0, $8_1 = 0, $10_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $5_1 = $330(8 | 0) | 0;
  $7_1 = HEAP32[($3_1 + 12 | 0) >> 2] | 0;
  $8_1 = HEAP32[$7_1 >> 2] | 0;
  HEAP32[($5_1 + 4 | 0) >> 2] = HEAP32[($7_1 + 4 | 0) >> 2] | 0;
  HEAP32[$5_1 >> 2] = $8_1;
  HEAP32[($3_1 + 8 | 0) >> 2] = $5_1;
  $10_1 = HEAP32[($3_1 + 8 | 0) >> 2] | 0;
  global$0 = $3_1 + 16 | 0;
  return $10_1 | 0;
 }
 
 function $188($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  var $4_1 = 0, $7_1 = 0, $8_1 = 0, $9_1 = 0, $12_1 = 0, $18_1 = 0, $22_1 = 0, $6_1 = 0, $23_1 = 0;
  $4_1 = global$0 - 48 | 0;
  global$0 = $4_1;
  HEAP32[($4_1 + 44 | 0) >> 2] = $0_1;
  HEAP32[($4_1 + 40 | 0) >> 2] = $1_1;
  $6_1 = $192(HEAP32[($4_1 + 40 | 0) >> 2] | 0 | 0) | 0;
  $7_1 = HEAP32[($4_1 + 44 | 0) >> 2] | 0;
  $8_1 = HEAP32[($7_1 + 4 | 0) >> 2] | 0;
  $9_1 = HEAP32[$7_1 >> 2] | 0;
  $12_1 = $6_1 + ($8_1 >> 1 | 0) | 0;
  label$1 : {
   label$2 : {
    if (!($8_1 & 1 | 0)) {
     break label$2
    }
    $18_1 = HEAP32[((HEAP32[$12_1 >> 2] | 0) + $9_1 | 0) >> 2] | 0;
    break label$1;
   }
   $18_1 = $9_1;
  }
  $22_1 = $4_1 + 8 | 0;
  FUNCTION_TABLE[$18_1 | 0]($22_1, $12_1);
  $23_1 = $193($22_1 | 0) | 0;
  global$0 = $4_1 + 48 | 0;
  return $23_1 | 0;
 }
 
 function $189($0_1) {
  $0_1 = $0_1 | 0;
  HEAP32[((global$0 - 16 | 0) + 12 | 0) >> 2] = $0_1;
  return 2 | 0;
 }
 
 function $190($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $4_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $4_1 = $194() | 0;
  global$0 = $3_1 + 16 | 0;
  return $4_1 | 0;
 }
 
 function $191($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $5_1 = 0, $7_1 = 0, $8_1 = 0, $10_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $5_1 = $330(8 | 0) | 0;
  $7_1 = HEAP32[($3_1 + 12 | 0) >> 2] | 0;
  $8_1 = HEAP32[$7_1 >> 2] | 0;
  HEAP32[($5_1 + 4 | 0) >> 2] = HEAP32[($7_1 + 4 | 0) >> 2] | 0;
  HEAP32[$5_1 >> 2] = $8_1;
  HEAP32[($3_1 + 8 | 0) >> 2] = $5_1;
  $10_1 = HEAP32[($3_1 + 8 | 0) >> 2] | 0;
  global$0 = $3_1 + 16 | 0;
  return $10_1 | 0;
 }
 
 function $192($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  return HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0;
 }
 
 function $193($0_1) {
  $0_1 = $0_1 | 0;
  var i64toi32_i32$0 = 0, i64toi32_i32$1 = 0, i64toi32_i32$2 = 0, $5_1 = 0, $3_1 = 0, $7_1 = 0, $8_1 = 0, $11_1 = 0, $14_1 = 0, $39_1 = 0, $49_1 = 0, $59_1 = 0, $69_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $5_1 = $330(32 | 0) | 0;
  $7_1 = $195(HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0) | 0;
  i64toi32_i32$2 = $7_1;
  i64toi32_i32$0 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$1 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $39_1 = i64toi32_i32$0;
  i64toi32_i32$0 = $5_1;
  HEAP32[i64toi32_i32$0 >> 2] = $39_1;
  HEAP32[(i64toi32_i32$0 + 4 | 0) >> 2] = i64toi32_i32$1;
  $8_1 = 24;
  i64toi32_i32$2 = i64toi32_i32$2 + $8_1 | 0;
  i64toi32_i32$1 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$0 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $49_1 = i64toi32_i32$1;
  i64toi32_i32$1 = $5_1 + $8_1 | 0;
  HEAP32[i64toi32_i32$1 >> 2] = $49_1;
  HEAP32[(i64toi32_i32$1 + 4 | 0) >> 2] = i64toi32_i32$0;
  $11_1 = 16;
  i64toi32_i32$2 = $7_1 + $11_1 | 0;
  i64toi32_i32$0 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$1 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $59_1 = i64toi32_i32$0;
  i64toi32_i32$0 = $5_1 + $11_1 | 0;
  HEAP32[i64toi32_i32$0 >> 2] = $59_1;
  HEAP32[(i64toi32_i32$0 + 4 | 0) >> 2] = i64toi32_i32$1;
  $14_1 = 8;
  i64toi32_i32$2 = $7_1 + $14_1 | 0;
  i64toi32_i32$1 = HEAP32[i64toi32_i32$2 >> 2] | 0;
  i64toi32_i32$0 = HEAP32[(i64toi32_i32$2 + 4 | 0) >> 2] | 0;
  $69_1 = i64toi32_i32$1;
  i64toi32_i32$1 = $5_1 + $14_1 | 0;
  HEAP32[i64toi32_i32$1 >> 2] = $69_1;
  HEAP32[(i64toi32_i32$1 + 4 | 0) >> 2] = i64toi32_i32$0;
  global$0 = $3_1 + 16 | 0;
  return $5_1 | 0;
 }
 
 function $194() {
  return 1992 | 0;
 }
 
 function $195($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  return HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0;
 }
 
 function $196() {
  $152();
  return;
 }
 
 function $197($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $6_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $6_1 = $326($198(HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0) | 0 | 0) | 0;
  global$0 = $3_1 + 16 | 0;
  return $6_1 | 0;
 }
 
 function $198($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  HEAP32[($3_1 + 8 | 0) >> 2] = $0_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = HEAP32[((HEAP32[($3_1 + 8 | 0) >> 2] | 0) + 4 | 0) >> 2] | 0;
  return HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0;
 }
 
 function $199() {
  var $18_1 = 0;
  $18_1 = 4;
  fimport$8($200() | 0 | 0, 2e3 | 0);
  fimport$9($169() | 0 | 0, 2005 | 0, 1 | 0, 1 & 1 | 0 | 0, 0 & 1 | 0 | 0);
  $201(2010 | 0);
  $202(2015 | 0);
  $203(2027 | 0);
  $204(2041 | 0);
  $205(2047 | 0);
  $206(2062 | 0);
  $207(2066 | 0);
  $208(2079 | 0);
  $209(2084 | 0);
  $210(2098 | 0);
  $211(2104 | 0);
  fimport$10($212() | 0 | 0, 2111 | 0);
  fimport$10($213() | 0 | 0, 2123 | 0);
  fimport$11($214() | 0 | 0, $18_1 | 0, 2156 | 0);
  fimport$11($215() | 0 | 0, 2 | 0, 2169 | 0);
  fimport$11($216() | 0 | 0, $18_1 | 0, 2184 | 0);
  fimport$12($217() | 0 | 0, 2199 | 0);
  $218(2215 | 0);
  $219(2245 | 0);
  $220(2282 | 0);
  $221(2321 | 0);
  $222(2352 | 0);
  $223(2392 | 0);
  $224(2421 | 0);
  $225(2459 | 0);
  $226(2489 | 0);
  $219(2528 | 0);
  $220(2560 | 0);
  $221(2593 | 0);
  $222(2626 | 0);
  $223(2660 | 0);
  $224(2693 | 0);
  $227(2727 | 0);
  $228(2758 | 0);
  return;
 }
 
 function $200() {
  return $229() | 0 | 0;
 }
 
 function $201($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $8_1 = 0, $12_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $8_1 = 24;
  $12_1 = 24;
  fimport$13($230() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0, 1 | 0, (($231() | 0) << $8_1 | 0) >> $8_1 | 0 | 0, (($232() | 0) << $12_1 | 0) >> $12_1 | 0 | 0);
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $202($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $8_1 = 0, $12_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $8_1 = 24;
  $12_1 = 24;
  fimport$13($233() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0, 1 | 0, (($234() | 0) << $8_1 | 0) >> $8_1 | 0 | 0, (($235() | 0) << $12_1 | 0) >> $12_1 | 0 | 0);
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $203($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  fimport$13($236() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0, 1 | 0, ($237() | 0) & 255 | 0 | 0, ($238() | 0) & 255 | 0 | 0);
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $204($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $8_1 = 0, $12_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $8_1 = 16;
  $12_1 = 16;
  fimport$13($239() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0, 2 | 0, (($240() | 0) << $8_1 | 0) >> $8_1 | 0 | 0, (($241() | 0) << $12_1 | 0) >> $12_1 | 0 | 0);
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $205($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  fimport$13($242() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0, 2 | 0, ($243() | 0) & 65535 | 0 | 0, ($244() | 0) & 65535 | 0 | 0);
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $206($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  fimport$13($245() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0, 4 | 0, $246() | 0 | 0, $247() | 0 | 0);
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $207($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  fimport$13($248() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0, 4 | 0, $249() | 0 | 0, $250() | 0 | 0);
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $208($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  fimport$13($251() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0, 4 | 0, $252() | 0 | 0, $253() | 0 | 0);
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $209($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  fimport$13($254() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0, 4 | 0, $255() | 0 | 0, $256() | 0 | 0);
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $210($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  fimport$14($42() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0, 4 | 0);
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $211($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  fimport$14($165() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0, 8 | 0);
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $212() {
  return $257() | 0 | 0;
 }
 
 function $213() {
  return $258() | 0 | 0;
 }
 
 function $214() {
  return $259() | 0 | 0;
 }
 
 function $215() {
  return $260() | 0 | 0;
 }
 
 function $216() {
  return $261() | 0 | 0;
 }
 
 function $217() {
  return $262() | 0 | 0;
 }
 
 function $218($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  fimport$15($263() | 0 | 0, $264() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0);
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $219($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  fimport$15($265() | 0 | 0, $266() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0);
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $220($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  fimport$15($267() | 0 | 0, $268() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0);
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $221($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  fimport$15($269() | 0 | 0, $270() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0);
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $222($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  fimport$15($271() | 0 | 0, $272() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0);
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $223($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  fimport$15($273() | 0 | 0, $274() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0);
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $224($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  fimport$15($275() | 0 | 0, $276() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0);
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $225($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  fimport$15($277() | 0 | 0, $278() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0);
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $226($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  fimport$15($279() | 0 | 0, $280() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0);
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $227($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  fimport$15($281() | 0 | 0, $282() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0);
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $228($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  fimport$15($283() | 0 | 0, $284() | 0 | 0, HEAP32[($3_1 + 12 | 0) >> 2] | 0 | 0);
  global$0 = $3_1 + 16 | 0;
  return;
 }
 
 function $229() {
  return 4220 | 0;
 }
 
 function $230() {
  return $287() | 0 | 0;
 }
 
 function $231() {
  var $1_1 = 0;
  $1_1 = 24;
  return (($288() | 0) << $1_1 | 0) >> $1_1 | 0 | 0;
 }
 
 function $232() {
  var $1_1 = 0;
  $1_1 = 24;
  return (($289() | 0) << $1_1 | 0) >> $1_1 | 0 | 0;
 }
 
 function $233() {
  return $290() | 0 | 0;
 }
 
 function $234() {
  var $1_1 = 0;
  $1_1 = 24;
  return (($291() | 0) << $1_1 | 0) >> $1_1 | 0 | 0;
 }
 
 function $235() {
  var $1_1 = 0;
  $1_1 = 24;
  return (($292() | 0) << $1_1 | 0) >> $1_1 | 0 | 0;
 }
 
 function $236() {
  return $293() | 0 | 0;
 }
 
 function $237() {
  return ($294() | 0) & 255 | 0 | 0;
 }
 
 function $238() {
  return ($295() | 0) & 255 | 0 | 0;
 }
 
 function $239() {
  return $296() | 0 | 0;
 }
 
 function $240() {
  var $1_1 = 0;
  $1_1 = 16;
  return (($297() | 0) << $1_1 | 0) >> $1_1 | 0 | 0;
 }
 
 function $241() {
  var $1_1 = 0;
  $1_1 = 16;
  return (($298() | 0) << $1_1 | 0) >> $1_1 | 0 | 0;
 }
 
 function $242() {
  return $299() | 0 | 0;
 }
 
 function $243() {
  return ($300() | 0) & 65535 | 0 | 0;
 }
 
 function $244() {
  return ($301() | 0) & 65535 | 0 | 0;
 }
 
 function $245() {
  return $302() | 0 | 0;
 }
 
 function $246() {
  return $303() | 0 | 0;
 }
 
 function $247() {
  return $304() | 0 | 0;
 }
 
 function $248() {
  return $305() | 0 | 0;
 }
 
 function $249() {
  return $306() | 0 | 0;
 }
 
 function $250() {
  return $307() | 0 | 0;
 }
 
 function $251() {
  return $308() | 0 | 0;
 }
 
 function $252() {
  return $309() | 0 | 0;
 }
 
 function $253() {
  return $310() | 0 | 0;
 }
 
 function $254() {
  return $311() | 0 | 0;
 }
 
 function $255() {
  return $312() | 0 | 0;
 }
 
 function $256() {
  return $313() | 0 | 0;
 }
 
 function $257() {
  return 2900 | 0;
 }
 
 function $258() {
  return 2988 | 0;
 }
 
 function $259() {
  return 3076 | 0;
 }
 
 function $260() {
  return 3168 | 0;
 }
 
 function $261() {
  return 3260 | 0;
 }
 
 function $262() {
  return 3304 | 0;
 }
 
 function $263() {
  return $314() | 0 | 0;
 }
 
 function $264() {
  return 0 | 0;
 }
 
 function $265() {
  return $315() | 0 | 0;
 }
 
 function $266() {
  return 0 | 0;
 }
 
 function $267() {
  return $316() | 0 | 0;
 }
 
 function $268() {
  return 1 | 0;
 }
 
 function $269() {
  return $317() | 0 | 0;
 }
 
 function $270() {
  return 2 | 0;
 }
 
 function $271() {
  return $318() | 0 | 0;
 }
 
 function $272() {
  return 3 | 0;
 }
 
 function $273() {
  return $319() | 0 | 0;
 }
 
 function $274() {
  return 4 | 0;
 }
 
 function $275() {
  return $320() | 0 | 0;
 }
 
 function $276() {
  return 5 | 0;
 }
 
 function $277() {
  return $321() | 0 | 0;
 }
 
 function $278() {
  return 4 | 0;
 }
 
 function $279() {
  return $322() | 0 | 0;
 }
 
 function $280() {
  return 5 | 0;
 }
 
 function $281() {
  return $323() | 0 | 0;
 }
 
 function $282() {
  return 6 | 0;
 }
 
 function $283() {
  return $324() | 0 | 0;
 }
 
 function $284() {
  return 7 | 0;
 }
 
 function $285() {
  FUNCTION_TABLE[41 | 0](4715) | 0;
  return;
 }
 
 function $286($0_1) {
  $0_1 = $0_1 | 0;
  var $3_1 = 0, $4_1 = 0;
  $3_1 = global$0 - 16 | 0;
  global$0 = $3_1;
  HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
  $4_1 = HEAP32[($3_1 + 12 | 0) >> 2] | 0;
  $199();
  global$0 = $3_1 + 16 | 0;
  return $4_1 | 0;
 }
 
 function $287() {
  return 4256 | 0;
 }
 
 function $288() {
  var $1_1 = 0;
  $1_1 = 24;
  return (128 << $1_1 | 0) >> $1_1 | 0 | 0;
 }
 
 function $289() {
  var $1_1 = 0;
  $1_1 = 24;
  return (127 << $1_1 | 0) >> $1_1 | 0 | 0;
 }
 
 function $290() {
  return 4280 | 0;
 }
 
 function $291() {
  var $1_1 = 0;
  $1_1 = 24;
  return (128 << $1_1 | 0) >> $1_1 | 0 | 0;
 }
 
 function $292() {
  var $1_1 = 0;
  $1_1 = 24;
  return (127 << $1_1 | 0) >> $1_1 | 0 | 0;
 }
 
 function $293() {
  return 4268 | 0;
 }
 
 function $294() {
  return 0 & 255 | 0 | 0;
 }
 
 function $295() {
  return 255 & 255 | 0 | 0;
 }
 
 function $296() {
  return 4292 | 0;
 }
 
 function $297() {
  var $1_1 = 0;
  $1_1 = 16;
  return (32768 << $1_1 | 0) >> $1_1 | 0 | 0;
 }
 
 function $298() {
  var $1_1 = 0;
  $1_1 = 16;
  return (32767 << $1_1 | 0) >> $1_1 | 0 | 0;
 }
 
 function $299() {
  return 4304 | 0;
 }
 
 function $300() {
  return 0 & 65535 | 0 | 0;
 }
 
 function $301() {
  return 65535 & 65535 | 0 | 0;
 }
 
 function $302() {
  return 4316 | 0;
 }
 
 function $303() {
  return -2147483648 | 0;
 }
 
 function $304() {
  return 2147483647 | 0;
 }
 
 function $305() {
  return 4328 | 0;
 }
 
 function $306() {
  return 0 | 0;
 }
 
 function $307() {
  return -1 | 0;
 }
 
 function $308() {
  return 4340 | 0;
 }
 
 function $309() {
  return -2147483648 | 0;
 }
 
 function $310() {
  return 2147483647 | 0;
 }
 
 function $311() {
  return 4352 | 0;
 }
 
 function $312() {
  return 0 | 0;
 }
 
 function $313() {
  return -1 | 0;
 }
 
 function $314() {
  return 3344 | 0;
 }
 
 function $315() {
  return 3384 | 0;
 }
 
 function $316() {
  return 3424 | 0;
 }
 
 function $317() {
  return 3464 | 0;
 }
 
 function $318() {
  return 3504 | 0;
 }
 
 function $319() {
  return 3544 | 0;
 }
 
 function $320() {
  return 3584 | 0;
 }
 
 function $321() {
  return 3624 | 0;
 }
 
 function $322() {
  return 3664 | 0;
 }
 
 function $323() {
  return 3704 | 0;
 }
 
 function $324() {
  return 3744 | 0;
 }
 
 function $325() {
  $285();
  return;
 }
 
 function $326($0_1) {
  $0_1 = $0_1 | 0;
  var $1_1 = 0, $2_1 = 0;
  label$1 : {
   $1_1 = ($377($0_1 | 0) | 0) + 1 | 0;
   $2_1 = $371($1_1 | 0) | 0;
   if ($2_1) {
    break label$1
   }
   return 0 | 0;
  }
  return $375($2_1 | 0, $0_1 | 0, $1_1 | 0) | 0 | 0;
 }
 
 function $327($0_1) {
  $0_1 = +$0_1;
  return +Math_sqrt($0_1);
 }
 
 function $328($0_1) {
  $0_1 = +$0_1;
  return +Math_abs($0_1);
 }
 
 function $329($0_1, $1_1) {
  $0_1 = +$0_1;
  $1_1 = +$1_1;
  var i64toi32_i32$1 = 0, i64toi32_i32$0 = 0, i64toi32_i32$4 = 0, i64toi32_i32$2 = 0, i64toi32_i32$3 = 0, $13_1 = 0.0, $3_1 = 0, $4_1 = 0, $5_1 = 0, $2_1 = 0, $15_1 = 0.0, $16_1 = 0.0, $7_1 = 0, $11_1 = 0, $14_1 = 0.0, $8_1 = 0, $9_1 = 0, $11$hi = 0, $17_1 = 0.0, $18_1 = 0.0, $6_1 = 0, $46_1 = 0, $12_1 = 0, $47_1 = 0, $10_1 = 0, $48_1 = 0, $49_1 = 0, $19_1 = 0.0, $20_1 = 0.0, $50_1 = 0, $21_1 = 0.0, $51_1 = 0, $52_1 = 0, $53_1 = 0, $54_1 = 0, $12$hi = 0, $275_1 = 0.0, $278$hi = 0, $281_1 = 0, $281$hi = 0, $301_1 = 0.0, $306_1 = 0.0, $309_1 = 0.0, $310_1 = 0.0, $311_1 = 0.0, $312_1 = 0.0, $510 = 0.0, $511 = 0.0, $525 = 0, $568 = 0.0, $595$hi = 0, $597 = 0, $597$hi = 0;
  $13_1 = 1.0;
  label$1 : {
   wasm2js_scratch_store_f64(+$1_1);
   i64toi32_i32$0 = wasm2js_scratch_load_i32(1 | 0) | 0;
   $11_1 = wasm2js_scratch_load_i32(0 | 0) | 0;
   $11$hi = i64toi32_i32$0;
   i64toi32_i32$2 = $11_1;
   i64toi32_i32$1 = 0;
   i64toi32_i32$3 = 32;
   i64toi32_i32$4 = i64toi32_i32$3 & 31 | 0;
   if (32 >>> 0 <= (i64toi32_i32$3 & 63 | 0) >>> 0) {
    i64toi32_i32$1 = 0;
    $46_1 = i64toi32_i32$0 >>> i64toi32_i32$4 | 0;
   } else {
    i64toi32_i32$1 = i64toi32_i32$0 >>> i64toi32_i32$4 | 0;
    $46_1 = (((1 << i64toi32_i32$4 | 0) - 1 | 0) & i64toi32_i32$0 | 0) << (32 - i64toi32_i32$4 | 0) | 0 | (i64toi32_i32$2 >>> i64toi32_i32$4 | 0) | 0;
   }
   $2_1 = $46_1;
   $3_1 = $2_1 & 2147483647 | 0;
   i64toi32_i32$1 = $11$hi;
   $4_1 = $11_1;
   if (!($3_1 | $4_1 | 0)) {
    break label$1
   }
   wasm2js_scratch_store_f64(+$0_1);
   i64toi32_i32$1 = wasm2js_scratch_load_i32(1 | 0) | 0;
   $12_1 = wasm2js_scratch_load_i32(0 | 0) | 0;
   $12$hi = i64toi32_i32$1;
   i64toi32_i32$0 = $12_1;
   i64toi32_i32$2 = 0;
   i64toi32_i32$3 = 32;
   i64toi32_i32$4 = i64toi32_i32$3 & 31 | 0;
   if (32 >>> 0 <= (i64toi32_i32$3 & 63 | 0) >>> 0) {
    i64toi32_i32$2 = 0;
    $47_1 = i64toi32_i32$1 >>> i64toi32_i32$4 | 0;
   } else {
    i64toi32_i32$2 = i64toi32_i32$1 >>> i64toi32_i32$4 | 0;
    $47_1 = (((1 << i64toi32_i32$4 | 0) - 1 | 0) & i64toi32_i32$1 | 0) << (32 - i64toi32_i32$4 | 0) | 0 | (i64toi32_i32$0 >>> i64toi32_i32$4 | 0) | 0;
   }
   $5_1 = $47_1;
   label$2 : {
    i64toi32_i32$2 = $12$hi;
    $6_1 = $12_1;
    if ($6_1) {
     break label$2
    }
    if (($5_1 | 0) == (1072693248 | 0)) {
     break label$1
    }
   }
   label$3 : {
    label$4 : {
     $7_1 = $5_1 & 2147483647 | 0;
     if ($7_1 >>> 0 > 2146435072 >>> 0) {
      break label$4
     }
     if (($6_1 | 0) != (0 | 0) & ($7_1 | 0) == (2146435072 | 0) | 0) {
      break label$4
     }
     if ($3_1 >>> 0 > 2146435072 >>> 0) {
      break label$4
     }
     if (!$4_1) {
      break label$3
     }
     if (($3_1 | 0) != (2146435072 | 0)) {
      break label$3
     }
    }
    return +($0_1 + $1_1);
   }
   label$5 : {
    label$6 : {
     label$7 : {
      label$8 : {
       if (($5_1 | 0) > (-1 | 0)) {
        break label$8
       }
       $8_1 = 2;
       if ($3_1 >>> 0 > 1128267775 >>> 0) {
        break label$7
       }
       if ($3_1 >>> 0 < 1072693248 >>> 0) {
        break label$8
       }
       $9_1 = $3_1 >>> 20 | 0;
       label$9 : {
        if ($3_1 >>> 0 < 1094713344 >>> 0) {
         break label$9
        }
        $8_1 = 0;
        $9_1 = 1075 - $9_1 | 0;
        $10_1 = $4_1 >>> $9_1 | 0;
        if (($10_1 << $9_1 | 0 | 0) != ($4_1 | 0)) {
         break label$7
        }
        $8_1 = 2 - ($10_1 & 1 | 0) | 0;
        break label$7;
       }
       $8_1 = 0;
       if ($4_1) {
        break label$5
       }
       $8_1 = 0;
       $4_1 = 1043 - $9_1 | 0;
       $9_1 = $3_1 >>> $4_1 | 0;
       if (($9_1 << $4_1 | 0 | 0) != ($3_1 | 0)) {
        break label$6
       }
       $8_1 = 2 - ($9_1 & 1 | 0) | 0;
       break label$6;
      }
      $8_1 = 0;
     }
     if ($4_1) {
      break label$5
     }
    }
    label$10 : {
     if (($3_1 | 0) != (2146435072 | 0)) {
      break label$10
     }
     if (!($7_1 + -1072693248 | 0 | $6_1 | 0)) {
      break label$1
     }
     label$11 : {
      if ($7_1 >>> 0 < 1072693248 >>> 0) {
       break label$11
      }
      return +(($2_1 | 0) > (-1 | 0) ? $1_1 : 0.0);
     }
     return +(($2_1 | 0) > (-1 | 0) ? 0.0 : -$1_1);
    }
    label$12 : {
     if (($3_1 | 0) != (1072693248 | 0)) {
      break label$12
     }
     label$13 : {
      if (($2_1 | 0) <= (-1 | 0)) {
       break label$13
      }
      return +$0_1;
     }
     return +(1.0 / $0_1);
    }
    label$14 : {
     if (($2_1 | 0) != (1073741824 | 0)) {
      break label$14
     }
     return +($0_1 * $0_1);
    }
    if (($5_1 | 0) < (0 | 0)) {
     break label$5
    }
    if (($2_1 | 0) != (1071644672 | 0)) {
     break label$5
    }
    return +(+$327(+$0_1));
   }
   $13_1 = +$328(+$0_1);
   label$15 : {
    if ($6_1) {
     break label$15
    }
    label$16 : {
     if (($5_1 & 1073741823 | 0 | 0) == (1072693248 | 0)) {
      break label$16
     }
     if ($7_1) {
      break label$15
     }
    }
    $13_1 = ($2_1 | 0) < (0 | 0) ? 1.0 / $13_1 : $13_1;
    if (($5_1 | 0) > (-1 | 0)) {
     break label$1
    }
    label$17 : {
     if ($8_1 | ($7_1 + -1072693248 | 0) | 0) {
      break label$17
     }
     $1_1 = $13_1 - $13_1;
     return +($1_1 / $1_1);
    }
    return +(($8_1 | 0) == (1 | 0) ? -$13_1 : $13_1);
   }
   $14_1 = 1.0;
   label$18 : {
    if (($5_1 | 0) > (-1 | 0)) {
     break label$18
    }
    label$19 : {
     switch ($8_1 | 0) {
     case 0:
      $1_1 = $0_1 - $0_1;
      return +($1_1 / $1_1);
     case 1:
      break label$19;
     default:
      break label$18;
     };
    }
    $14_1 = -1.0;
   }
   label$21 : {
    label$22 : {
     if ($3_1 >>> 0 < 1105199105 >>> 0) {
      break label$22
     }
     label$23 : {
      if ($3_1 >>> 0 < 1139802113 >>> 0) {
       break label$23
      }
      label$24 : {
       if ($7_1 >>> 0 > 1072693247 >>> 0) {
        break label$24
       }
       return +(($2_1 | 0) < (0 | 0) ? infinity : 0.0);
      }
      return +(($2_1 | 0) > (0 | 0) ? infinity : 0.0);
     }
     label$25 : {
      if ($7_1 >>> 0 > 1072693246 >>> 0) {
       break label$25
      }
      return +(($2_1 | 0) < (0 | 0) ? $14_1 * 1.e+300 * 1.e+300 : $14_1 * 1.0e-300 * 1.0e-300);
     }
     label$26 : {
      if ($7_1 >>> 0 < 1072693249 >>> 0) {
       break label$26
      }
      return +(($2_1 | 0) > (0 | 0) ? $14_1 * 1.e+300 * 1.e+300 : $14_1 * 1.0e-300 * 1.0e-300);
     }
     $0_1 = $13_1 + -1.0;
     $13_1 = $0_1 * 1.4426950216293335;
     $15_1 = $0_1 * 1.9259629911266175e-08 + $0_1 * $0_1 * (.5 - $0_1 * ($0_1 * -.25 + .3333333333333333)) * -1.4426950408889634;
     wasm2js_scratch_store_f64(+($13_1 + $15_1));
     i64toi32_i32$2 = wasm2js_scratch_load_i32(1 | 0) | 0;
     i64toi32_i32$1 = wasm2js_scratch_load_i32(0 | 0) | 0;
     i64toi32_i32$0 = -1;
     i64toi32_i32$3 = 0;
     i64toi32_i32$0 = i64toi32_i32$2 & i64toi32_i32$0 | 0;
     wasm2js_scratch_store_i32(0 | 0, i64toi32_i32$1 & i64toi32_i32$3 | 0 | 0);
     wasm2js_scratch_store_i32(1 | 0, i64toi32_i32$0 | 0);
     $0_1 = +wasm2js_scratch_load_f64();
     $16_1 = $0_1 - $13_1;
     break label$21;
    }
    $0_1 = $13_1 * 9007199254740992.0;
    $3_1 = $7_1 >>> 0 < 1048576 >>> 0;
    $13_1 = $3_1 ? $0_1 : $13_1;
    wasm2js_scratch_store_f64(+$0_1);
    i64toi32_i32$0 = wasm2js_scratch_load_i32(1 | 0) | 0;
    i64toi32_i32$2 = wasm2js_scratch_load_i32(0 | 0) | 0;
    i64toi32_i32$1 = 0;
    i64toi32_i32$3 = 32;
    i64toi32_i32$4 = i64toi32_i32$3 & 31 | 0;
    if (32 >>> 0 <= (i64toi32_i32$3 & 63 | 0) >>> 0) {
     i64toi32_i32$1 = 0;
     $48_1 = i64toi32_i32$0 >>> i64toi32_i32$4 | 0;
    } else {
     i64toi32_i32$1 = i64toi32_i32$0 >>> i64toi32_i32$4 | 0;
     $48_1 = (((1 << i64toi32_i32$4 | 0) - 1 | 0) & i64toi32_i32$0 | 0) << (32 - i64toi32_i32$4 | 0) | 0 | (i64toi32_i32$2 >>> i64toi32_i32$4 | 0) | 0;
    }
    $2_1 = $3_1 ? $48_1 : $7_1;
    $4_1 = $2_1 & 1048575 | 0;
    $5_1 = $4_1 | 1072693248 | 0;
    $2_1 = ($3_1 ? -1076 : -1023) + ($2_1 >> 20 | 0) | 0;
    $3_1 = 0;
    label$27 : {
     if ($4_1 >>> 0 < 235663 >>> 0) {
      break label$27
     }
     label$28 : {
      if ($4_1 >>> 0 >= 767610 >>> 0) {
       break label$28
      }
      $3_1 = 1;
      break label$27;
     }
     $5_1 = $5_1 + -1048576 | 0;
     $2_1 = $2_1 + 1 | 0;
    }
    $4_1 = $3_1 << 3 | 0;
    $17_1 = +HEAPF64[($4_1 + 3792 | 0) >> 3];
    $275_1 = $17_1;
    i64toi32_i32$1 = 0;
    i64toi32_i32$0 = $5_1;
    i64toi32_i32$2 = 0;
    i64toi32_i32$3 = 32;
    i64toi32_i32$4 = i64toi32_i32$3 & 31 | 0;
    if (32 >>> 0 <= (i64toi32_i32$3 & 63 | 0) >>> 0) {
     i64toi32_i32$2 = i64toi32_i32$0 << i64toi32_i32$4 | 0;
     $49_1 = 0;
    } else {
     i64toi32_i32$2 = ((1 << i64toi32_i32$4 | 0) - 1 | 0) & (i64toi32_i32$0 >>> (32 - i64toi32_i32$4 | 0) | 0) | 0 | (i64toi32_i32$1 << i64toi32_i32$4 | 0) | 0;
     $49_1 = i64toi32_i32$0 << i64toi32_i32$4 | 0;
    }
    $278$hi = i64toi32_i32$2;
    wasm2js_scratch_store_f64(+$13_1);
    i64toi32_i32$2 = wasm2js_scratch_load_i32(1 | 0) | 0;
    i64toi32_i32$1 = wasm2js_scratch_load_i32(0 | 0) | 0;
    i64toi32_i32$0 = 0;
    i64toi32_i32$3 = -1;
    i64toi32_i32$0 = i64toi32_i32$2 & i64toi32_i32$0 | 0;
    $281_1 = i64toi32_i32$1 & i64toi32_i32$3 | 0;
    $281$hi = i64toi32_i32$0;
    i64toi32_i32$0 = $278$hi;
    i64toi32_i32$2 = $49_1;
    i64toi32_i32$1 = $281$hi;
    i64toi32_i32$3 = $281_1;
    i64toi32_i32$1 = i64toi32_i32$0 | i64toi32_i32$1 | 0;
    wasm2js_scratch_store_i32(0 | 0, i64toi32_i32$2 | i64toi32_i32$3 | 0 | 0);
    wasm2js_scratch_store_i32(1 | 0, i64toi32_i32$1 | 0);
    $15_1 = +wasm2js_scratch_load_f64();
    $16_1 = +HEAPF64[($4_1 + 3760 | 0) >> 3];
    $18_1 = $15_1 - $16_1;
    $19_1 = 1.0 / ($16_1 + $15_1);
    $13_1 = $18_1 * $19_1;
    wasm2js_scratch_store_f64(+$13_1);
    i64toi32_i32$1 = wasm2js_scratch_load_i32(1 | 0) | 0;
    i64toi32_i32$0 = wasm2js_scratch_load_i32(0 | 0) | 0;
    i64toi32_i32$2 = -1;
    i64toi32_i32$3 = 0;
    i64toi32_i32$2 = i64toi32_i32$1 & i64toi32_i32$2 | 0;
    wasm2js_scratch_store_i32(0 | 0, i64toi32_i32$0 & i64toi32_i32$3 | 0 | 0);
    wasm2js_scratch_store_i32(1 | 0, i64toi32_i32$2 | 0);
    $0_1 = +wasm2js_scratch_load_f64();
    $301_1 = $0_1;
    $20_1 = $0_1 * $0_1;
    $306_1 = $20_1 + 3.0;
    $309_1 = $13_1 + $0_1;
    $310_1 = $19_1;
    $311_1 = $18_1;
    $312_1 = $0_1;
    i64toi32_i32$2 = 0;
    i64toi32_i32$1 = (($5_1 >> 1 | 0 | 536870912 | 0) + ($3_1 << 18 | 0) | 0) + 524288 | 0;
    i64toi32_i32$0 = 0;
    i64toi32_i32$3 = 32;
    i64toi32_i32$4 = i64toi32_i32$3 & 31 | 0;
    if (32 >>> 0 <= (i64toi32_i32$3 & 63 | 0) >>> 0) {
     i64toi32_i32$0 = i64toi32_i32$1 << i64toi32_i32$4 | 0;
     $50_1 = 0;
    } else {
     i64toi32_i32$0 = ((1 << i64toi32_i32$4 | 0) - 1 | 0) & (i64toi32_i32$1 >>> (32 - i64toi32_i32$4 | 0) | 0) | 0 | (i64toi32_i32$2 << i64toi32_i32$4 | 0) | 0;
     $50_1 = i64toi32_i32$1 << i64toi32_i32$4 | 0;
    }
    wasm2js_scratch_store_i32(0 | 0, $50_1 | 0);
    wasm2js_scratch_store_i32(1 | 0, i64toi32_i32$0 | 0);
    $21_1 = +wasm2js_scratch_load_f64();
    $15_1 = $310_1 * ($311_1 - $312_1 * $21_1 - $0_1 * ($15_1 - ($21_1 - $16_1)));
    $0_1 = $13_1 * $13_1;
    $16_1 = $309_1 * $15_1 + $0_1 * $0_1 * ($0_1 * ($0_1 * ($0_1 * ($0_1 * ($0_1 * .20697501780033842 + .23066074577556175) + .272728123808534) + .33333332981837743) + .4285714285785502) + .5999999999999946);
    wasm2js_scratch_store_f64(+($306_1 + $16_1));
    i64toi32_i32$0 = wasm2js_scratch_load_i32(1 | 0) | 0;
    i64toi32_i32$2 = wasm2js_scratch_load_i32(0 | 0) | 0;
    i64toi32_i32$1 = -1;
    i64toi32_i32$3 = 0;
    i64toi32_i32$1 = i64toi32_i32$0 & i64toi32_i32$1 | 0;
    wasm2js_scratch_store_i32(0 | 0, i64toi32_i32$2 & i64toi32_i32$3 | 0 | 0);
    wasm2js_scratch_store_i32(1 | 0, i64toi32_i32$1 | 0);
    $0_1 = +wasm2js_scratch_load_f64();
    $18_1 = $301_1 * $0_1;
    $13_1 = $15_1 * $0_1 + $13_1 * ($16_1 - ($0_1 + -3.0 - $20_1));
    wasm2js_scratch_store_f64(+($18_1 + $13_1));
    i64toi32_i32$1 = wasm2js_scratch_load_i32(1 | 0) | 0;
    i64toi32_i32$0 = wasm2js_scratch_load_i32(0 | 0) | 0;
    i64toi32_i32$2 = -1;
    i64toi32_i32$3 = 0;
    i64toi32_i32$2 = i64toi32_i32$1 & i64toi32_i32$2 | 0;
    wasm2js_scratch_store_i32(0 | 0, i64toi32_i32$0 & i64toi32_i32$3 | 0 | 0);
    wasm2js_scratch_store_i32(1 | 0, i64toi32_i32$2 | 0);
    $0_1 = +wasm2js_scratch_load_f64();
    $16_1 = $0_1 * .9617967009544373;
    $15_1 = +HEAPF64[($4_1 + 3776 | 0) >> 3] + (($13_1 - ($0_1 - $18_1)) * .9617966939259756 + $0_1 * -7.028461650952758e-09);
    $13_1 = +($2_1 | 0);
    wasm2js_scratch_store_f64(+($275_1 + ($16_1 + $15_1) + $13_1));
    i64toi32_i32$2 = wasm2js_scratch_load_i32(1 | 0) | 0;
    i64toi32_i32$1 = wasm2js_scratch_load_i32(0 | 0) | 0;
    i64toi32_i32$0 = -1;
    i64toi32_i32$3 = 0;
    i64toi32_i32$0 = i64toi32_i32$2 & i64toi32_i32$0 | 0;
    wasm2js_scratch_store_i32(0 | 0, i64toi32_i32$1 & i64toi32_i32$3 | 0 | 0);
    wasm2js_scratch_store_i32(1 | 0, i64toi32_i32$0 | 0);
    $0_1 = +wasm2js_scratch_load_f64();
    $16_1 = $0_1 - $13_1 - $17_1 - $16_1;
   }
   i64toi32_i32$0 = $11$hi;
   i64toi32_i32$2 = $11_1;
   i64toi32_i32$1 = -1;
   i64toi32_i32$3 = 0;
   i64toi32_i32$1 = i64toi32_i32$0 & i64toi32_i32$1 | 0;
   wasm2js_scratch_store_i32(0 | 0, i64toi32_i32$2 & i64toi32_i32$3 | 0 | 0);
   wasm2js_scratch_store_i32(1 | 0, i64toi32_i32$1 | 0);
   $17_1 = +wasm2js_scratch_load_f64();
   $13_1 = $0_1 * $17_1;
   $1_1 = ($15_1 - $16_1) * $1_1 + ($1_1 - $17_1) * $0_1;
   $0_1 = $13_1 + $1_1;
   wasm2js_scratch_store_f64(+$0_1);
   i64toi32_i32$1 = wasm2js_scratch_load_i32(1 | 0) | 0;
   $11_1 = wasm2js_scratch_load_i32(0 | 0) | 0;
   $11$hi = i64toi32_i32$1;
   $3_1 = $11_1;
   label$29 : {
    label$30 : {
     i64toi32_i32$0 = $3_1;
     i64toi32_i32$2 = 0;
     i64toi32_i32$3 = 32;
     i64toi32_i32$4 = i64toi32_i32$3 & 31 | 0;
     if (32 >>> 0 <= (i64toi32_i32$3 & 63 | 0) >>> 0) {
      i64toi32_i32$2 = 0;
      $51_1 = i64toi32_i32$1 >>> i64toi32_i32$4 | 0;
     } else {
      i64toi32_i32$2 = i64toi32_i32$1 >>> i64toi32_i32$4 | 0;
      $51_1 = (((1 << i64toi32_i32$4 | 0) - 1 | 0) & i64toi32_i32$1 | 0) << (32 - i64toi32_i32$4 | 0) | 0 | (i64toi32_i32$0 >>> i64toi32_i32$4 | 0) | 0;
     }
     $5_1 = $51_1;
     if (($5_1 | 0) < (1083179008 | 0)) {
      break label$30
     }
     label$31 : {
      if (!($5_1 + -1083179008 | 0 | $3_1 | 0)) {
       break label$31
      }
      return +($14_1 * 1.e+300 * 1.e+300);
     }
     if ($1_1 + 8.008566259537294e-17 > $0_1 - $13_1 ^ 1 | 0) {
      break label$29
     }
     return +($14_1 * 1.e+300 * 1.e+300);
    }
    if (($5_1 & 2147482624 | 0) >>> 0 < 1083231232 >>> 0) {
     break label$29
    }
    label$32 : {
     if (!($5_1 + 1064252416 | 0 | $3_1 | 0)) {
      break label$32
     }
     return +($14_1 * 1.0e-300 * 1.0e-300);
    }
    if ($1_1 <= $0_1 - $13_1 ^ 1 | 0) {
     break label$29
    }
    return +($14_1 * 1.0e-300 * 1.0e-300);
   }
   $3_1 = 0;
   label$33 : {
    $4_1 = $5_1 & 2147483647 | 0;
    if ($4_1 >>> 0 < 1071644673 >>> 0) {
     break label$33
    }
    $4_1 = (1048576 >>> (($4_1 >>> 20 | 0) + -1022 | 0) | 0) + $5_1 | 0;
    $2_1 = ($4_1 >>> 20 | 0) & 2047 | 0;
    $3_1 = ($4_1 & 1048575 | 0 | 1048576 | 0) >>> (1043 - $2_1 | 0) | 0;
    $3_1 = ($5_1 | 0) < (0 | 0) ? 0 - $3_1 | 0 : $3_1;
    $510 = $1_1;
    $511 = $13_1;
    i64toi32_i32$2 = 0;
    i64toi32_i32$1 = (-1048576 >> ($2_1 + -1023 | 0) | 0) & $4_1 | 0;
    i64toi32_i32$0 = 0;
    i64toi32_i32$3 = 32;
    i64toi32_i32$4 = i64toi32_i32$3 & 31 | 0;
    if (32 >>> 0 <= (i64toi32_i32$3 & 63 | 0) >>> 0) {
     i64toi32_i32$0 = i64toi32_i32$1 << i64toi32_i32$4 | 0;
     $52_1 = 0;
    } else {
     i64toi32_i32$0 = ((1 << i64toi32_i32$4 | 0) - 1 | 0) & (i64toi32_i32$1 >>> (32 - i64toi32_i32$4 | 0) | 0) | 0 | (i64toi32_i32$2 << i64toi32_i32$4 | 0) | 0;
     $52_1 = i64toi32_i32$1 << i64toi32_i32$4 | 0;
    }
    wasm2js_scratch_store_i32(0 | 0, $52_1 | 0);
    wasm2js_scratch_store_i32(1 | 0, i64toi32_i32$0 | 0);
    $13_1 = $511 - +wasm2js_scratch_load_f64();
    wasm2js_scratch_store_f64(+($510 + $13_1));
    i64toi32_i32$0 = wasm2js_scratch_load_i32(1 | 0) | 0;
    $11_1 = wasm2js_scratch_load_i32(0 | 0) | 0;
    $11$hi = i64toi32_i32$0;
   }
   label$34 : {
    label$35 : {
     $525 = $3_1 << 20 | 0;
     i64toi32_i32$0 = $11$hi;
     i64toi32_i32$2 = $11_1;
     i64toi32_i32$1 = -1;
     i64toi32_i32$3 = 0;
     i64toi32_i32$1 = i64toi32_i32$0 & i64toi32_i32$1 | 0;
     wasm2js_scratch_store_i32(0 | 0, i64toi32_i32$2 & i64toi32_i32$3 | 0 | 0);
     wasm2js_scratch_store_i32(1 | 0, i64toi32_i32$1 | 0);
     $0_1 = +wasm2js_scratch_load_f64();
     $15_1 = $0_1 * .6931471824645996;
     $13_1 = ($1_1 - ($0_1 - $13_1)) * .6931471805599453 + $0_1 * -1.904654299957768e-09;
     $1_1 = $15_1 + $13_1;
     $0_1 = $1_1 * $1_1;
     $0_1 = $1_1 - $0_1 * ($0_1 * ($0_1 * ($0_1 * ($0_1 * 4.1381367970572385e-08 + -1.6533902205465252e-06) + 6.613756321437934e-05) + -2.7777777777015593e-03) + .16666666666666602);
     $568 = $1_1 * $0_1 / ($0_1 + -2.0);
     $0_1 = $13_1 - ($1_1 - $15_1);
     $1_1 = $1_1 - ($568 - ($0_1 + $1_1 * $0_1)) + 1.0;
     wasm2js_scratch_store_f64(+$1_1);
     i64toi32_i32$1 = wasm2js_scratch_load_i32(1 | 0) | 0;
     $11_1 = wasm2js_scratch_load_i32(0 | 0) | 0;
     $11$hi = i64toi32_i32$1;
     i64toi32_i32$0 = $11_1;
     i64toi32_i32$2 = 0;
     i64toi32_i32$3 = 32;
     i64toi32_i32$4 = i64toi32_i32$3 & 31 | 0;
     if (32 >>> 0 <= (i64toi32_i32$3 & 63 | 0) >>> 0) {
      i64toi32_i32$2 = 0;
      $53_1 = i64toi32_i32$1 >>> i64toi32_i32$4 | 0;
     } else {
      i64toi32_i32$2 = i64toi32_i32$1 >>> i64toi32_i32$4 | 0;
      $53_1 = (((1 << i64toi32_i32$4 | 0) - 1 | 0) & i64toi32_i32$1 | 0) << (32 - i64toi32_i32$4 | 0) | 0 | (i64toi32_i32$0 >>> i64toi32_i32$4 | 0) | 0;
     }
     $5_1 = $525 + $53_1 | 0;
     if (($5_1 | 0) > (1048575 | 0)) {
      break label$35
     }
     $1_1 = +$374(+$1_1, $3_1 | 0);
     break label$34;
    }
    i64toi32_i32$2 = 0;
    i64toi32_i32$1 = $5_1;
    i64toi32_i32$0 = 0;
    i64toi32_i32$3 = 32;
    i64toi32_i32$4 = i64toi32_i32$3 & 31 | 0;
    if (32 >>> 0 <= (i64toi32_i32$3 & 63 | 0) >>> 0) {
     i64toi32_i32$0 = i64toi32_i32$1 << i64toi32_i32$4 | 0;
     $54_1 = 0;
    } else {
     i64toi32_i32$0 = ((1 << i64toi32_i32$4 | 0) - 1 | 0) & (i64toi32_i32$1 >>> (32 - i64toi32_i32$4 | 0) | 0) | 0 | (i64toi32_i32$2 << i64toi32_i32$4 | 0) | 0;
     $54_1 = i64toi32_i32$1 << i64toi32_i32$4 | 0;
    }
    $595$hi = i64toi32_i32$0;
    i64toi32_i32$0 = $11$hi;
    i64toi32_i32$2 = $11_1;
    i64toi32_i32$1 = 0;
    i64toi32_i32$3 = -1;
    i64toi32_i32$1 = i64toi32_i32$0 & i64toi32_i32$1 | 0;
    $597 = i64toi32_i32$2 & i64toi32_i32$3 | 0;
    $597$hi = i64toi32_i32$1;
    i64toi32_i32$1 = $595$hi;
    i64toi32_i32$0 = $54_1;
    i64toi32_i32$2 = $597$hi;
    i64toi32_i32$3 = $597;
    i64toi32_i32$2 = i64toi32_i32$1 | i64toi32_i32$2 | 0;
    wasm2js_scratch_store_i32(0 | 0, i64toi32_i32$0 | i64toi32_i32$3 | 0 | 0);
    wasm2js_scratch_store_i32(1 | 0, i64toi32_i32$2 | 0);
    $1_1 = +wasm2js_scratch_load_f64();
   }
   $13_1 = $14_1 * $1_1;
  }
  return +$13_1;
 }
 
 function $330($0_1) {
  $0_1 = $0_1 | 0;
  var $1_1 = 0;
  $1_1 = $0_1 ? $0_1 : 1;
  label$1 : {
   label$2 : while (1) {
    $0_1 = $371($1_1 | 0) | 0;
    if ($0_1) {
     break label$1
    }
    label$3 : {
     $0_1 = $333() | 0;
     if (!$0_1) {
      break label$3
     }
     FUNCTION_TABLE[$0_1 | 0]();
     continue label$2;
    }
    break label$2;
   };
   fimport$16();
   abort();
  }
  return $0_1 | 0;
 }
 
 function $331($0_1) {
  $0_1 = $0_1 | 0;
  $372($0_1 | 0);
 }
 
 function $332($0_1) {
  $0_1 = $0_1 | 0;
  return HEAP32[$0_1 >> 2] | 0 | 0;
 }
 
 function $333() {
  return $332(4716 | 0) | 0 | 0;
 }
 
 function $334($0_1) {
  $0_1 = $0_1 | 0;
  return $0_1 | 0;
 }
 
 function $335($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  var $3_1 = 0, $2_1 = 0;
  $2_1 = HEAPU8[$1_1 >> 0] | 0;
  label$1 : {
   $3_1 = HEAPU8[$0_1 >> 0] | 0;
   if (!$3_1) {
    break label$1
   }
   if (($3_1 | 0) != ($2_1 & 255 | 0 | 0)) {
    break label$1
   }
   label$2 : while (1) {
    $2_1 = HEAPU8[($1_1 + 1 | 0) >> 0] | 0;
    $3_1 = HEAPU8[($0_1 + 1 | 0) >> 0] | 0;
    if (!$3_1) {
     break label$1
    }
    $1_1 = $1_1 + 1 | 0;
    $0_1 = $0_1 + 1 | 0;
    if (($3_1 | 0) == ($2_1 & 255 | 0 | 0)) {
     continue label$2
    }
    break label$2;
   };
  }
  return $3_1 - ($2_1 & 255 | 0) | 0 | 0;
 }
 
 function $336($0_1) {
  $0_1 = $0_1 | 0;
  $334($0_1 | 0) | 0;
  return $0_1 | 0;
 }
 
 function $337($0_1) {
  $0_1 = $0_1 | 0;
 }
 
 function $338($0_1) {
  $0_1 = $0_1 | 0;
 }
 
 function $339($0_1) {
  $0_1 = $0_1 | 0;
  $336($0_1 | 0) | 0;
  $331($0_1 | 0);
 }
 
 function $340($0_1) {
  $0_1 = $0_1 | 0;
  $336($0_1 | 0) | 0;
  $331($0_1 | 0);
 }
 
 function $341($0_1) {
  $0_1 = $0_1 | 0;
  $336($0_1 | 0) | 0;
  $331($0_1 | 0);
 }
 
 function $342($0_1) {
  $0_1 = $0_1 | 0;
  $336($0_1 | 0) | 0;
  $331($0_1 | 0);
 }
 
 function $343($0_1) {
  $0_1 = $0_1 | 0;
  $336($0_1 | 0) | 0;
  $331($0_1 | 0);
 }
 
 function $344($0_1) {
  $0_1 = $0_1 | 0;
  $336($0_1 | 0) | 0;
  $331($0_1 | 0);
 }
 
 function $345($0_1, $1_1, $2_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  return $346($0_1 | 0, $1_1 | 0, 0 | 0) | 0 | 0;
 }
 
 function $346($0_1, $1_1, $2_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  label$1 : {
   if ($2_1) {
    break label$1
   }
   return $347($0_1 | 0, $1_1 | 0) | 0 | 0;
  }
  label$2 : {
   if (($0_1 | 0) != ($1_1 | 0)) {
    break label$2
   }
   return 1 | 0;
  }
  return !($335($198($0_1 | 0) | 0 | 0, $198($1_1 | 0) | 0 | 0) | 0) | 0;
 }
 
 function $347($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  return (HEAP32[($0_1 + 4 | 0) >> 2] | 0 | 0) == (HEAP32[($1_1 + 4 | 0) >> 2] | 0 | 0) | 0;
 }
 
 function $348($0_1, $1_1, $2_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  return $346($0_1 | 0, $1_1 | 0, 0 | 0) | 0 | 0;
 }
 
 function $349($0_1, $1_1, $2_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  var $3_1 = 0, $4_1 = 0;
  $3_1 = global$0 - 64 | 0;
  global$0 = $3_1;
  $4_1 = 1;
  label$1 : {
   if ($346($0_1 | 0, $1_1 | 0, 0 | 0) | 0) {
    break label$1
   }
   $4_1 = 0;
   if (!$1_1) {
    break label$1
   }
   $4_1 = 0;
   $1_1 = $350($1_1 | 0, 3868 | 0, 3916 | 0, 0 | 0) | 0;
   if (!$1_1) {
    break label$1
   }
   HEAP32[($3_1 + 20 | 0) >> 2] = -1;
   HEAP32[($3_1 + 16 | 0) >> 2] = $0_1;
   HEAP32[($3_1 + 12 | 0) >> 2] = 0;
   HEAP32[($3_1 + 8 | 0) >> 2] = $1_1;
   $376($3_1 + 24 | 0 | 0, 0 | 0, 39 | 0) | 0;
   HEAP32[($3_1 + 56 | 0) >> 2] = 1;
   FUNCTION_TABLE[HEAP32[((HEAP32[$1_1 >> 2] | 0) + 28 | 0) >> 2] | 0 | 0]($1_1, $3_1 + 8 | 0, HEAP32[$2_1 >> 2] | 0, 1);
   label$2 : {
    $4_1 = HEAP32[($3_1 + 32 | 0) >> 2] | 0;
    if (($4_1 | 0) != (1 | 0)) {
     break label$2
    }
    HEAP32[$2_1 >> 2] = HEAP32[($3_1 + 24 | 0) >> 2] | 0;
   }
   $4_1 = ($4_1 | 0) == (1 | 0);
  }
  global$0 = $3_1 + 64 | 0;
  return $4_1 | 0;
 }
 
 function $350($0_1, $1_1, $2_1, $3_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  var $4_1 = 0, $6_1 = 0, $5_1 = 0, wasm2js_i32$0 = 0, wasm2js_i32$1 = 0, wasm2js_i32$2 = 0, wasm2js_i32$3 = 0, wasm2js_i32$4 = 0, wasm2js_i32$5 = 0, wasm2js_i32$6 = 0, wasm2js_i32$7 = 0, wasm2js_i32$8 = 0;
  $4_1 = global$0 - 64 | 0;
  global$0 = $4_1;
  $5_1 = HEAP32[$0_1 >> 2] | 0;
  $6_1 = HEAP32[($5_1 + -4 | 0) >> 2] | 0;
  $5_1 = HEAP32[($5_1 + -8 | 0) >> 2] | 0;
  HEAP32[($4_1 + 20 | 0) >> 2] = $3_1;
  HEAP32[($4_1 + 16 | 0) >> 2] = $1_1;
  HEAP32[($4_1 + 12 | 0) >> 2] = $0_1;
  HEAP32[($4_1 + 8 | 0) >> 2] = $2_1;
  $1_1 = 0;
  $376($4_1 + 24 | 0 | 0, 0 | 0, 39 | 0) | 0;
  $0_1 = $0_1 + $5_1 | 0;
  label$1 : {
   label$2 : {
    if (!($346($6_1 | 0, $2_1 | 0, 0 | 0) | 0)) {
     break label$2
    }
    HEAP32[($4_1 + 56 | 0) >> 2] = 1;
    FUNCTION_TABLE[HEAP32[((HEAP32[$6_1 >> 2] | 0) + 20 | 0) >> 2] | 0 | 0]($6_1, $4_1 + 8 | 0, $0_1, $0_1, 1, 0);
    $1_1 = (HEAP32[($4_1 + 32 | 0) >> 2] | 0 | 0) == (1 | 0) ? $0_1 : 0;
    break label$1;
   }
   FUNCTION_TABLE[HEAP32[((HEAP32[$6_1 >> 2] | 0) + 24 | 0) >> 2] | 0 | 0]($6_1, $4_1 + 8 | 0, $0_1, 1, 0);
   label$3 : {
    switch (HEAP32[($4_1 + 44 | 0) >> 2] | 0 | 0) {
    case 0:
     $1_1 = (wasm2js_i32$0 = (wasm2js_i32$3 = (wasm2js_i32$6 = HEAP32[($4_1 + 28 | 0) >> 2] | 0, wasm2js_i32$7 = 0, wasm2js_i32$8 = (HEAP32[($4_1 + 40 | 0) >> 2] | 0 | 0) == (1 | 0), wasm2js_i32$8 ? wasm2js_i32$6 : wasm2js_i32$7), wasm2js_i32$4 = 0, wasm2js_i32$5 = (HEAP32[($4_1 + 36 | 0) >> 2] | 0 | 0) == (1 | 0), wasm2js_i32$5 ? wasm2js_i32$3 : wasm2js_i32$4), wasm2js_i32$1 = 0, wasm2js_i32$2 = (HEAP32[($4_1 + 48 | 0) >> 2] | 0 | 0) == (1 | 0), wasm2js_i32$2 ? wasm2js_i32$0 : wasm2js_i32$1);
     break label$1;
    case 1:
     break label$3;
    default:
     break label$1;
    };
   }
   label$5 : {
    if ((HEAP32[($4_1 + 32 | 0) >> 2] | 0 | 0) == (1 | 0)) {
     break label$5
    }
    if (HEAP32[($4_1 + 48 | 0) >> 2] | 0) {
     break label$1
    }
    if ((HEAP32[($4_1 + 36 | 0) >> 2] | 0 | 0) != (1 | 0)) {
     break label$1
    }
    if ((HEAP32[($4_1 + 40 | 0) >> 2] | 0 | 0) != (1 | 0)) {
     break label$1
    }
   }
   $1_1 = HEAP32[($4_1 + 24 | 0) >> 2] | 0;
  }
  global$0 = $4_1 + 64 | 0;
  return $1_1 | 0;
 }
 
 function $351($0_1, $1_1, $2_1, $3_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  var $4_1 = 0;
  label$1 : {
   $4_1 = HEAP32[($1_1 + 16 | 0) >> 2] | 0;
   if ($4_1) {
    break label$1
   }
   HEAP32[($1_1 + 36 | 0) >> 2] = 1;
   HEAP32[($1_1 + 24 | 0) >> 2] = $3_1;
   HEAP32[($1_1 + 16 | 0) >> 2] = $2_1;
   return;
  }
  label$2 : {
   label$3 : {
    if (($4_1 | 0) != ($2_1 | 0)) {
     break label$3
    }
    if ((HEAP32[($1_1 + 24 | 0) >> 2] | 0 | 0) != (2 | 0)) {
     break label$2
    }
    HEAP32[($1_1 + 24 | 0) >> 2] = $3_1;
    return;
   }
   HEAP8[($1_1 + 54 | 0) >> 0] = 1;
   HEAP32[($1_1 + 24 | 0) >> 2] = 2;
   HEAP32[($1_1 + 36 | 0) >> 2] = (HEAP32[($1_1 + 36 | 0) >> 2] | 0) + 1 | 0;
  }
 }
 
 function $352($0_1, $1_1, $2_1, $3_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  label$1 : {
   if (!($346($0_1 | 0, HEAP32[($1_1 + 8 | 0) >> 2] | 0 | 0, 0 | 0) | 0)) {
    break label$1
   }
   $351($1_1 | 0, $1_1 | 0, $2_1 | 0, $3_1 | 0);
  }
 }
 
 function $353($0_1, $1_1, $2_1, $3_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  label$1 : {
   if (!($346($0_1 | 0, HEAP32[($1_1 + 8 | 0) >> 2] | 0 | 0, 0 | 0) | 0)) {
    break label$1
   }
   $351($1_1 | 0, $1_1 | 0, $2_1 | 0, $3_1 | 0);
   return;
  }
  $0_1 = HEAP32[($0_1 + 8 | 0) >> 2] | 0;
  FUNCTION_TABLE[HEAP32[((HEAP32[$0_1 >> 2] | 0) + 28 | 0) >> 2] | 0 | 0]($0_1, $1_1, $2_1, $3_1);
 }
 
 function $354($0_1, $1_1, $2_1, $3_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  var $5_1 = 0, $4_1 = 0;
  $4_1 = HEAP32[($0_1 + 4 | 0) >> 2] | 0;
  label$1 : {
   label$2 : {
    if ($2_1) {
     break label$2
    }
    $5_1 = 0;
    break label$1;
   }
   $5_1 = $4_1 >> 8 | 0;
   if (!($4_1 & 1 | 0)) {
    break label$1
   }
   $5_1 = HEAP32[((HEAP32[$2_1 >> 2] | 0) + $5_1 | 0) >> 2] | 0;
  }
  $0_1 = HEAP32[$0_1 >> 2] | 0;
  FUNCTION_TABLE[HEAP32[((HEAP32[$0_1 >> 2] | 0) + 28 | 0) >> 2] | 0 | 0]($0_1, $1_1, $2_1 + $5_1 | 0, $4_1 & 2 | 0 ? $3_1 : 2);
 }
 
 function $355($0_1, $1_1, $2_1, $3_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  var $4_1 = 0, $5_1 = 0;
  label$1 : {
   if (!($346($0_1 | 0, HEAP32[($1_1 + 8 | 0) >> 2] | 0 | 0, 0 | 0) | 0)) {
    break label$1
   }
   $351($0_1 | 0, $1_1 | 0, $2_1 | 0, $3_1 | 0);
   return;
  }
  $4_1 = HEAP32[($0_1 + 12 | 0) >> 2] | 0;
  $5_1 = $0_1 + 16 | 0;
  $354($5_1 | 0, $1_1 | 0, $2_1 | 0, $3_1 | 0);
  label$2 : {
   if (($4_1 | 0) < (2 | 0)) {
    break label$2
   }
   $4_1 = $5_1 + ($4_1 << 3 | 0) | 0;
   $0_1 = $0_1 + 24 | 0;
   label$3 : while (1) {
    $354($0_1 | 0, $1_1 | 0, $2_1 | 0, $3_1 | 0);
    if (HEAPU8[($1_1 + 54 | 0) >> 0] | 0) {
     break label$2
    }
    $0_1 = $0_1 + 8 | 0;
    if ($0_1 >>> 0 < $4_1 >>> 0) {
     continue label$3
    }
    break label$3;
   };
  }
 }
 
 function $356($0_1, $1_1, $2_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  var $3_1 = 0, $4_1 = 0;
  $3_1 = 1;
  label$1 : {
   label$2 : {
    if ((HEAPU8[($0_1 + 8 | 0) >> 0] | 0) & 24 | 0) {
     break label$2
    }
    $3_1 = 0;
    if (!$1_1) {
     break label$1
    }
    $4_1 = $350($1_1 | 0, 3868 | 0, 3964 | 0, 0 | 0) | 0;
    if (!$4_1) {
     break label$1
    }
    $3_1 = ((HEAPU8[($4_1 + 8 | 0) >> 0] | 0) & 24 | 0 | 0) != (0 | 0);
   }
   $3_1 = $346($0_1 | 0, $1_1 | 0, $3_1 | 0) | 0;
  }
  return $3_1 | 0;
 }
 
 function $357($0_1, $1_1, $2_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  var $5_1 = 0, $4_1 = 0, $3_1 = 0, $6_1 = 0;
  $3_1 = global$0 - 64 | 0;
  global$0 = $3_1;
  label$1 : {
   label$2 : {
    if (!($346($1_1 | 0, 4232 | 0, 0 | 0) | 0)) {
     break label$2
    }
    HEAP32[$2_1 >> 2] = 0;
    $4_1 = 1;
    break label$1;
   }
   label$3 : {
    if (!($356($0_1 | 0, $1_1 | 0, $1_1 | 0) | 0)) {
     break label$3
    }
    $4_1 = 1;
    $1_1 = HEAP32[$2_1 >> 2] | 0;
    if (!$1_1) {
     break label$1
    }
    HEAP32[$2_1 >> 2] = HEAP32[$1_1 >> 2] | 0;
    break label$1;
   }
   label$4 : {
    if (!$1_1) {
     break label$4
    }
    $4_1 = 0;
    $1_1 = $350($1_1 | 0, 3868 | 0, 4012 | 0, 0 | 0) | 0;
    if (!$1_1) {
     break label$1
    }
    label$5 : {
     $5_1 = HEAP32[$2_1 >> 2] | 0;
     if (!$5_1) {
      break label$5
     }
     HEAP32[$2_1 >> 2] = HEAP32[$5_1 >> 2] | 0;
    }
    $5_1 = HEAP32[($1_1 + 8 | 0) >> 2] | 0;
    $6_1 = HEAP32[($0_1 + 8 | 0) >> 2] | 0;
    if (($5_1 & ($6_1 ^ -1 | 0) | 0) & 7 | 0) {
     break label$1
    }
    if ((($5_1 ^ -1 | 0) & $6_1 | 0) & 96 | 0) {
     break label$1
    }
    $4_1 = 1;
    if ($346(HEAP32[($0_1 + 12 | 0) >> 2] | 0 | 0, HEAP32[($1_1 + 12 | 0) >> 2] | 0 | 0, 0 | 0) | 0) {
     break label$1
    }
    label$6 : {
     if (!($346(HEAP32[($0_1 + 12 | 0) >> 2] | 0 | 0, 4220 | 0, 0 | 0) | 0)) {
      break label$6
     }
     $1_1 = HEAP32[($1_1 + 12 | 0) >> 2] | 0;
     if (!$1_1) {
      break label$1
     }
     $4_1 = !($350($1_1 | 0, 3868 | 0, 4064 | 0, 0 | 0) | 0);
     break label$1;
    }
    $5_1 = HEAP32[($0_1 + 12 | 0) >> 2] | 0;
    if (!$5_1) {
     break label$4
    }
    $4_1 = 0;
    label$7 : {
     $5_1 = $350($5_1 | 0, 3868 | 0, 4012 | 0, 0 | 0) | 0;
     if (!$5_1) {
      break label$7
     }
     if (!((HEAPU8[($0_1 + 8 | 0) >> 0] | 0) & 1 | 0)) {
      break label$1
     }
     $4_1 = $358($5_1 | 0, HEAP32[($1_1 + 12 | 0) >> 2] | 0 | 0) | 0;
     break label$1;
    }
    $5_1 = HEAP32[($0_1 + 12 | 0) >> 2] | 0;
    if (!$5_1) {
     break label$1
    }
    $4_1 = 0;
    label$8 : {
     $5_1 = $350($5_1 | 0, 3868 | 0, 4124 | 0, 0 | 0) | 0;
     if (!$5_1) {
      break label$8
     }
     if (!((HEAPU8[($0_1 + 8 | 0) >> 0] | 0) & 1 | 0)) {
      break label$1
     }
     $4_1 = $359($5_1 | 0, HEAP32[($1_1 + 12 | 0) >> 2] | 0 | 0) | 0;
     break label$1;
    }
    $0_1 = HEAP32[($0_1 + 12 | 0) >> 2] | 0;
    if (!$0_1) {
     break label$1
    }
    $4_1 = 0;
    $0_1 = $350($0_1 | 0, 3868 | 0, 3916 | 0, 0 | 0) | 0;
    if (!$0_1) {
     break label$1
    }
    $1_1 = HEAP32[($1_1 + 12 | 0) >> 2] | 0;
    if (!$1_1) {
     break label$1
    }
    $4_1 = 0;
    $1_1 = $350($1_1 | 0, 3868 | 0, 3916 | 0, 0 | 0) | 0;
    if (!$1_1) {
     break label$1
    }
    HEAP32[($3_1 + 20 | 0) >> 2] = -1;
    HEAP32[($3_1 + 16 | 0) >> 2] = $0_1;
    HEAP32[($3_1 + 12 | 0) >> 2] = 0;
    HEAP32[($3_1 + 8 | 0) >> 2] = $1_1;
    $376($3_1 + 24 | 0 | 0, 0 | 0, 39 | 0) | 0;
    HEAP32[($3_1 + 56 | 0) >> 2] = 1;
    FUNCTION_TABLE[HEAP32[((HEAP32[$1_1 >> 2] | 0) + 28 | 0) >> 2] | 0 | 0]($1_1, $3_1 + 8 | 0, HEAP32[$2_1 >> 2] | 0, 1);
    $1_1 = HEAP32[($3_1 + 32 | 0) >> 2] | 0;
    label$9 : {
     if (!(HEAP32[$2_1 >> 2] | 0)) {
      break label$9
     }
     if (($1_1 | 0) != (1 | 0)) {
      break label$9
     }
     HEAP32[$2_1 >> 2] = HEAP32[($3_1 + 24 | 0) >> 2] | 0;
    }
    $4_1 = ($1_1 | 0) == (1 | 0);
    break label$1;
   }
   $4_1 = 0;
  }
  global$0 = $3_1 + 64 | 0;
  return $4_1 | 0;
 }
 
 function $358($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  var $3_1 = 0, $2_1 = 0;
  label$1 : {
   label$2 : while (1) {
    label$3 : {
     if ($1_1) {
      break label$3
     }
     return 0 | 0;
    }
    $2_1 = 0;
    $1_1 = $350($1_1 | 0, 3868 | 0, 4012 | 0, 0 | 0) | 0;
    if (!$1_1) {
     break label$1
    }
    if ((HEAP32[($1_1 + 8 | 0) >> 2] | 0) & ((HEAP32[($0_1 + 8 | 0) >> 2] | 0) ^ -1 | 0) | 0) {
     break label$1
    }
    label$4 : {
     if (!($346(HEAP32[($0_1 + 12 | 0) >> 2] | 0 | 0, HEAP32[($1_1 + 12 | 0) >> 2] | 0 | 0, 0 | 0) | 0)) {
      break label$4
     }
     return 1 | 0;
    }
    if (!((HEAPU8[($0_1 + 8 | 0) >> 0] | 0) & 1 | 0)) {
     break label$1
    }
    $3_1 = HEAP32[($0_1 + 12 | 0) >> 2] | 0;
    if (!$3_1) {
     break label$1
    }
    label$5 : {
     $3_1 = $350($3_1 | 0, 3868 | 0, 4012 | 0, 0 | 0) | 0;
     if (!$3_1) {
      break label$5
     }
     $1_1 = HEAP32[($1_1 + 12 | 0) >> 2] | 0;
     $0_1 = $3_1;
     continue label$2;
    }
    break label$2;
   };
   $0_1 = HEAP32[($0_1 + 12 | 0) >> 2] | 0;
   if (!$0_1) {
    break label$1
   }
   $2_1 = 0;
   $0_1 = $350($0_1 | 0, 3868 | 0, 4124 | 0, 0 | 0) | 0;
   if (!$0_1) {
    break label$1
   }
   $2_1 = $359($0_1 | 0, HEAP32[($1_1 + 12 | 0) >> 2] | 0 | 0) | 0;
  }
  return $2_1 | 0;
 }
 
 function $359($0_1, $1_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  label$1 : {
   if (!$1_1) {
    break label$1
   }
   $1_1 = $350($1_1 | 0, 3868 | 0, 4124 | 0, 0 | 0) | 0;
   if (!$1_1) {
    break label$1
   }
   if ((HEAP32[($1_1 + 8 | 0) >> 2] | 0) & ((HEAP32[($0_1 + 8 | 0) >> 2] | 0) ^ -1 | 0) | 0) {
    break label$1
   }
   if (!($346(HEAP32[($0_1 + 12 | 0) >> 2] | 0 | 0, HEAP32[($1_1 + 12 | 0) >> 2] | 0 | 0, 0 | 0) | 0)) {
    break label$1
   }
   return $346(HEAP32[($0_1 + 16 | 0) >> 2] | 0 | 0, HEAP32[($1_1 + 16 | 0) >> 2] | 0 | 0, 0 | 0) | 0 | 0;
  }
  return 0 | 0;
 }
 
 function $360($0_1, $1_1, $2_1, $3_1, $4_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  $4_1 = $4_1 | 0;
  HEAP8[($1_1 + 53 | 0) >> 0] = 1;
  label$1 : {
   if ((HEAP32[($1_1 + 4 | 0) >> 2] | 0 | 0) != ($3_1 | 0)) {
    break label$1
   }
   HEAP8[($1_1 + 52 | 0) >> 0] = 1;
   label$2 : {
    $3_1 = HEAP32[($1_1 + 16 | 0) >> 2] | 0;
    if ($3_1) {
     break label$2
    }
    HEAP32[($1_1 + 36 | 0) >> 2] = 1;
    HEAP32[($1_1 + 24 | 0) >> 2] = $4_1;
    HEAP32[($1_1 + 16 | 0) >> 2] = $2_1;
    if (($4_1 | 0) != (1 | 0)) {
     break label$1
    }
    if ((HEAP32[($1_1 + 48 | 0) >> 2] | 0 | 0) != (1 | 0)) {
     break label$1
    }
    HEAP8[($1_1 + 54 | 0) >> 0] = 1;
    return;
   }
   label$3 : {
    if (($3_1 | 0) != ($2_1 | 0)) {
     break label$3
    }
    label$4 : {
     $3_1 = HEAP32[($1_1 + 24 | 0) >> 2] | 0;
     if (($3_1 | 0) != (2 | 0)) {
      break label$4
     }
     HEAP32[($1_1 + 24 | 0) >> 2] = $4_1;
     $3_1 = $4_1;
    }
    if ((HEAP32[($1_1 + 48 | 0) >> 2] | 0 | 0) != (1 | 0)) {
     break label$1
    }
    if (($3_1 | 0) != (1 | 0)) {
     break label$1
    }
    HEAP8[($1_1 + 54 | 0) >> 0] = 1;
    return;
   }
   HEAP8[($1_1 + 54 | 0) >> 0] = 1;
   HEAP32[($1_1 + 36 | 0) >> 2] = (HEAP32[($1_1 + 36 | 0) >> 2] | 0) + 1 | 0;
  }
 }
 
 function $361($0_1, $1_1, $2_1, $3_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  label$1 : {
   if ((HEAP32[($1_1 + 4 | 0) >> 2] | 0 | 0) != ($2_1 | 0)) {
    break label$1
   }
   if ((HEAP32[($1_1 + 28 | 0) >> 2] | 0 | 0) == (1 | 0)) {
    break label$1
   }
   HEAP32[($1_1 + 28 | 0) >> 2] = $3_1;
  }
 }
 
 function $362($0_1, $1_1, $2_1, $3_1, $4_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  $4_1 = $4_1 | 0;
  var $5_1 = 0, $8_1 = 0, $6_1 = 0, $7_1 = 0;
  label$1 : {
   if (!($346($0_1 | 0, HEAP32[($1_1 + 8 | 0) >> 2] | 0 | 0, $4_1 | 0) | 0)) {
    break label$1
   }
   $361($1_1 | 0, $1_1 | 0, $2_1 | 0, $3_1 | 0);
   return;
  }
  label$2 : {
   label$3 : {
    if (!($346($0_1 | 0, HEAP32[$1_1 >> 2] | 0 | 0, $4_1 | 0) | 0)) {
     break label$3
    }
    label$4 : {
     label$5 : {
      if ((HEAP32[($1_1 + 16 | 0) >> 2] | 0 | 0) == ($2_1 | 0)) {
       break label$5
      }
      if ((HEAP32[($1_1 + 20 | 0) >> 2] | 0 | 0) != ($2_1 | 0)) {
       break label$4
      }
     }
     if (($3_1 | 0) != (1 | 0)) {
      break label$2
     }
     HEAP32[($1_1 + 32 | 0) >> 2] = 1;
     return;
    }
    HEAP32[($1_1 + 32 | 0) >> 2] = $3_1;
    label$6 : {
     if ((HEAP32[($1_1 + 44 | 0) >> 2] | 0 | 0) == (4 | 0)) {
      break label$6
     }
     $5_1 = $0_1 + 16 | 0;
     $3_1 = $5_1 + ((HEAP32[($0_1 + 12 | 0) >> 2] | 0) << 3 | 0) | 0;
     $6_1 = 0;
     $7_1 = 0;
     label$7 : {
      label$8 : {
       label$9 : {
        label$10 : while (1) {
         if ($5_1 >>> 0 >= $3_1 >>> 0) {
          break label$9
         }
         HEAP16[($1_1 + 52 | 0) >> 1] = 0;
         $363($5_1 | 0, $1_1 | 0, $2_1 | 0, $2_1 | 0, 1 | 0, $4_1 | 0);
         if (HEAPU8[($1_1 + 54 | 0) >> 0] | 0) {
          break label$9
         }
         label$11 : {
          if (!(HEAPU8[($1_1 + 53 | 0) >> 0] | 0)) {
           break label$11
          }
          label$12 : {
           if (!(HEAPU8[($1_1 + 52 | 0) >> 0] | 0)) {
            break label$12
           }
           $8_1 = 1;
           if ((HEAP32[($1_1 + 24 | 0) >> 2] | 0 | 0) == (1 | 0)) {
            break label$8
           }
           $6_1 = 1;
           $7_1 = 1;
           $8_1 = 1;
           if ((HEAPU8[($0_1 + 8 | 0) >> 0] | 0) & 2 | 0) {
            break label$11
           }
           break label$8;
          }
          $6_1 = 1;
          $8_1 = $7_1;
          if (!((HEAPU8[($0_1 + 8 | 0) >> 0] | 0) & 1 | 0)) {
           break label$8
          }
         }
         $5_1 = $5_1 + 8 | 0;
         continue label$10;
        };
       }
       $5_1 = 4;
       $8_1 = $7_1;
       if (!($6_1 & 1 | 0)) {
        break label$7
       }
      }
      $5_1 = 3;
     }
     HEAP32[($1_1 + 44 | 0) >> 2] = $5_1;
     if ($8_1 & 1 | 0) {
      break label$2
     }
    }
    HEAP32[($1_1 + 20 | 0) >> 2] = $2_1;
    HEAP32[($1_1 + 40 | 0) >> 2] = (HEAP32[($1_1 + 40 | 0) >> 2] | 0) + 1 | 0;
    if ((HEAP32[($1_1 + 36 | 0) >> 2] | 0 | 0) != (1 | 0)) {
     break label$2
    }
    if ((HEAP32[($1_1 + 24 | 0) >> 2] | 0 | 0) != (2 | 0)) {
     break label$2
    }
    HEAP8[($1_1 + 54 | 0) >> 0] = 1;
    return;
   }
   $5_1 = HEAP32[($0_1 + 12 | 0) >> 2] | 0;
   $8_1 = $0_1 + 16 | 0;
   $364($8_1 | 0, $1_1 | 0, $2_1 | 0, $3_1 | 0, $4_1 | 0);
   if (($5_1 | 0) < (2 | 0)) {
    break label$2
   }
   $8_1 = $8_1 + ($5_1 << 3 | 0) | 0;
   $5_1 = $0_1 + 24 | 0;
   label$13 : {
    label$14 : {
     $0_1 = HEAP32[($0_1 + 8 | 0) >> 2] | 0;
     if ($0_1 & 2 | 0) {
      break label$14
     }
     if ((HEAP32[($1_1 + 36 | 0) >> 2] | 0 | 0) != (1 | 0)) {
      break label$13
     }
    }
    label$15 : while (1) {
     if (HEAPU8[($1_1 + 54 | 0) >> 0] | 0) {
      break label$2
     }
     $364($5_1 | 0, $1_1 | 0, $2_1 | 0, $3_1 | 0, $4_1 | 0);
     $5_1 = $5_1 + 8 | 0;
     if ($5_1 >>> 0 < $8_1 >>> 0) {
      continue label$15
     }
     break label$2;
    };
   }
   label$16 : {
    if ($0_1 & 1 | 0) {
     break label$16
    }
    label$17 : while (1) {
     if (HEAPU8[($1_1 + 54 | 0) >> 0] | 0) {
      break label$2
     }
     if ((HEAP32[($1_1 + 36 | 0) >> 2] | 0 | 0) == (1 | 0)) {
      break label$2
     }
     $364($5_1 | 0, $1_1 | 0, $2_1 | 0, $3_1 | 0, $4_1 | 0);
     $5_1 = $5_1 + 8 | 0;
     if ($5_1 >>> 0 < $8_1 >>> 0) {
      continue label$17
     }
     break label$2;
    };
   }
   label$18 : while (1) {
    if (HEAPU8[($1_1 + 54 | 0) >> 0] | 0) {
     break label$2
    }
    label$19 : {
     if ((HEAP32[($1_1 + 36 | 0) >> 2] | 0 | 0) != (1 | 0)) {
      break label$19
     }
     if ((HEAP32[($1_1 + 24 | 0) >> 2] | 0 | 0) == (1 | 0)) {
      break label$2
     }
    }
    $364($5_1 | 0, $1_1 | 0, $2_1 | 0, $3_1 | 0, $4_1 | 0);
    $5_1 = $5_1 + 8 | 0;
    if ($5_1 >>> 0 < $8_1 >>> 0) {
     continue label$18
    }
    break label$18;
   };
  }
 }
 
 function $363($0_1, $1_1, $2_1, $3_1, $4_1, $5_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  $4_1 = $4_1 | 0;
  $5_1 = $5_1 | 0;
  var $6_1 = 0, $7_1 = 0;
  $6_1 = HEAP32[($0_1 + 4 | 0) >> 2] | 0;
  $7_1 = $6_1 >> 8 | 0;
  label$1 : {
   if (!($6_1 & 1 | 0)) {
    break label$1
   }
   $7_1 = HEAP32[((HEAP32[$3_1 >> 2] | 0) + $7_1 | 0) >> 2] | 0;
  }
  $0_1 = HEAP32[$0_1 >> 2] | 0;
  FUNCTION_TABLE[HEAP32[((HEAP32[$0_1 >> 2] | 0) + 20 | 0) >> 2] | 0 | 0]($0_1, $1_1, $2_1, $3_1 + $7_1 | 0, $6_1 & 2 | 0 ? $4_1 : 2, $5_1);
 }
 
 function $364($0_1, $1_1, $2_1, $3_1, $4_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  $4_1 = $4_1 | 0;
  var $5_1 = 0, $6_1 = 0;
  $5_1 = HEAP32[($0_1 + 4 | 0) >> 2] | 0;
  $6_1 = $5_1 >> 8 | 0;
  label$1 : {
   if (!($5_1 & 1 | 0)) {
    break label$1
   }
   $6_1 = HEAP32[((HEAP32[$2_1 >> 2] | 0) + $6_1 | 0) >> 2] | 0;
  }
  $0_1 = HEAP32[$0_1 >> 2] | 0;
  FUNCTION_TABLE[HEAP32[((HEAP32[$0_1 >> 2] | 0) + 24 | 0) >> 2] | 0 | 0]($0_1, $1_1, $2_1 + $6_1 | 0, $5_1 & 2 | 0 ? $3_1 : 2, $4_1);
 }
 
 function $365($0_1, $1_1, $2_1, $3_1, $4_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  $4_1 = $4_1 | 0;
  label$1 : {
   if (!($346($0_1 | 0, HEAP32[($1_1 + 8 | 0) >> 2] | 0 | 0, $4_1 | 0) | 0)) {
    break label$1
   }
   $361($1_1 | 0, $1_1 | 0, $2_1 | 0, $3_1 | 0);
   return;
  }
  label$2 : {
   label$3 : {
    if (!($346($0_1 | 0, HEAP32[$1_1 >> 2] | 0 | 0, $4_1 | 0) | 0)) {
     break label$3
    }
    label$4 : {
     label$5 : {
      if ((HEAP32[($1_1 + 16 | 0) >> 2] | 0 | 0) == ($2_1 | 0)) {
       break label$5
      }
      if ((HEAP32[($1_1 + 20 | 0) >> 2] | 0 | 0) != ($2_1 | 0)) {
       break label$4
      }
     }
     if (($3_1 | 0) != (1 | 0)) {
      break label$2
     }
     HEAP32[($1_1 + 32 | 0) >> 2] = 1;
     return;
    }
    HEAP32[($1_1 + 32 | 0) >> 2] = $3_1;
    label$6 : {
     if ((HEAP32[($1_1 + 44 | 0) >> 2] | 0 | 0) == (4 | 0)) {
      break label$6
     }
     HEAP16[($1_1 + 52 | 0) >> 1] = 0;
     $0_1 = HEAP32[($0_1 + 8 | 0) >> 2] | 0;
     FUNCTION_TABLE[HEAP32[((HEAP32[$0_1 >> 2] | 0) + 20 | 0) >> 2] | 0 | 0]($0_1, $1_1, $2_1, $2_1, 1, $4_1);
     label$7 : {
      if (!(HEAPU8[($1_1 + 53 | 0) >> 0] | 0)) {
       break label$7
      }
      HEAP32[($1_1 + 44 | 0) >> 2] = 3;
      if (!(HEAPU8[($1_1 + 52 | 0) >> 0] | 0)) {
       break label$6
      }
      break label$2;
     }
     HEAP32[($1_1 + 44 | 0) >> 2] = 4;
    }
    HEAP32[($1_1 + 20 | 0) >> 2] = $2_1;
    HEAP32[($1_1 + 40 | 0) >> 2] = (HEAP32[($1_1 + 40 | 0) >> 2] | 0) + 1 | 0;
    if ((HEAP32[($1_1 + 36 | 0) >> 2] | 0 | 0) != (1 | 0)) {
     break label$2
    }
    if ((HEAP32[($1_1 + 24 | 0) >> 2] | 0 | 0) != (2 | 0)) {
     break label$2
    }
    HEAP8[($1_1 + 54 | 0) >> 0] = 1;
    return;
   }
   $0_1 = HEAP32[($0_1 + 8 | 0) >> 2] | 0;
   FUNCTION_TABLE[HEAP32[((HEAP32[$0_1 >> 2] | 0) + 24 | 0) >> 2] | 0 | 0]($0_1, $1_1, $2_1, $3_1, $4_1);
  }
 }
 
 function $366($0_1, $1_1, $2_1, $3_1, $4_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  $4_1 = $4_1 | 0;
  label$1 : {
   if (!($346($0_1 | 0, HEAP32[($1_1 + 8 | 0) >> 2] | 0 | 0, $4_1 | 0) | 0)) {
    break label$1
   }
   $361($1_1 | 0, $1_1 | 0, $2_1 | 0, $3_1 | 0);
   return;
  }
  label$2 : {
   if (!($346($0_1 | 0, HEAP32[$1_1 >> 2] | 0 | 0, $4_1 | 0) | 0)) {
    break label$2
   }
   label$3 : {
    label$4 : {
     if ((HEAP32[($1_1 + 16 | 0) >> 2] | 0 | 0) == ($2_1 | 0)) {
      break label$4
     }
     if ((HEAP32[($1_1 + 20 | 0) >> 2] | 0 | 0) != ($2_1 | 0)) {
      break label$3
     }
    }
    if (($3_1 | 0) != (1 | 0)) {
     break label$2
    }
    HEAP32[($1_1 + 32 | 0) >> 2] = 1;
    return;
   }
   HEAP32[($1_1 + 20 | 0) >> 2] = $2_1;
   HEAP32[($1_1 + 32 | 0) >> 2] = $3_1;
   HEAP32[($1_1 + 40 | 0) >> 2] = (HEAP32[($1_1 + 40 | 0) >> 2] | 0) + 1 | 0;
   label$5 : {
    if ((HEAP32[($1_1 + 36 | 0) >> 2] | 0 | 0) != (1 | 0)) {
     break label$5
    }
    if ((HEAP32[($1_1 + 24 | 0) >> 2] | 0 | 0) != (2 | 0)) {
     break label$5
    }
    HEAP8[($1_1 + 54 | 0) >> 0] = 1;
   }
   HEAP32[($1_1 + 44 | 0) >> 2] = 4;
  }
 }
 
 function $367($0_1, $1_1, $2_1, $3_1, $4_1, $5_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  $4_1 = $4_1 | 0;
  $5_1 = $5_1 | 0;
  var $7_1 = 0, $6_1 = 0, $8_1 = 0, $9_1 = 0, $10_1 = 0, $11_1 = 0;
  label$1 : {
   if (!($346($0_1 | 0, HEAP32[($1_1 + 8 | 0) >> 2] | 0 | 0, $5_1 | 0) | 0)) {
    break label$1
   }
   $360($1_1 | 0, $1_1 | 0, $2_1 | 0, $3_1 | 0, $4_1 | 0);
   return;
  }
  $6_1 = HEAPU8[($1_1 + 53 | 0) >> 0] | 0;
  $7_1 = HEAP32[($0_1 + 12 | 0) >> 2] | 0;
  HEAP8[($1_1 + 53 | 0) >> 0] = 0;
  $8_1 = HEAPU8[($1_1 + 52 | 0) >> 0] | 0;
  HEAP8[($1_1 + 52 | 0) >> 0] = 0;
  $9_1 = $0_1 + 16 | 0;
  $363($9_1 | 0, $1_1 | 0, $2_1 | 0, $3_1 | 0, $4_1 | 0, $5_1 | 0);
  $10_1 = HEAPU8[($1_1 + 53 | 0) >> 0] | 0;
  $6_1 = $6_1 | $10_1 | 0;
  $11_1 = HEAPU8[($1_1 + 52 | 0) >> 0] | 0;
  $8_1 = $8_1 | $11_1 | 0;
  label$2 : {
   if (($7_1 | 0) < (2 | 0)) {
    break label$2
   }
   $9_1 = $9_1 + ($7_1 << 3 | 0) | 0;
   $7_1 = $0_1 + 24 | 0;
   label$3 : while (1) {
    if (HEAPU8[($1_1 + 54 | 0) >> 0] | 0) {
     break label$2
    }
    label$4 : {
     label$5 : {
      if (!($11_1 & 255 | 0)) {
       break label$5
      }
      if ((HEAP32[($1_1 + 24 | 0) >> 2] | 0 | 0) == (1 | 0)) {
       break label$2
      }
      if ((HEAPU8[($0_1 + 8 | 0) >> 0] | 0) & 2 | 0) {
       break label$4
      }
      break label$2;
     }
     if (!($10_1 & 255 | 0)) {
      break label$4
     }
     if (!((HEAPU8[($0_1 + 8 | 0) >> 0] | 0) & 1 | 0)) {
      break label$2
     }
    }
    HEAP16[($1_1 + 52 | 0) >> 1] = 0;
    $363($7_1 | 0, $1_1 | 0, $2_1 | 0, $3_1 | 0, $4_1 | 0, $5_1 | 0);
    $10_1 = HEAPU8[($1_1 + 53 | 0) >> 0] | 0;
    $6_1 = $10_1 | $6_1 | 0;
    $11_1 = HEAPU8[($1_1 + 52 | 0) >> 0] | 0;
    $8_1 = $11_1 | $8_1 | 0;
    $7_1 = $7_1 + 8 | 0;
    if ($7_1 >>> 0 < $9_1 >>> 0) {
     continue label$3
    }
    break label$3;
   };
  }
  HEAP8[($1_1 + 53 | 0) >> 0] = ($6_1 & 255 | 0 | 0) != (0 | 0);
  HEAP8[($1_1 + 52 | 0) >> 0] = ($8_1 & 255 | 0 | 0) != (0 | 0);
 }
 
 function $368($0_1, $1_1, $2_1, $3_1, $4_1, $5_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  $4_1 = $4_1 | 0;
  $5_1 = $5_1 | 0;
  label$1 : {
   if (!($346($0_1 | 0, HEAP32[($1_1 + 8 | 0) >> 2] | 0 | 0, $5_1 | 0) | 0)) {
    break label$1
   }
   $360($1_1 | 0, $1_1 | 0, $2_1 | 0, $3_1 | 0, $4_1 | 0);
   return;
  }
  $0_1 = HEAP32[($0_1 + 8 | 0) >> 2] | 0;
  FUNCTION_TABLE[HEAP32[((HEAP32[$0_1 >> 2] | 0) + 20 | 0) >> 2] | 0 | 0]($0_1, $1_1, $2_1, $3_1, $4_1, $5_1);
 }
 
 function $369($0_1, $1_1, $2_1, $3_1, $4_1, $5_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  $3_1 = $3_1 | 0;
  $4_1 = $4_1 | 0;
  $5_1 = $5_1 | 0;
  label$1 : {
   if (!($346($0_1 | 0, HEAP32[($1_1 + 8 | 0) >> 2] | 0 | 0, $5_1 | 0) | 0)) {
    break label$1
   }
   $360($1_1 | 0, $1_1 | 0, $2_1 | 0, $3_1 | 0, $4_1 | 0);
  }
 }
 
 function $370() {
  return 4720 | 0;
 }
 
 function $371($0_1) {
  $0_1 = $0_1 | 0;
  var $4_1 = 0, $6_1 = 0, $5_1 = 0, $3_1 = 0, $8_1 = 0, $2_1 = 0, $12_1 = 0, $9_1 = 0, $7_1 = 0, i64toi32_i32$0 = 0, i64toi32_i32$1 = 0, i64toi32_i32$2 = 0, $10_1 = 0, $11_1 = 0, $1_1 = 0, $83_1 = 0, $96_1 = 0, $107_1 = 0, $115_1 = 0, $123_1 = 0, $217_1 = 0, $228_1 = 0, $236_1 = 0, $244_1 = 0, $279_1 = 0, $363_1 = 0, $370_1 = 0, $461 = 0, $472 = 0, $480 = 0, $488 = 0, $1205 = 0, $1212 = 0, $1334 = 0, $1336 = 0, $1406 = 0, $1413 = 0, $1654 = 0, $1661 = 0;
  $1_1 = global$0 - 16 | 0;
  global$0 = $1_1;
  label$1 : {
   label$2 : {
    label$3 : {
     label$4 : {
      label$5 : {
       label$6 : {
        label$7 : {
         label$8 : {
          label$9 : {
           label$10 : {
            label$11 : {
             label$12 : {
              label$13 : {
               if ($0_1 >>> 0 > 244 >>> 0) {
                break label$13
               }
               label$14 : {
                $2_1 = HEAP32[(0 + 4724 | 0) >> 2] | 0;
                $3_1 = $0_1 >>> 0 < 11 >>> 0 ? 16 : ($0_1 + 11 | 0) & -8 | 0;
                $4_1 = $3_1 >>> 3 | 0;
                $0_1 = $2_1 >>> $4_1 | 0;
                if (!($0_1 & 3 | 0)) {
                 break label$14
                }
                $3_1 = (($0_1 ^ -1 | 0) & 1 | 0) + $4_1 | 0;
                $5_1 = $3_1 << 3 | 0;
                $4_1 = HEAP32[($5_1 + 4772 | 0) >> 2] | 0;
                $0_1 = $4_1 + 8 | 0;
                label$15 : {
                 label$16 : {
                  $6_1 = HEAP32[($4_1 + 8 | 0) >> 2] | 0;
                  $5_1 = $5_1 + 4764 | 0;
                  if (($6_1 | 0) != ($5_1 | 0)) {
                   break label$16
                  }
                  HEAP32[(0 + 4724 | 0) >> 2] = $2_1 & (__wasm_rotl_i32(-2 | 0, $3_1 | 0) | 0) | 0;
                  break label$15;
                 }
                 HEAP32[(0 + 4740 | 0) >> 2] | 0;
                 HEAP32[($6_1 + 12 | 0) >> 2] = $5_1;
                 HEAP32[($5_1 + 8 | 0) >> 2] = $6_1;
                }
                $6_1 = $3_1 << 3 | 0;
                HEAP32[($4_1 + 4 | 0) >> 2] = $6_1 | 3 | 0;
                $4_1 = $4_1 + $6_1 | 0;
                HEAP32[($4_1 + 4 | 0) >> 2] = HEAP32[($4_1 + 4 | 0) >> 2] | 0 | 1 | 0;
                break label$1;
               }
               $7_1 = HEAP32[(0 + 4732 | 0) >> 2] | 0;
               if ($3_1 >>> 0 <= $7_1 >>> 0) {
                break label$12
               }
               label$17 : {
                if (!$0_1) {
                 break label$17
                }
                label$18 : {
                 label$19 : {
                  $83_1 = $0_1 << $4_1 | 0;
                  $0_1 = 2 << $4_1 | 0;
                  $0_1 = $83_1 & ($0_1 | (0 - $0_1 | 0) | 0) | 0;
                  $0_1 = ($0_1 & (0 - $0_1 | 0) | 0) + -1 | 0;
                  $96_1 = $0_1;
                  $0_1 = ($0_1 >>> 12 | 0) & 16 | 0;
                  $4_1 = $96_1 >>> $0_1 | 0;
                  $6_1 = ($4_1 >>> 5 | 0) & 8 | 0;
                  $107_1 = $6_1 | $0_1 | 0;
                  $0_1 = $4_1 >>> $6_1 | 0;
                  $4_1 = ($0_1 >>> 2 | 0) & 4 | 0;
                  $115_1 = $107_1 | $4_1 | 0;
                  $0_1 = $0_1 >>> $4_1 | 0;
                  $4_1 = ($0_1 >>> 1 | 0) & 2 | 0;
                  $123_1 = $115_1 | $4_1 | 0;
                  $0_1 = $0_1 >>> $4_1 | 0;
                  $4_1 = ($0_1 >>> 1 | 0) & 1 | 0;
                  $6_1 = ($123_1 | $4_1 | 0) + ($0_1 >>> $4_1 | 0) | 0;
                  $5_1 = $6_1 << 3 | 0;
                  $4_1 = HEAP32[($5_1 + 4772 | 0) >> 2] | 0;
                  $0_1 = HEAP32[($4_1 + 8 | 0) >> 2] | 0;
                  $5_1 = $5_1 + 4764 | 0;
                  if (($0_1 | 0) != ($5_1 | 0)) {
                   break label$19
                  }
                  $2_1 = $2_1 & (__wasm_rotl_i32(-2 | 0, $6_1 | 0) | 0) | 0;
                  HEAP32[(0 + 4724 | 0) >> 2] = $2_1;
                  break label$18;
                 }
                 HEAP32[(0 + 4740 | 0) >> 2] | 0;
                 HEAP32[($0_1 + 12 | 0) >> 2] = $5_1;
                 HEAP32[($5_1 + 8 | 0) >> 2] = $0_1;
                }
                $0_1 = $4_1 + 8 | 0;
                HEAP32[($4_1 + 4 | 0) >> 2] = $3_1 | 3 | 0;
                $5_1 = $4_1 + $3_1 | 0;
                $8_1 = $6_1 << 3 | 0;
                $6_1 = $8_1 - $3_1 | 0;
                HEAP32[($5_1 + 4 | 0) >> 2] = $6_1 | 1 | 0;
                HEAP32[($4_1 + $8_1 | 0) >> 2] = $6_1;
                label$20 : {
                 if (!$7_1) {
                  break label$20
                 }
                 $8_1 = $7_1 >>> 3 | 0;
                 $3_1 = ($8_1 << 3 | 0) + 4764 | 0;
                 $4_1 = HEAP32[(0 + 4744 | 0) >> 2] | 0;
                 label$21 : {
                  label$22 : {
                   $8_1 = 1 << $8_1 | 0;
                   if ($2_1 & $8_1 | 0) {
                    break label$22
                   }
                   HEAP32[(0 + 4724 | 0) >> 2] = $2_1 | $8_1 | 0;
                   $8_1 = $3_1;
                   break label$21;
                  }
                  $8_1 = HEAP32[($3_1 + 8 | 0) >> 2] | 0;
                 }
                 HEAP32[($3_1 + 8 | 0) >> 2] = $4_1;
                 HEAP32[($8_1 + 12 | 0) >> 2] = $4_1;
                 HEAP32[($4_1 + 12 | 0) >> 2] = $3_1;
                 HEAP32[($4_1 + 8 | 0) >> 2] = $8_1;
                }
                HEAP32[(0 + 4744 | 0) >> 2] = $5_1;
                HEAP32[(0 + 4732 | 0) >> 2] = $6_1;
                break label$1;
               }
               $9_1 = HEAP32[(0 + 4728 | 0) >> 2] | 0;
               if (!$9_1) {
                break label$12
               }
               $0_1 = ($9_1 & (0 - $9_1 | 0) | 0) + -1 | 0;
               $217_1 = $0_1;
               $0_1 = ($0_1 >>> 12 | 0) & 16 | 0;
               $4_1 = $217_1 >>> $0_1 | 0;
               $6_1 = ($4_1 >>> 5 | 0) & 8 | 0;
               $228_1 = $6_1 | $0_1 | 0;
               $0_1 = $4_1 >>> $6_1 | 0;
               $4_1 = ($0_1 >>> 2 | 0) & 4 | 0;
               $236_1 = $228_1 | $4_1 | 0;
               $0_1 = $0_1 >>> $4_1 | 0;
               $4_1 = ($0_1 >>> 1 | 0) & 2 | 0;
               $244_1 = $236_1 | $4_1 | 0;
               $0_1 = $0_1 >>> $4_1 | 0;
               $4_1 = ($0_1 >>> 1 | 0) & 1 | 0;
               $5_1 = HEAP32[(((($244_1 | $4_1 | 0) + ($0_1 >>> $4_1 | 0) | 0) << 2 | 0) + 5028 | 0) >> 2] | 0;
               $4_1 = ((HEAP32[($5_1 + 4 | 0) >> 2] | 0) & -8 | 0) - $3_1 | 0;
               $6_1 = $5_1;
               label$23 : {
                label$24 : while (1) {
                 label$25 : {
                  $0_1 = HEAP32[($6_1 + 16 | 0) >> 2] | 0;
                  if ($0_1) {
                   break label$25
                  }
                  $0_1 = HEAP32[($6_1 + 20 | 0) >> 2] | 0;
                  if (!$0_1) {
                   break label$23
                  }
                 }
                 $6_1 = ((HEAP32[($0_1 + 4 | 0) >> 2] | 0) & -8 | 0) - $3_1 | 0;
                 $279_1 = $6_1;
                 $6_1 = $6_1 >>> 0 < $4_1 >>> 0;
                 $4_1 = $6_1 ? $279_1 : $4_1;
                 $5_1 = $6_1 ? $0_1 : $5_1;
                 $6_1 = $0_1;
                 continue label$24;
                };
               }
               $10_1 = $5_1 + $3_1 | 0;
               if ($10_1 >>> 0 <= $5_1 >>> 0) {
                break label$11
               }
               $11_1 = HEAP32[($5_1 + 24 | 0) >> 2] | 0;
               label$26 : {
                $8_1 = HEAP32[($5_1 + 12 | 0) >> 2] | 0;
                if (($8_1 | 0) == ($5_1 | 0)) {
                 break label$26
                }
                label$27 : {
                 $0_1 = HEAP32[($5_1 + 8 | 0) >> 2] | 0;
                 if ((HEAP32[(0 + 4740 | 0) >> 2] | 0) >>> 0 > $0_1 >>> 0) {
                  break label$27
                 }
                 HEAP32[($0_1 + 12 | 0) >> 2] | 0;
                }
                HEAP32[($0_1 + 12 | 0) >> 2] = $8_1;
                HEAP32[($8_1 + 8 | 0) >> 2] = $0_1;
                break label$2;
               }
               label$28 : {
                $6_1 = $5_1 + 20 | 0;
                $0_1 = HEAP32[$6_1 >> 2] | 0;
                if ($0_1) {
                 break label$28
                }
                $0_1 = HEAP32[($5_1 + 16 | 0) >> 2] | 0;
                if (!$0_1) {
                 break label$10
                }
                $6_1 = $5_1 + 16 | 0;
               }
               label$29 : while (1) {
                $12_1 = $6_1;
                $8_1 = $0_1;
                $6_1 = $0_1 + 20 | 0;
                $0_1 = HEAP32[$6_1 >> 2] | 0;
                if ($0_1) {
                 continue label$29
                }
                $6_1 = $8_1 + 16 | 0;
                $0_1 = HEAP32[($8_1 + 16 | 0) >> 2] | 0;
                if ($0_1) {
                 continue label$29
                }
                break label$29;
               };
               HEAP32[$12_1 >> 2] = 0;
               break label$2;
              }
              $3_1 = -1;
              if ($0_1 >>> 0 > -65 >>> 0) {
               break label$12
              }
              $0_1 = $0_1 + 11 | 0;
              $3_1 = $0_1 & -8 | 0;
              $7_1 = HEAP32[(0 + 4728 | 0) >> 2] | 0;
              if (!$7_1) {
               break label$12
              }
              $12_1 = 0;
              label$30 : {
               $0_1 = $0_1 >>> 8 | 0;
               if (!$0_1) {
                break label$30
               }
               $12_1 = 31;
               if ($3_1 >>> 0 > 16777215 >>> 0) {
                break label$30
               }
               $4_1 = (($0_1 + 1048320 | 0) >>> 16 | 0) & 8 | 0;
               $0_1 = $0_1 << $4_1 | 0;
               $363_1 = $0_1;
               $0_1 = (($0_1 + 520192 | 0) >>> 16 | 0) & 4 | 0;
               $6_1 = $363_1 << $0_1 | 0;
               $370_1 = $6_1;
               $6_1 = (($6_1 + 245760 | 0) >>> 16 | 0) & 2 | 0;
               $0_1 = (($370_1 << $6_1 | 0) >>> 15 | 0) - ($0_1 | $4_1 | 0 | $6_1 | 0) | 0;
               $12_1 = ($0_1 << 1 | 0 | (($3_1 >>> ($0_1 + 21 | 0) | 0) & 1 | 0) | 0) + 28 | 0;
              }
              $4_1 = 0 - $3_1 | 0;
              label$31 : {
               label$32 : {
                label$33 : {
                 label$34 : {
                  $6_1 = HEAP32[(($12_1 << 2 | 0) + 5028 | 0) >> 2] | 0;
                  if ($6_1) {
                   break label$34
                  }
                  $0_1 = 0;
                  $8_1 = 0;
                  break label$33;
                 }
                 $0_1 = 0;
                 $5_1 = $3_1 << (($12_1 | 0) == (31 | 0) ? 0 : 25 - ($12_1 >>> 1 | 0) | 0) | 0;
                 $8_1 = 0;
                 label$35 : while (1) {
                  label$36 : {
                   $2_1 = ((HEAP32[($6_1 + 4 | 0) >> 2] | 0) & -8 | 0) - $3_1 | 0;
                   if ($2_1 >>> 0 >= $4_1 >>> 0) {
                    break label$36
                   }
                   $4_1 = $2_1;
                   $8_1 = $6_1;
                   if ($4_1) {
                    break label$36
                   }
                   $4_1 = 0;
                   $8_1 = $6_1;
                   $0_1 = $6_1;
                   break label$32;
                  }
                  $2_1 = HEAP32[($6_1 + 20 | 0) >> 2] | 0;
                  $6_1 = HEAP32[(($6_1 + (($5_1 >>> 29 | 0) & 4 | 0) | 0) + 16 | 0) >> 2] | 0;
                  $0_1 = $2_1 ? (($2_1 | 0) == ($6_1 | 0) ? $0_1 : $2_1) : $0_1;
                  $5_1 = $5_1 << 1 | 0;
                  if ($6_1) {
                   continue label$35
                  }
                  break label$35;
                 };
                }
                label$37 : {
                 if ($0_1 | $8_1 | 0) {
                  break label$37
                 }
                 $0_1 = 2 << $12_1 | 0;
                 $0_1 = ($0_1 | (0 - $0_1 | 0) | 0) & $7_1 | 0;
                 if (!$0_1) {
                  break label$12
                 }
                 $0_1 = ($0_1 & (0 - $0_1 | 0) | 0) + -1 | 0;
                 $461 = $0_1;
                 $0_1 = ($0_1 >>> 12 | 0) & 16 | 0;
                 $6_1 = $461 >>> $0_1 | 0;
                 $5_1 = ($6_1 >>> 5 | 0) & 8 | 0;
                 $472 = $5_1 | $0_1 | 0;
                 $0_1 = $6_1 >>> $5_1 | 0;
                 $6_1 = ($0_1 >>> 2 | 0) & 4 | 0;
                 $480 = $472 | $6_1 | 0;
                 $0_1 = $0_1 >>> $6_1 | 0;
                 $6_1 = ($0_1 >>> 1 | 0) & 2 | 0;
                 $488 = $480 | $6_1 | 0;
                 $0_1 = $0_1 >>> $6_1 | 0;
                 $6_1 = ($0_1 >>> 1 | 0) & 1 | 0;
                 $0_1 = HEAP32[(((($488 | $6_1 | 0) + ($0_1 >>> $6_1 | 0) | 0) << 2 | 0) + 5028 | 0) >> 2] | 0;
                }
                if (!$0_1) {
                 break label$31
                }
               }
               label$38 : while (1) {
                $2_1 = ((HEAP32[($0_1 + 4 | 0) >> 2] | 0) & -8 | 0) - $3_1 | 0;
                $5_1 = $2_1 >>> 0 < $4_1 >>> 0;
                label$39 : {
                 $6_1 = HEAP32[($0_1 + 16 | 0) >> 2] | 0;
                 if ($6_1) {
                  break label$39
                 }
                 $6_1 = HEAP32[($0_1 + 20 | 0) >> 2] | 0;
                }
                $4_1 = $5_1 ? $2_1 : $4_1;
                $8_1 = $5_1 ? $0_1 : $8_1;
                $0_1 = $6_1;
                if ($0_1) {
                 continue label$38
                }
                break label$38;
               };
              }
              if (!$8_1) {
               break label$12
              }
              if ($4_1 >>> 0 >= ((HEAP32[(0 + 4732 | 0) >> 2] | 0) - $3_1 | 0) >>> 0) {
               break label$12
              }
              $12_1 = $8_1 + $3_1 | 0;
              if ($12_1 >>> 0 <= $8_1 >>> 0) {
               break label$11
              }
              $9_1 = HEAP32[($8_1 + 24 | 0) >> 2] | 0;
              label$40 : {
               $5_1 = HEAP32[($8_1 + 12 | 0) >> 2] | 0;
               if (($5_1 | 0) == ($8_1 | 0)) {
                break label$40
               }
               label$41 : {
                $0_1 = HEAP32[($8_1 + 8 | 0) >> 2] | 0;
                if ((HEAP32[(0 + 4740 | 0) >> 2] | 0) >>> 0 > $0_1 >>> 0) {
                 break label$41
                }
                HEAP32[($0_1 + 12 | 0) >> 2] | 0;
               }
               HEAP32[($0_1 + 12 | 0) >> 2] = $5_1;
               HEAP32[($5_1 + 8 | 0) >> 2] = $0_1;
               break label$3;
              }
              label$42 : {
               $6_1 = $8_1 + 20 | 0;
               $0_1 = HEAP32[$6_1 >> 2] | 0;
               if ($0_1) {
                break label$42
               }
               $0_1 = HEAP32[($8_1 + 16 | 0) >> 2] | 0;
               if (!$0_1) {
                break label$9
               }
               $6_1 = $8_1 + 16 | 0;
              }
              label$43 : while (1) {
               $2_1 = $6_1;
               $5_1 = $0_1;
               $6_1 = $0_1 + 20 | 0;
               $0_1 = HEAP32[$6_1 >> 2] | 0;
               if ($0_1) {
                continue label$43
               }
               $6_1 = $5_1 + 16 | 0;
               $0_1 = HEAP32[($5_1 + 16 | 0) >> 2] | 0;
               if ($0_1) {
                continue label$43
               }
               break label$43;
              };
              HEAP32[$2_1 >> 2] = 0;
              break label$3;
             }
             label$44 : {
              $0_1 = HEAP32[(0 + 4732 | 0) >> 2] | 0;
              if ($0_1 >>> 0 < $3_1 >>> 0) {
               break label$44
              }
              $4_1 = HEAP32[(0 + 4744 | 0) >> 2] | 0;
              label$45 : {
               label$46 : {
                $6_1 = $0_1 - $3_1 | 0;
                if ($6_1 >>> 0 < 16 >>> 0) {
                 break label$46
                }
                HEAP32[(0 + 4732 | 0) >> 2] = $6_1;
                $5_1 = $4_1 + $3_1 | 0;
                HEAP32[(0 + 4744 | 0) >> 2] = $5_1;
                HEAP32[($5_1 + 4 | 0) >> 2] = $6_1 | 1 | 0;
                HEAP32[($4_1 + $0_1 | 0) >> 2] = $6_1;
                HEAP32[($4_1 + 4 | 0) >> 2] = $3_1 | 3 | 0;
                break label$45;
               }
               HEAP32[(0 + 4744 | 0) >> 2] = 0;
               HEAP32[(0 + 4732 | 0) >> 2] = 0;
               HEAP32[($4_1 + 4 | 0) >> 2] = $0_1 | 3 | 0;
               $0_1 = $4_1 + $0_1 | 0;
               HEAP32[($0_1 + 4 | 0) >> 2] = HEAP32[($0_1 + 4 | 0) >> 2] | 0 | 1 | 0;
              }
              $0_1 = $4_1 + 8 | 0;
              break label$1;
             }
             label$47 : {
              $5_1 = HEAP32[(0 + 4736 | 0) >> 2] | 0;
              if ($5_1 >>> 0 <= $3_1 >>> 0) {
               break label$47
              }
              $4_1 = $5_1 - $3_1 | 0;
              HEAP32[(0 + 4736 | 0) >> 2] = $4_1;
              $0_1 = HEAP32[(0 + 4748 | 0) >> 2] | 0;
              $6_1 = $0_1 + $3_1 | 0;
              HEAP32[(0 + 4748 | 0) >> 2] = $6_1;
              HEAP32[($6_1 + 4 | 0) >> 2] = $4_1 | 1 | 0;
              HEAP32[($0_1 + 4 | 0) >> 2] = $3_1 | 3 | 0;
              $0_1 = $0_1 + 8 | 0;
              break label$1;
             }
             label$48 : {
              label$49 : {
               if (!(HEAP32[(0 + 5196 | 0) >> 2] | 0)) {
                break label$49
               }
               $4_1 = HEAP32[(0 + 5204 | 0) >> 2] | 0;
               break label$48;
              }
              i64toi32_i32$1 = 0;
              i64toi32_i32$0 = -1;
              HEAP32[(i64toi32_i32$1 + 5208 | 0) >> 2] = -1;
              HEAP32[(i64toi32_i32$1 + 5212 | 0) >> 2] = i64toi32_i32$0;
              i64toi32_i32$1 = 0;
              i64toi32_i32$0 = 4096;
              HEAP32[(i64toi32_i32$1 + 5200 | 0) >> 2] = 4096;
              HEAP32[(i64toi32_i32$1 + 5204 | 0) >> 2] = i64toi32_i32$0;
              HEAP32[(0 + 5196 | 0) >> 2] = (($1_1 + 12 | 0) & -16 | 0) ^ 1431655768 | 0;
              HEAP32[(0 + 5216 | 0) >> 2] = 0;
              HEAP32[(0 + 5168 | 0) >> 2] = 0;
              $4_1 = 4096;
             }
             $0_1 = 0;
             $7_1 = $3_1 + 47 | 0;
             $2_1 = $4_1 + $7_1 | 0;
             $12_1 = 0 - $4_1 | 0;
             $8_1 = $2_1 & $12_1 | 0;
             if ($8_1 >>> 0 <= $3_1 >>> 0) {
              break label$1
             }
             $0_1 = 0;
             label$50 : {
              $4_1 = HEAP32[(0 + 5164 | 0) >> 2] | 0;
              if (!$4_1) {
               break label$50
              }
              $6_1 = HEAP32[(0 + 5156 | 0) >> 2] | 0;
              $9_1 = $6_1 + $8_1 | 0;
              if ($9_1 >>> 0 <= $6_1 >>> 0) {
               break label$1
              }
              if ($9_1 >>> 0 > $4_1 >>> 0) {
               break label$1
              }
             }
             if ((HEAPU8[(0 + 5168 | 0) >> 0] | 0) & 4 | 0) {
              break label$6
             }
             label$51 : {
              label$52 : {
               label$53 : {
                $4_1 = HEAP32[(0 + 4748 | 0) >> 2] | 0;
                if (!$4_1) {
                 break label$53
                }
                $0_1 = 5172;
                label$54 : while (1) {
                 label$55 : {
                  $6_1 = HEAP32[$0_1 >> 2] | 0;
                  if ($6_1 >>> 0 > $4_1 >>> 0) {
                   break label$55
                  }
                  if (($6_1 + (HEAP32[($0_1 + 4 | 0) >> 2] | 0) | 0) >>> 0 > $4_1 >>> 0) {
                   break label$52
                  }
                 }
                 $0_1 = HEAP32[($0_1 + 8 | 0) >> 2] | 0;
                 if ($0_1) {
                  continue label$54
                 }
                 break label$54;
                };
               }
               $5_1 = $373(0 | 0) | 0;
               if (($5_1 | 0) == (-1 | 0)) {
                break label$7
               }
               $2_1 = $8_1;
               label$56 : {
                $0_1 = HEAP32[(0 + 5200 | 0) >> 2] | 0;
                $4_1 = $0_1 + -1 | 0;
                if (!($4_1 & $5_1 | 0)) {
                 break label$56
                }
                $2_1 = ($8_1 - $5_1 | 0) + (($4_1 + $5_1 | 0) & (0 - $0_1 | 0) | 0) | 0;
               }
               if ($2_1 >>> 0 <= $3_1 >>> 0) {
                break label$7
               }
               if ($2_1 >>> 0 > 2147483646 >>> 0) {
                break label$7
               }
               label$57 : {
                $0_1 = HEAP32[(0 + 5164 | 0) >> 2] | 0;
                if (!$0_1) {
                 break label$57
                }
                $4_1 = HEAP32[(0 + 5156 | 0) >> 2] | 0;
                $6_1 = $4_1 + $2_1 | 0;
                if ($6_1 >>> 0 <= $4_1 >>> 0) {
                 break label$7
                }
                if ($6_1 >>> 0 > $0_1 >>> 0) {
                 break label$7
                }
               }
               $0_1 = $373($2_1 | 0) | 0;
               if (($0_1 | 0) != ($5_1 | 0)) {
                break label$51
               }
               break label$5;
              }
              $2_1 = ($2_1 - $5_1 | 0) & $12_1 | 0;
              if ($2_1 >>> 0 > 2147483646 >>> 0) {
               break label$7
              }
              $5_1 = $373($2_1 | 0) | 0;
              if (($5_1 | 0) == ((HEAP32[$0_1 >> 2] | 0) + (HEAP32[($0_1 + 4 | 0) >> 2] | 0) | 0 | 0)) {
               break label$8
              }
              $0_1 = $5_1;
             }
             label$58 : {
              if (($3_1 + 48 | 0) >>> 0 <= $2_1 >>> 0) {
               break label$58
              }
              if (($0_1 | 0) == (-1 | 0)) {
               break label$58
              }
              label$59 : {
               $4_1 = HEAP32[(0 + 5204 | 0) >> 2] | 0;
               $4_1 = (($7_1 - $2_1 | 0) + $4_1 | 0) & (0 - $4_1 | 0) | 0;
               if ($4_1 >>> 0 <= 2147483646 >>> 0) {
                break label$59
               }
               $5_1 = $0_1;
               break label$5;
              }
              label$60 : {
               if (($373($4_1 | 0) | 0 | 0) == (-1 | 0)) {
                break label$60
               }
               $2_1 = $4_1 + $2_1 | 0;
               $5_1 = $0_1;
               break label$5;
              }
              $373(0 - $2_1 | 0 | 0) | 0;
              break label$7;
             }
             $5_1 = $0_1;
             if (($0_1 | 0) != (-1 | 0)) {
              break label$5
             }
             break label$7;
            }
            abort();
           }
           $8_1 = 0;
           break label$2;
          }
          $5_1 = 0;
          break label$3;
         }
         if (($5_1 | 0) != (-1 | 0)) {
          break label$5
         }
        }
        HEAP32[(0 + 5168 | 0) >> 2] = HEAP32[(0 + 5168 | 0) >> 2] | 0 | 4 | 0;
       }
       if ($8_1 >>> 0 > 2147483646 >>> 0) {
        break label$4
       }
       $5_1 = $373($8_1 | 0) | 0;
       $0_1 = $373(0 | 0) | 0;
       if ($5_1 >>> 0 >= $0_1 >>> 0) {
        break label$4
       }
       if (($5_1 | 0) == (-1 | 0)) {
        break label$4
       }
       if (($0_1 | 0) == (-1 | 0)) {
        break label$4
       }
       $2_1 = $0_1 - $5_1 | 0;
       if ($2_1 >>> 0 <= ($3_1 + 40 | 0) >>> 0) {
        break label$4
       }
      }
      $0_1 = (HEAP32[(0 + 5156 | 0) >> 2] | 0) + $2_1 | 0;
      HEAP32[(0 + 5156 | 0) >> 2] = $0_1;
      label$61 : {
       if ($0_1 >>> 0 <= (HEAP32[(0 + 5160 | 0) >> 2] | 0) >>> 0) {
        break label$61
       }
       HEAP32[(0 + 5160 | 0) >> 2] = $0_1;
      }
      label$62 : {
       label$63 : {
        label$64 : {
         label$65 : {
          $4_1 = HEAP32[(0 + 4748 | 0) >> 2] | 0;
          if (!$4_1) {
           break label$65
          }
          $0_1 = 5172;
          label$66 : while (1) {
           $6_1 = HEAP32[$0_1 >> 2] | 0;
           $8_1 = HEAP32[($0_1 + 4 | 0) >> 2] | 0;
           if (($5_1 | 0) == ($6_1 + $8_1 | 0 | 0)) {
            break label$64
           }
           $0_1 = HEAP32[($0_1 + 8 | 0) >> 2] | 0;
           if ($0_1) {
            continue label$66
           }
           break label$63;
          };
         }
         label$67 : {
          label$68 : {
           $0_1 = HEAP32[(0 + 4740 | 0) >> 2] | 0;
           if (!$0_1) {
            break label$68
           }
           if ($5_1 >>> 0 >= $0_1 >>> 0) {
            break label$67
           }
          }
          HEAP32[(0 + 4740 | 0) >> 2] = $5_1;
         }
         $0_1 = 0;
         HEAP32[(0 + 5176 | 0) >> 2] = $2_1;
         HEAP32[(0 + 5172 | 0) >> 2] = $5_1;
         HEAP32[(0 + 4756 | 0) >> 2] = -1;
         HEAP32[(0 + 4760 | 0) >> 2] = HEAP32[(0 + 5196 | 0) >> 2] | 0;
         HEAP32[(0 + 5184 | 0) >> 2] = 0;
         label$69 : while (1) {
          $4_1 = $0_1 << 3 | 0;
          $6_1 = $4_1 + 4764 | 0;
          HEAP32[($4_1 + 4772 | 0) >> 2] = $6_1;
          HEAP32[($4_1 + 4776 | 0) >> 2] = $6_1;
          $0_1 = $0_1 + 1 | 0;
          if (($0_1 | 0) != (32 | 0)) {
           continue label$69
          }
          break label$69;
         };
         $0_1 = $2_1 + -40 | 0;
         $4_1 = ($5_1 + 8 | 0) & 7 | 0 ? (-8 - $5_1 | 0) & 7 | 0 : 0;
         $6_1 = $0_1 - $4_1 | 0;
         HEAP32[(0 + 4736 | 0) >> 2] = $6_1;
         $4_1 = $5_1 + $4_1 | 0;
         HEAP32[(0 + 4748 | 0) >> 2] = $4_1;
         HEAP32[($4_1 + 4 | 0) >> 2] = $6_1 | 1 | 0;
         HEAP32[(($5_1 + $0_1 | 0) + 4 | 0) >> 2] = 40;
         HEAP32[(0 + 4752 | 0) >> 2] = HEAP32[(0 + 5212 | 0) >> 2] | 0;
         break label$62;
        }
        if ((HEAPU8[($0_1 + 12 | 0) >> 0] | 0) & 8 | 0) {
         break label$63
        }
        if ($5_1 >>> 0 <= $4_1 >>> 0) {
         break label$63
        }
        if ($6_1 >>> 0 > $4_1 >>> 0) {
         break label$63
        }
        HEAP32[($0_1 + 4 | 0) >> 2] = $8_1 + $2_1 | 0;
        $0_1 = ($4_1 + 8 | 0) & 7 | 0 ? (-8 - $4_1 | 0) & 7 | 0 : 0;
        $6_1 = $4_1 + $0_1 | 0;
        HEAP32[(0 + 4748 | 0) >> 2] = $6_1;
        $5_1 = (HEAP32[(0 + 4736 | 0) >> 2] | 0) + $2_1 | 0;
        $0_1 = $5_1 - $0_1 | 0;
        HEAP32[(0 + 4736 | 0) >> 2] = $0_1;
        HEAP32[($6_1 + 4 | 0) >> 2] = $0_1 | 1 | 0;
        HEAP32[(($4_1 + $5_1 | 0) + 4 | 0) >> 2] = 40;
        HEAP32[(0 + 4752 | 0) >> 2] = HEAP32[(0 + 5212 | 0) >> 2] | 0;
        break label$62;
       }
       label$70 : {
        $8_1 = HEAP32[(0 + 4740 | 0) >> 2] | 0;
        if ($5_1 >>> 0 >= $8_1 >>> 0) {
         break label$70
        }
        HEAP32[(0 + 4740 | 0) >> 2] = $5_1;
        $8_1 = $5_1;
       }
       $6_1 = $5_1 + $2_1 | 0;
       $0_1 = 5172;
       label$71 : {
        label$72 : {
         label$73 : {
          label$74 : {
           label$75 : {
            label$76 : {
             label$77 : {
              label$78 : while (1) {
               if ((HEAP32[$0_1 >> 2] | 0 | 0) == ($6_1 | 0)) {
                break label$77
               }
               $0_1 = HEAP32[($0_1 + 8 | 0) >> 2] | 0;
               if ($0_1) {
                continue label$78
               }
               break label$76;
              };
             }
             if (!((HEAPU8[($0_1 + 12 | 0) >> 0] | 0) & 8 | 0)) {
              break label$75
             }
            }
            $0_1 = 5172;
            label$79 : while (1) {
             label$80 : {
              $6_1 = HEAP32[$0_1 >> 2] | 0;
              if ($6_1 >>> 0 > $4_1 >>> 0) {
               break label$80
              }
              $6_1 = $6_1 + (HEAP32[($0_1 + 4 | 0) >> 2] | 0) | 0;
              if ($6_1 >>> 0 > $4_1 >>> 0) {
               break label$74
              }
             }
             $0_1 = HEAP32[($0_1 + 8 | 0) >> 2] | 0;
             continue label$79;
            };
           }
           HEAP32[$0_1 >> 2] = $5_1;
           HEAP32[($0_1 + 4 | 0) >> 2] = (HEAP32[($0_1 + 4 | 0) >> 2] | 0) + $2_1 | 0;
           $12_1 = $5_1 + (($5_1 + 8 | 0) & 7 | 0 ? (-8 - $5_1 | 0) & 7 | 0 : 0) | 0;
           HEAP32[($12_1 + 4 | 0) >> 2] = $3_1 | 3 | 0;
           $5_1 = $6_1 + (($6_1 + 8 | 0) & 7 | 0 ? (-8 - $6_1 | 0) & 7 | 0 : 0) | 0;
           $0_1 = ($5_1 - $12_1 | 0) - $3_1 | 0;
           $6_1 = $12_1 + $3_1 | 0;
           label$81 : {
            if (($4_1 | 0) != ($5_1 | 0)) {
             break label$81
            }
            HEAP32[(0 + 4748 | 0) >> 2] = $6_1;
            $0_1 = (HEAP32[(0 + 4736 | 0) >> 2] | 0) + $0_1 | 0;
            HEAP32[(0 + 4736 | 0) >> 2] = $0_1;
            HEAP32[($6_1 + 4 | 0) >> 2] = $0_1 | 1 | 0;
            break label$72;
           }
           label$82 : {
            if ((HEAP32[(0 + 4744 | 0) >> 2] | 0 | 0) != ($5_1 | 0)) {
             break label$82
            }
            HEAP32[(0 + 4744 | 0) >> 2] = $6_1;
            $0_1 = (HEAP32[(0 + 4732 | 0) >> 2] | 0) + $0_1 | 0;
            HEAP32[(0 + 4732 | 0) >> 2] = $0_1;
            HEAP32[($6_1 + 4 | 0) >> 2] = $0_1 | 1 | 0;
            HEAP32[($6_1 + $0_1 | 0) >> 2] = $0_1;
            break label$72;
           }
           label$83 : {
            $4_1 = HEAP32[($5_1 + 4 | 0) >> 2] | 0;
            if (($4_1 & 3 | 0 | 0) != (1 | 0)) {
             break label$83
            }
            $7_1 = $4_1 & -8 | 0;
            label$84 : {
             label$85 : {
              if ($4_1 >>> 0 > 255 >>> 0) {
               break label$85
              }
              $3_1 = HEAP32[($5_1 + 12 | 0) >> 2] | 0;
              label$86 : {
               $2_1 = HEAP32[($5_1 + 8 | 0) >> 2] | 0;
               $9_1 = $4_1 >>> 3 | 0;
               $4_1 = ($9_1 << 3 | 0) + 4764 | 0;
               if (($2_1 | 0) == ($4_1 | 0)) {
                break label$86
               }
              }
              label$87 : {
               if (($3_1 | 0) != ($2_1 | 0)) {
                break label$87
               }
               HEAP32[(0 + 4724 | 0) >> 2] = (HEAP32[(0 + 4724 | 0) >> 2] | 0) & (__wasm_rotl_i32(-2 | 0, $9_1 | 0) | 0) | 0;
               break label$84;
              }
              label$88 : {
               if (($3_1 | 0) == ($4_1 | 0)) {
                break label$88
               }
              }
              HEAP32[($2_1 + 12 | 0) >> 2] = $3_1;
              HEAP32[($3_1 + 8 | 0) >> 2] = $2_1;
              break label$84;
             }
             $9_1 = HEAP32[($5_1 + 24 | 0) >> 2] | 0;
             label$89 : {
              label$90 : {
               $2_1 = HEAP32[($5_1 + 12 | 0) >> 2] | 0;
               if (($2_1 | 0) == ($5_1 | 0)) {
                break label$90
               }
               label$91 : {
                $4_1 = HEAP32[($5_1 + 8 | 0) >> 2] | 0;
                if ($8_1 >>> 0 > $4_1 >>> 0) {
                 break label$91
                }
                HEAP32[($4_1 + 12 | 0) >> 2] | 0;
               }
               HEAP32[($4_1 + 12 | 0) >> 2] = $2_1;
               HEAP32[($2_1 + 8 | 0) >> 2] = $4_1;
               break label$89;
              }
              label$92 : {
               $4_1 = $5_1 + 20 | 0;
               $3_1 = HEAP32[$4_1 >> 2] | 0;
               if ($3_1) {
                break label$92
               }
               $4_1 = $5_1 + 16 | 0;
               $3_1 = HEAP32[$4_1 >> 2] | 0;
               if ($3_1) {
                break label$92
               }
               $2_1 = 0;
               break label$89;
              }
              label$93 : while (1) {
               $8_1 = $4_1;
               $2_1 = $3_1;
               $4_1 = $3_1 + 20 | 0;
               $3_1 = HEAP32[$4_1 >> 2] | 0;
               if ($3_1) {
                continue label$93
               }
               $4_1 = $2_1 + 16 | 0;
               $3_1 = HEAP32[($2_1 + 16 | 0) >> 2] | 0;
               if ($3_1) {
                continue label$93
               }
               break label$93;
              };
              HEAP32[$8_1 >> 2] = 0;
             }
             if (!$9_1) {
              break label$84
             }
             label$94 : {
              label$95 : {
               $3_1 = HEAP32[($5_1 + 28 | 0) >> 2] | 0;
               $4_1 = ($3_1 << 2 | 0) + 5028 | 0;
               if ((HEAP32[$4_1 >> 2] | 0 | 0) != ($5_1 | 0)) {
                break label$95
               }
               HEAP32[$4_1 >> 2] = $2_1;
               if ($2_1) {
                break label$94
               }
               HEAP32[(0 + 4728 | 0) >> 2] = (HEAP32[(0 + 4728 | 0) >> 2] | 0) & (__wasm_rotl_i32(-2 | 0, $3_1 | 0) | 0) | 0;
               break label$84;
              }
              HEAP32[($9_1 + ((HEAP32[($9_1 + 16 | 0) >> 2] | 0 | 0) == ($5_1 | 0) ? 16 : 20) | 0) >> 2] = $2_1;
              if (!$2_1) {
               break label$84
              }
             }
             HEAP32[($2_1 + 24 | 0) >> 2] = $9_1;
             label$96 : {
              $4_1 = HEAP32[($5_1 + 16 | 0) >> 2] | 0;
              if (!$4_1) {
               break label$96
              }
              HEAP32[($2_1 + 16 | 0) >> 2] = $4_1;
              HEAP32[($4_1 + 24 | 0) >> 2] = $2_1;
             }
             $4_1 = HEAP32[($5_1 + 20 | 0) >> 2] | 0;
             if (!$4_1) {
              break label$84
             }
             HEAP32[($2_1 + 20 | 0) >> 2] = $4_1;
             HEAP32[($4_1 + 24 | 0) >> 2] = $2_1;
            }
            $0_1 = $7_1 + $0_1 | 0;
            $5_1 = $5_1 + $7_1 | 0;
           }
           HEAP32[($5_1 + 4 | 0) >> 2] = (HEAP32[($5_1 + 4 | 0) >> 2] | 0) & -2 | 0;
           HEAP32[($6_1 + 4 | 0) >> 2] = $0_1 | 1 | 0;
           HEAP32[($6_1 + $0_1 | 0) >> 2] = $0_1;
           label$97 : {
            if ($0_1 >>> 0 > 255 >>> 0) {
             break label$97
            }
            $4_1 = $0_1 >>> 3 | 0;
            $0_1 = ($4_1 << 3 | 0) + 4764 | 0;
            label$98 : {
             label$99 : {
              $3_1 = HEAP32[(0 + 4724 | 0) >> 2] | 0;
              $4_1 = 1 << $4_1 | 0;
              if ($3_1 & $4_1 | 0) {
               break label$99
              }
              HEAP32[(0 + 4724 | 0) >> 2] = $3_1 | $4_1 | 0;
              $4_1 = $0_1;
              break label$98;
             }
             $4_1 = HEAP32[($0_1 + 8 | 0) >> 2] | 0;
            }
            HEAP32[($0_1 + 8 | 0) >> 2] = $6_1;
            HEAP32[($4_1 + 12 | 0) >> 2] = $6_1;
            HEAP32[($6_1 + 12 | 0) >> 2] = $0_1;
            HEAP32[($6_1 + 8 | 0) >> 2] = $4_1;
            break label$72;
           }
           $4_1 = 0;
           label$100 : {
            $3_1 = $0_1 >>> 8 | 0;
            if (!$3_1) {
             break label$100
            }
            $4_1 = 31;
            if ($0_1 >>> 0 > 16777215 >>> 0) {
             break label$100
            }
            $4_1 = (($3_1 + 1048320 | 0) >>> 16 | 0) & 8 | 0;
            $3_1 = $3_1 << $4_1 | 0;
            $1205 = $3_1;
            $3_1 = (($3_1 + 520192 | 0) >>> 16 | 0) & 4 | 0;
            $5_1 = $1205 << $3_1 | 0;
            $1212 = $5_1;
            $5_1 = (($5_1 + 245760 | 0) >>> 16 | 0) & 2 | 0;
            $4_1 = (($1212 << $5_1 | 0) >>> 15 | 0) - ($3_1 | $4_1 | 0 | $5_1 | 0) | 0;
            $4_1 = ($4_1 << 1 | 0 | (($0_1 >>> ($4_1 + 21 | 0) | 0) & 1 | 0) | 0) + 28 | 0;
           }
           HEAP32[($6_1 + 28 | 0) >> 2] = $4_1;
           i64toi32_i32$1 = $6_1;
           i64toi32_i32$0 = 0;
           HEAP32[($6_1 + 16 | 0) >> 2] = 0;
           HEAP32[($6_1 + 20 | 0) >> 2] = i64toi32_i32$0;
           $3_1 = ($4_1 << 2 | 0) + 5028 | 0;
           label$101 : {
            label$102 : {
             $5_1 = HEAP32[(0 + 4728 | 0) >> 2] | 0;
             $8_1 = 1 << $4_1 | 0;
             if ($5_1 & $8_1 | 0) {
              break label$102
             }
             HEAP32[(0 + 4728 | 0) >> 2] = $5_1 | $8_1 | 0;
             HEAP32[$3_1 >> 2] = $6_1;
             HEAP32[($6_1 + 24 | 0) >> 2] = $3_1;
             break label$101;
            }
            $4_1 = $0_1 << (($4_1 | 0) == (31 | 0) ? 0 : 25 - ($4_1 >>> 1 | 0) | 0) | 0;
            $5_1 = HEAP32[$3_1 >> 2] | 0;
            label$103 : while (1) {
             $3_1 = $5_1;
             if (((HEAP32[($5_1 + 4 | 0) >> 2] | 0) & -8 | 0 | 0) == ($0_1 | 0)) {
              break label$73
             }
             $5_1 = $4_1 >>> 29 | 0;
             $4_1 = $4_1 << 1 | 0;
             $8_1 = ($3_1 + ($5_1 & 4 | 0) | 0) + 16 | 0;
             $5_1 = HEAP32[$8_1 >> 2] | 0;
             if ($5_1) {
              continue label$103
             }
             break label$103;
            };
            HEAP32[$8_1 >> 2] = $6_1;
            HEAP32[($6_1 + 24 | 0) >> 2] = $3_1;
           }
           HEAP32[($6_1 + 12 | 0) >> 2] = $6_1;
           HEAP32[($6_1 + 8 | 0) >> 2] = $6_1;
           break label$72;
          }
          $0_1 = $2_1 + -40 | 0;
          $8_1 = ($5_1 + 8 | 0) & 7 | 0 ? (-8 - $5_1 | 0) & 7 | 0 : 0;
          $12_1 = $0_1 - $8_1 | 0;
          HEAP32[(0 + 4736 | 0) >> 2] = $12_1;
          $8_1 = $5_1 + $8_1 | 0;
          HEAP32[(0 + 4748 | 0) >> 2] = $8_1;
          HEAP32[($8_1 + 4 | 0) >> 2] = $12_1 | 1 | 0;
          HEAP32[(($5_1 + $0_1 | 0) + 4 | 0) >> 2] = 40;
          HEAP32[(0 + 4752 | 0) >> 2] = HEAP32[(0 + 5212 | 0) >> 2] | 0;
          $0_1 = ($6_1 + (($6_1 + -39 | 0) & 7 | 0 ? (39 - $6_1 | 0) & 7 | 0 : 0) | 0) + -47 | 0;
          $8_1 = $0_1 >>> 0 < ($4_1 + 16 | 0) >>> 0 ? $4_1 : $0_1;
          HEAP32[($8_1 + 4 | 0) >> 2] = 27;
          i64toi32_i32$2 = 0;
          i64toi32_i32$0 = HEAP32[(i64toi32_i32$2 + 5180 | 0) >> 2] | 0;
          i64toi32_i32$1 = HEAP32[(i64toi32_i32$2 + 5184 | 0) >> 2] | 0;
          $1334 = i64toi32_i32$0;
          i64toi32_i32$0 = $8_1 + 16 | 0;
          HEAP32[i64toi32_i32$0 >> 2] = $1334;
          HEAP32[(i64toi32_i32$0 + 4 | 0) >> 2] = i64toi32_i32$1;
          i64toi32_i32$2 = 0;
          i64toi32_i32$1 = HEAP32[(i64toi32_i32$2 + 5172 | 0) >> 2] | 0;
          i64toi32_i32$0 = HEAP32[(i64toi32_i32$2 + 5176 | 0) >> 2] | 0;
          $1336 = i64toi32_i32$1;
          i64toi32_i32$1 = $8_1;
          HEAP32[($8_1 + 8 | 0) >> 2] = $1336;
          HEAP32[($8_1 + 12 | 0) >> 2] = i64toi32_i32$0;
          HEAP32[(0 + 5180 | 0) >> 2] = $8_1 + 8 | 0;
          HEAP32[(0 + 5176 | 0) >> 2] = $2_1;
          HEAP32[(0 + 5172 | 0) >> 2] = $5_1;
          HEAP32[(0 + 5184 | 0) >> 2] = 0;
          $0_1 = $8_1 + 24 | 0;
          label$104 : while (1) {
           HEAP32[($0_1 + 4 | 0) >> 2] = 7;
           $5_1 = $0_1 + 8 | 0;
           $0_1 = $0_1 + 4 | 0;
           if ($6_1 >>> 0 > $5_1 >>> 0) {
            continue label$104
           }
           break label$104;
          };
          if (($8_1 | 0) == ($4_1 | 0)) {
           break label$62
          }
          HEAP32[($8_1 + 4 | 0) >> 2] = (HEAP32[($8_1 + 4 | 0) >> 2] | 0) & -2 | 0;
          $2_1 = $8_1 - $4_1 | 0;
          HEAP32[($4_1 + 4 | 0) >> 2] = $2_1 | 1 | 0;
          HEAP32[$8_1 >> 2] = $2_1;
          label$105 : {
           if ($2_1 >>> 0 > 255 >>> 0) {
            break label$105
           }
           $6_1 = $2_1 >>> 3 | 0;
           $0_1 = ($6_1 << 3 | 0) + 4764 | 0;
           label$106 : {
            label$107 : {
             $5_1 = HEAP32[(0 + 4724 | 0) >> 2] | 0;
             $6_1 = 1 << $6_1 | 0;
             if ($5_1 & $6_1 | 0) {
              break label$107
             }
             HEAP32[(0 + 4724 | 0) >> 2] = $5_1 | $6_1 | 0;
             $6_1 = $0_1;
             break label$106;
            }
            $6_1 = HEAP32[($0_1 + 8 | 0) >> 2] | 0;
           }
           HEAP32[($0_1 + 8 | 0) >> 2] = $4_1;
           HEAP32[($6_1 + 12 | 0) >> 2] = $4_1;
           HEAP32[($4_1 + 12 | 0) >> 2] = $0_1;
           HEAP32[($4_1 + 8 | 0) >> 2] = $6_1;
           break label$62;
          }
          $0_1 = 0;
          label$108 : {
           $6_1 = $2_1 >>> 8 | 0;
           if (!$6_1) {
            break label$108
           }
           $0_1 = 31;
           if ($2_1 >>> 0 > 16777215 >>> 0) {
            break label$108
           }
           $0_1 = (($6_1 + 1048320 | 0) >>> 16 | 0) & 8 | 0;
           $6_1 = $6_1 << $0_1 | 0;
           $1406 = $6_1;
           $6_1 = (($6_1 + 520192 | 0) >>> 16 | 0) & 4 | 0;
           $5_1 = $1406 << $6_1 | 0;
           $1413 = $5_1;
           $5_1 = (($5_1 + 245760 | 0) >>> 16 | 0) & 2 | 0;
           $0_1 = (($1413 << $5_1 | 0) >>> 15 | 0) - ($6_1 | $0_1 | 0 | $5_1 | 0) | 0;
           $0_1 = ($0_1 << 1 | 0 | (($2_1 >>> ($0_1 + 21 | 0) | 0) & 1 | 0) | 0) + 28 | 0;
          }
          i64toi32_i32$1 = $4_1;
          i64toi32_i32$0 = 0;
          HEAP32[($4_1 + 16 | 0) >> 2] = 0;
          HEAP32[($4_1 + 20 | 0) >> 2] = i64toi32_i32$0;
          HEAP32[($4_1 + 28 | 0) >> 2] = $0_1;
          $6_1 = ($0_1 << 2 | 0) + 5028 | 0;
          label$109 : {
           label$110 : {
            $5_1 = HEAP32[(0 + 4728 | 0) >> 2] | 0;
            $8_1 = 1 << $0_1 | 0;
            if ($5_1 & $8_1 | 0) {
             break label$110
            }
            HEAP32[(0 + 4728 | 0) >> 2] = $5_1 | $8_1 | 0;
            HEAP32[$6_1 >> 2] = $4_1;
            HEAP32[($4_1 + 24 | 0) >> 2] = $6_1;
            break label$109;
           }
           $0_1 = $2_1 << (($0_1 | 0) == (31 | 0) ? 0 : 25 - ($0_1 >>> 1 | 0) | 0) | 0;
           $5_1 = HEAP32[$6_1 >> 2] | 0;
           label$111 : while (1) {
            $6_1 = $5_1;
            if (((HEAP32[($6_1 + 4 | 0) >> 2] | 0) & -8 | 0 | 0) == ($2_1 | 0)) {
             break label$71
            }
            $5_1 = $0_1 >>> 29 | 0;
            $0_1 = $0_1 << 1 | 0;
            $8_1 = ($6_1 + ($5_1 & 4 | 0) | 0) + 16 | 0;
            $5_1 = HEAP32[$8_1 >> 2] | 0;
            if ($5_1) {
             continue label$111
            }
            break label$111;
           };
           HEAP32[$8_1 >> 2] = $4_1;
           HEAP32[($4_1 + 24 | 0) >> 2] = $6_1;
          }
          HEAP32[($4_1 + 12 | 0) >> 2] = $4_1;
          HEAP32[($4_1 + 8 | 0) >> 2] = $4_1;
          break label$62;
         }
         $0_1 = HEAP32[($3_1 + 8 | 0) >> 2] | 0;
         HEAP32[($0_1 + 12 | 0) >> 2] = $6_1;
         HEAP32[($3_1 + 8 | 0) >> 2] = $6_1;
         HEAP32[($6_1 + 24 | 0) >> 2] = 0;
         HEAP32[($6_1 + 12 | 0) >> 2] = $3_1;
         HEAP32[($6_1 + 8 | 0) >> 2] = $0_1;
        }
        $0_1 = $12_1 + 8 | 0;
        break label$1;
       }
       $0_1 = HEAP32[($6_1 + 8 | 0) >> 2] | 0;
       HEAP32[($0_1 + 12 | 0) >> 2] = $4_1;
       HEAP32[($6_1 + 8 | 0) >> 2] = $4_1;
       HEAP32[($4_1 + 24 | 0) >> 2] = 0;
       HEAP32[($4_1 + 12 | 0) >> 2] = $6_1;
       HEAP32[($4_1 + 8 | 0) >> 2] = $0_1;
      }
      $0_1 = HEAP32[(0 + 4736 | 0) >> 2] | 0;
      if ($0_1 >>> 0 <= $3_1 >>> 0) {
       break label$4
      }
      $4_1 = $0_1 - $3_1 | 0;
      HEAP32[(0 + 4736 | 0) >> 2] = $4_1;
      $0_1 = HEAP32[(0 + 4748 | 0) >> 2] | 0;
      $6_1 = $0_1 + $3_1 | 0;
      HEAP32[(0 + 4748 | 0) >> 2] = $6_1;
      HEAP32[($6_1 + 4 | 0) >> 2] = $4_1 | 1 | 0;
      HEAP32[($0_1 + 4 | 0) >> 2] = $3_1 | 3 | 0;
      $0_1 = $0_1 + 8 | 0;
      break label$1;
     }
     HEAP32[($370() | 0) >> 2] = 48;
     $0_1 = 0;
     break label$1;
    }
    label$112 : {
     if (!$9_1) {
      break label$112
     }
     label$113 : {
      label$114 : {
       $6_1 = HEAP32[($8_1 + 28 | 0) >> 2] | 0;
       $0_1 = ($6_1 << 2 | 0) + 5028 | 0;
       if (($8_1 | 0) != (HEAP32[$0_1 >> 2] | 0 | 0)) {
        break label$114
       }
       HEAP32[$0_1 >> 2] = $5_1;
       if ($5_1) {
        break label$113
       }
       $7_1 = $7_1 & (__wasm_rotl_i32(-2 | 0, $6_1 | 0) | 0) | 0;
       HEAP32[(0 + 4728 | 0) >> 2] = $7_1;
       break label$112;
      }
      HEAP32[($9_1 + ((HEAP32[($9_1 + 16 | 0) >> 2] | 0 | 0) == ($8_1 | 0) ? 16 : 20) | 0) >> 2] = $5_1;
      if (!$5_1) {
       break label$112
      }
     }
     HEAP32[($5_1 + 24 | 0) >> 2] = $9_1;
     label$115 : {
      $0_1 = HEAP32[($8_1 + 16 | 0) >> 2] | 0;
      if (!$0_1) {
       break label$115
      }
      HEAP32[($5_1 + 16 | 0) >> 2] = $0_1;
      HEAP32[($0_1 + 24 | 0) >> 2] = $5_1;
     }
     $0_1 = HEAP32[($8_1 + 20 | 0) >> 2] | 0;
     if (!$0_1) {
      break label$112
     }
     HEAP32[($5_1 + 20 | 0) >> 2] = $0_1;
     HEAP32[($0_1 + 24 | 0) >> 2] = $5_1;
    }
    label$116 : {
     label$117 : {
      if ($4_1 >>> 0 > 15 >>> 0) {
       break label$117
      }
      $0_1 = $4_1 + $3_1 | 0;
      HEAP32[($8_1 + 4 | 0) >> 2] = $0_1 | 3 | 0;
      $0_1 = $8_1 + $0_1 | 0;
      HEAP32[($0_1 + 4 | 0) >> 2] = HEAP32[($0_1 + 4 | 0) >> 2] | 0 | 1 | 0;
      break label$116;
     }
     HEAP32[($8_1 + 4 | 0) >> 2] = $3_1 | 3 | 0;
     HEAP32[($12_1 + 4 | 0) >> 2] = $4_1 | 1 | 0;
     HEAP32[($12_1 + $4_1 | 0) >> 2] = $4_1;
     label$118 : {
      if ($4_1 >>> 0 > 255 >>> 0) {
       break label$118
      }
      $4_1 = $4_1 >>> 3 | 0;
      $0_1 = ($4_1 << 3 | 0) + 4764 | 0;
      label$119 : {
       label$120 : {
        $6_1 = HEAP32[(0 + 4724 | 0) >> 2] | 0;
        $4_1 = 1 << $4_1 | 0;
        if ($6_1 & $4_1 | 0) {
         break label$120
        }
        HEAP32[(0 + 4724 | 0) >> 2] = $6_1 | $4_1 | 0;
        $4_1 = $0_1;
        break label$119;
       }
       $4_1 = HEAP32[($0_1 + 8 | 0) >> 2] | 0;
      }
      HEAP32[($0_1 + 8 | 0) >> 2] = $12_1;
      HEAP32[($4_1 + 12 | 0) >> 2] = $12_1;
      HEAP32[($12_1 + 12 | 0) >> 2] = $0_1;
      HEAP32[($12_1 + 8 | 0) >> 2] = $4_1;
      break label$116;
     }
     label$121 : {
      label$122 : {
       $6_1 = $4_1 >>> 8 | 0;
       if ($6_1) {
        break label$122
       }
       $0_1 = 0;
       break label$121;
      }
      $0_1 = 31;
      if ($4_1 >>> 0 > 16777215 >>> 0) {
       break label$121
      }
      $0_1 = (($6_1 + 1048320 | 0) >>> 16 | 0) & 8 | 0;
      $6_1 = $6_1 << $0_1 | 0;
      $1654 = $6_1;
      $6_1 = (($6_1 + 520192 | 0) >>> 16 | 0) & 4 | 0;
      $3_1 = $1654 << $6_1 | 0;
      $1661 = $3_1;
      $3_1 = (($3_1 + 245760 | 0) >>> 16 | 0) & 2 | 0;
      $0_1 = (($1661 << $3_1 | 0) >>> 15 | 0) - ($6_1 | $0_1 | 0 | $3_1 | 0) | 0;
      $0_1 = ($0_1 << 1 | 0 | (($4_1 >>> ($0_1 + 21 | 0) | 0) & 1 | 0) | 0) + 28 | 0;
     }
     HEAP32[($12_1 + 28 | 0) >> 2] = $0_1;
     i64toi32_i32$1 = $12_1;
     i64toi32_i32$0 = 0;
     HEAP32[($12_1 + 16 | 0) >> 2] = 0;
     HEAP32[($12_1 + 20 | 0) >> 2] = i64toi32_i32$0;
     $6_1 = ($0_1 << 2 | 0) + 5028 | 0;
     label$123 : {
      label$124 : {
       label$125 : {
        $3_1 = 1 << $0_1 | 0;
        if ($7_1 & $3_1 | 0) {
         break label$125
        }
        HEAP32[(0 + 4728 | 0) >> 2] = $7_1 | $3_1 | 0;
        HEAP32[$6_1 >> 2] = $12_1;
        HEAP32[($12_1 + 24 | 0) >> 2] = $6_1;
        break label$124;
       }
       $0_1 = $4_1 << (($0_1 | 0) == (31 | 0) ? 0 : 25 - ($0_1 >>> 1 | 0) | 0) | 0;
       $3_1 = HEAP32[$6_1 >> 2] | 0;
       label$126 : while (1) {
        $6_1 = $3_1;
        if (((HEAP32[($6_1 + 4 | 0) >> 2] | 0) & -8 | 0 | 0) == ($4_1 | 0)) {
         break label$123
        }
        $3_1 = $0_1 >>> 29 | 0;
        $0_1 = $0_1 << 1 | 0;
        $5_1 = ($6_1 + ($3_1 & 4 | 0) | 0) + 16 | 0;
        $3_1 = HEAP32[$5_1 >> 2] | 0;
        if ($3_1) {
         continue label$126
        }
        break label$126;
       };
       HEAP32[$5_1 >> 2] = $12_1;
       HEAP32[($12_1 + 24 | 0) >> 2] = $6_1;
      }
      HEAP32[($12_1 + 12 | 0) >> 2] = $12_1;
      HEAP32[($12_1 + 8 | 0) >> 2] = $12_1;
      break label$116;
     }
     $0_1 = HEAP32[($6_1 + 8 | 0) >> 2] | 0;
     HEAP32[($0_1 + 12 | 0) >> 2] = $12_1;
     HEAP32[($6_1 + 8 | 0) >> 2] = $12_1;
     HEAP32[($12_1 + 24 | 0) >> 2] = 0;
     HEAP32[($12_1 + 12 | 0) >> 2] = $6_1;
     HEAP32[($12_1 + 8 | 0) >> 2] = $0_1;
    }
    $0_1 = $8_1 + 8 | 0;
    break label$1;
   }
   label$127 : {
    if (!$11_1) {
     break label$127
    }
    label$128 : {
     label$129 : {
      $6_1 = HEAP32[($5_1 + 28 | 0) >> 2] | 0;
      $0_1 = ($6_1 << 2 | 0) + 5028 | 0;
      if (($5_1 | 0) != (HEAP32[$0_1 >> 2] | 0 | 0)) {
       break label$129
      }
      HEAP32[$0_1 >> 2] = $8_1;
      if ($8_1) {
       break label$128
      }
      HEAP32[(0 + 4728 | 0) >> 2] = $9_1 & (__wasm_rotl_i32(-2 | 0, $6_1 | 0) | 0) | 0;
      break label$127;
     }
     HEAP32[($11_1 + ((HEAP32[($11_1 + 16 | 0) >> 2] | 0 | 0) == ($5_1 | 0) ? 16 : 20) | 0) >> 2] = $8_1;
     if (!$8_1) {
      break label$127
     }
    }
    HEAP32[($8_1 + 24 | 0) >> 2] = $11_1;
    label$130 : {
     $0_1 = HEAP32[($5_1 + 16 | 0) >> 2] | 0;
     if (!$0_1) {
      break label$130
     }
     HEAP32[($8_1 + 16 | 0) >> 2] = $0_1;
     HEAP32[($0_1 + 24 | 0) >> 2] = $8_1;
    }
    $0_1 = HEAP32[($5_1 + 20 | 0) >> 2] | 0;
    if (!$0_1) {
     break label$127
    }
    HEAP32[($8_1 + 20 | 0) >> 2] = $0_1;
    HEAP32[($0_1 + 24 | 0) >> 2] = $8_1;
   }
   label$131 : {
    label$132 : {
     if ($4_1 >>> 0 > 15 >>> 0) {
      break label$132
     }
     $0_1 = $4_1 + $3_1 | 0;
     HEAP32[($5_1 + 4 | 0) >> 2] = $0_1 | 3 | 0;
     $0_1 = $5_1 + $0_1 | 0;
     HEAP32[($0_1 + 4 | 0) >> 2] = HEAP32[($0_1 + 4 | 0) >> 2] | 0 | 1 | 0;
     break label$131;
    }
    HEAP32[($5_1 + 4 | 0) >> 2] = $3_1 | 3 | 0;
    HEAP32[($10_1 + 4 | 0) >> 2] = $4_1 | 1 | 0;
    HEAP32[($10_1 + $4_1 | 0) >> 2] = $4_1;
    label$133 : {
     if (!$7_1) {
      break label$133
     }
     $3_1 = $7_1 >>> 3 | 0;
     $6_1 = ($3_1 << 3 | 0) + 4764 | 0;
     $0_1 = HEAP32[(0 + 4744 | 0) >> 2] | 0;
     label$134 : {
      label$135 : {
       $3_1 = 1 << $3_1 | 0;
       if ($3_1 & $2_1 | 0) {
        break label$135
       }
       HEAP32[(0 + 4724 | 0) >> 2] = $3_1 | $2_1 | 0;
       $3_1 = $6_1;
       break label$134;
      }
      $3_1 = HEAP32[($6_1 + 8 | 0) >> 2] | 0;
     }
     HEAP32[($6_1 + 8 | 0) >> 2] = $0_1;
     HEAP32[($3_1 + 12 | 0) >> 2] = $0_1;
     HEAP32[($0_1 + 12 | 0) >> 2] = $6_1;
     HEAP32[($0_1 + 8 | 0) >> 2] = $3_1;
    }
    HEAP32[(0 + 4744 | 0) >> 2] = $10_1;
    HEAP32[(0 + 4732 | 0) >> 2] = $4_1;
   }
   $0_1 = $5_1 + 8 | 0;
  }
  global$0 = $1_1 + 16 | 0;
  return $0_1 | 0;
 }
 
 function $372($0_1) {
  $0_1 = $0_1 | 0;
  var $2_1 = 0, $5_1 = 0, $1_1 = 0, $4_1 = 0, $3_1 = 0, $7_1 = 0, $6_1 = 0, $408 = 0, $415 = 0;
  label$1 : {
   if (!$0_1) {
    break label$1
   }
   $1_1 = $0_1 + -8 | 0;
   $2_1 = HEAP32[($0_1 + -4 | 0) >> 2] | 0;
   $0_1 = $2_1 & -8 | 0;
   $3_1 = $1_1 + $0_1 | 0;
   label$2 : {
    if ($2_1 & 1 | 0) {
     break label$2
    }
    if (!($2_1 & 3 | 0)) {
     break label$1
    }
    $2_1 = HEAP32[$1_1 >> 2] | 0;
    $1_1 = $1_1 - $2_1 | 0;
    $4_1 = HEAP32[(0 + 4740 | 0) >> 2] | 0;
    if ($1_1 >>> 0 < $4_1 >>> 0) {
     break label$1
    }
    $0_1 = $2_1 + $0_1 | 0;
    label$3 : {
     if ((HEAP32[(0 + 4744 | 0) >> 2] | 0 | 0) == ($1_1 | 0)) {
      break label$3
     }
     label$4 : {
      if ($2_1 >>> 0 > 255 >>> 0) {
       break label$4
      }
      $5_1 = HEAP32[($1_1 + 12 | 0) >> 2] | 0;
      label$5 : {
       $6_1 = HEAP32[($1_1 + 8 | 0) >> 2] | 0;
       $7_1 = $2_1 >>> 3 | 0;
       $2_1 = ($7_1 << 3 | 0) + 4764 | 0;
       if (($6_1 | 0) == ($2_1 | 0)) {
        break label$5
       }
      }
      label$6 : {
       if (($5_1 | 0) != ($6_1 | 0)) {
        break label$6
       }
       HEAP32[(0 + 4724 | 0) >> 2] = (HEAP32[(0 + 4724 | 0) >> 2] | 0) & (__wasm_rotl_i32(-2 | 0, $7_1 | 0) | 0) | 0;
       break label$2;
      }
      label$7 : {
       if (($5_1 | 0) == ($2_1 | 0)) {
        break label$7
       }
      }
      HEAP32[($6_1 + 12 | 0) >> 2] = $5_1;
      HEAP32[($5_1 + 8 | 0) >> 2] = $6_1;
      break label$2;
     }
     $7_1 = HEAP32[($1_1 + 24 | 0) >> 2] | 0;
     label$8 : {
      label$9 : {
       $5_1 = HEAP32[($1_1 + 12 | 0) >> 2] | 0;
       if (($5_1 | 0) == ($1_1 | 0)) {
        break label$9
       }
       label$10 : {
        $2_1 = HEAP32[($1_1 + 8 | 0) >> 2] | 0;
        if ($4_1 >>> 0 > $2_1 >>> 0) {
         break label$10
        }
        HEAP32[($2_1 + 12 | 0) >> 2] | 0;
       }
       HEAP32[($2_1 + 12 | 0) >> 2] = $5_1;
       HEAP32[($5_1 + 8 | 0) >> 2] = $2_1;
       break label$8;
      }
      label$11 : {
       $2_1 = $1_1 + 20 | 0;
       $4_1 = HEAP32[$2_1 >> 2] | 0;
       if ($4_1) {
        break label$11
       }
       $2_1 = $1_1 + 16 | 0;
       $4_1 = HEAP32[$2_1 >> 2] | 0;
       if ($4_1) {
        break label$11
       }
       $5_1 = 0;
       break label$8;
      }
      label$12 : while (1) {
       $6_1 = $2_1;
       $5_1 = $4_1;
       $2_1 = $5_1 + 20 | 0;
       $4_1 = HEAP32[$2_1 >> 2] | 0;
       if ($4_1) {
        continue label$12
       }
       $2_1 = $5_1 + 16 | 0;
       $4_1 = HEAP32[($5_1 + 16 | 0) >> 2] | 0;
       if ($4_1) {
        continue label$12
       }
       break label$12;
      };
      HEAP32[$6_1 >> 2] = 0;
     }
     if (!$7_1) {
      break label$2
     }
     label$13 : {
      label$14 : {
       $4_1 = HEAP32[($1_1 + 28 | 0) >> 2] | 0;
       $2_1 = ($4_1 << 2 | 0) + 5028 | 0;
       if ((HEAP32[$2_1 >> 2] | 0 | 0) != ($1_1 | 0)) {
        break label$14
       }
       HEAP32[$2_1 >> 2] = $5_1;
       if ($5_1) {
        break label$13
       }
       HEAP32[(0 + 4728 | 0) >> 2] = (HEAP32[(0 + 4728 | 0) >> 2] | 0) & (__wasm_rotl_i32(-2 | 0, $4_1 | 0) | 0) | 0;
       break label$2;
      }
      HEAP32[($7_1 + ((HEAP32[($7_1 + 16 | 0) >> 2] | 0 | 0) == ($1_1 | 0) ? 16 : 20) | 0) >> 2] = $5_1;
      if (!$5_1) {
       break label$2
      }
     }
     HEAP32[($5_1 + 24 | 0) >> 2] = $7_1;
     label$15 : {
      $2_1 = HEAP32[($1_1 + 16 | 0) >> 2] | 0;
      if (!$2_1) {
       break label$15
      }
      HEAP32[($5_1 + 16 | 0) >> 2] = $2_1;
      HEAP32[($2_1 + 24 | 0) >> 2] = $5_1;
     }
     $2_1 = HEAP32[($1_1 + 20 | 0) >> 2] | 0;
     if (!$2_1) {
      break label$2
     }
     HEAP32[($5_1 + 20 | 0) >> 2] = $2_1;
     HEAP32[($2_1 + 24 | 0) >> 2] = $5_1;
     break label$2;
    }
    $2_1 = HEAP32[($3_1 + 4 | 0) >> 2] | 0;
    if (($2_1 & 3 | 0 | 0) != (3 | 0)) {
     break label$2
    }
    HEAP32[(0 + 4732 | 0) >> 2] = $0_1;
    HEAP32[($3_1 + 4 | 0) >> 2] = $2_1 & -2 | 0;
    HEAP32[($1_1 + 4 | 0) >> 2] = $0_1 | 1 | 0;
    HEAP32[($1_1 + $0_1 | 0) >> 2] = $0_1;
    return;
   }
   if ($3_1 >>> 0 <= $1_1 >>> 0) {
    break label$1
   }
   $2_1 = HEAP32[($3_1 + 4 | 0) >> 2] | 0;
   if (!($2_1 & 1 | 0)) {
    break label$1
   }
   label$16 : {
    label$17 : {
     if ($2_1 & 2 | 0) {
      break label$17
     }
     label$18 : {
      if ((HEAP32[(0 + 4748 | 0) >> 2] | 0 | 0) != ($3_1 | 0)) {
       break label$18
      }
      HEAP32[(0 + 4748 | 0) >> 2] = $1_1;
      $0_1 = (HEAP32[(0 + 4736 | 0) >> 2] | 0) + $0_1 | 0;
      HEAP32[(0 + 4736 | 0) >> 2] = $0_1;
      HEAP32[($1_1 + 4 | 0) >> 2] = $0_1 | 1 | 0;
      if (($1_1 | 0) != (HEAP32[(0 + 4744 | 0) >> 2] | 0 | 0)) {
       break label$1
      }
      HEAP32[(0 + 4732 | 0) >> 2] = 0;
      HEAP32[(0 + 4744 | 0) >> 2] = 0;
      return;
     }
     label$19 : {
      if ((HEAP32[(0 + 4744 | 0) >> 2] | 0 | 0) != ($3_1 | 0)) {
       break label$19
      }
      HEAP32[(0 + 4744 | 0) >> 2] = $1_1;
      $0_1 = (HEAP32[(0 + 4732 | 0) >> 2] | 0) + $0_1 | 0;
      HEAP32[(0 + 4732 | 0) >> 2] = $0_1;
      HEAP32[($1_1 + 4 | 0) >> 2] = $0_1 | 1 | 0;
      HEAP32[($1_1 + $0_1 | 0) >> 2] = $0_1;
      return;
     }
     $0_1 = ($2_1 & -8 | 0) + $0_1 | 0;
     label$20 : {
      label$21 : {
       if ($2_1 >>> 0 > 255 >>> 0) {
        break label$21
       }
       $4_1 = HEAP32[($3_1 + 12 | 0) >> 2] | 0;
       label$22 : {
        $5_1 = HEAP32[($3_1 + 8 | 0) >> 2] | 0;
        $3_1 = $2_1 >>> 3 | 0;
        $2_1 = ($3_1 << 3 | 0) + 4764 | 0;
        if (($5_1 | 0) == ($2_1 | 0)) {
         break label$22
        }
        HEAP32[(0 + 4740 | 0) >> 2] | 0;
       }
       label$23 : {
        if (($4_1 | 0) != ($5_1 | 0)) {
         break label$23
        }
        HEAP32[(0 + 4724 | 0) >> 2] = (HEAP32[(0 + 4724 | 0) >> 2] | 0) & (__wasm_rotl_i32(-2 | 0, $3_1 | 0) | 0) | 0;
        break label$20;
       }
       label$24 : {
        if (($4_1 | 0) == ($2_1 | 0)) {
         break label$24
        }
        HEAP32[(0 + 4740 | 0) >> 2] | 0;
       }
       HEAP32[($5_1 + 12 | 0) >> 2] = $4_1;
       HEAP32[($4_1 + 8 | 0) >> 2] = $5_1;
       break label$20;
      }
      $7_1 = HEAP32[($3_1 + 24 | 0) >> 2] | 0;
      label$25 : {
       label$26 : {
        $5_1 = HEAP32[($3_1 + 12 | 0) >> 2] | 0;
        if (($5_1 | 0) == ($3_1 | 0)) {
         break label$26
        }
        label$27 : {
         $2_1 = HEAP32[($3_1 + 8 | 0) >> 2] | 0;
         if ((HEAP32[(0 + 4740 | 0) >> 2] | 0) >>> 0 > $2_1 >>> 0) {
          break label$27
         }
         HEAP32[($2_1 + 12 | 0) >> 2] | 0;
        }
        HEAP32[($2_1 + 12 | 0) >> 2] = $5_1;
        HEAP32[($5_1 + 8 | 0) >> 2] = $2_1;
        break label$25;
       }
       label$28 : {
        $2_1 = $3_1 + 20 | 0;
        $4_1 = HEAP32[$2_1 >> 2] | 0;
        if ($4_1) {
         break label$28
        }
        $2_1 = $3_1 + 16 | 0;
        $4_1 = HEAP32[$2_1 >> 2] | 0;
        if ($4_1) {
         break label$28
        }
        $5_1 = 0;
        break label$25;
       }
       label$29 : while (1) {
        $6_1 = $2_1;
        $5_1 = $4_1;
        $2_1 = $5_1 + 20 | 0;
        $4_1 = HEAP32[$2_1 >> 2] | 0;
        if ($4_1) {
         continue label$29
        }
        $2_1 = $5_1 + 16 | 0;
        $4_1 = HEAP32[($5_1 + 16 | 0) >> 2] | 0;
        if ($4_1) {
         continue label$29
        }
        break label$29;
       };
       HEAP32[$6_1 >> 2] = 0;
      }
      if (!$7_1) {
       break label$20
      }
      label$30 : {
       label$31 : {
        $4_1 = HEAP32[($3_1 + 28 | 0) >> 2] | 0;
        $2_1 = ($4_1 << 2 | 0) + 5028 | 0;
        if ((HEAP32[$2_1 >> 2] | 0 | 0) != ($3_1 | 0)) {
         break label$31
        }
        HEAP32[$2_1 >> 2] = $5_1;
        if ($5_1) {
         break label$30
        }
        HEAP32[(0 + 4728 | 0) >> 2] = (HEAP32[(0 + 4728 | 0) >> 2] | 0) & (__wasm_rotl_i32(-2 | 0, $4_1 | 0) | 0) | 0;
        break label$20;
       }
       HEAP32[($7_1 + ((HEAP32[($7_1 + 16 | 0) >> 2] | 0 | 0) == ($3_1 | 0) ? 16 : 20) | 0) >> 2] = $5_1;
       if (!$5_1) {
        break label$20
       }
      }
      HEAP32[($5_1 + 24 | 0) >> 2] = $7_1;
      label$32 : {
       $2_1 = HEAP32[($3_1 + 16 | 0) >> 2] | 0;
       if (!$2_1) {
        break label$32
       }
       HEAP32[($5_1 + 16 | 0) >> 2] = $2_1;
       HEAP32[($2_1 + 24 | 0) >> 2] = $5_1;
      }
      $2_1 = HEAP32[($3_1 + 20 | 0) >> 2] | 0;
      if (!$2_1) {
       break label$20
      }
      HEAP32[($5_1 + 20 | 0) >> 2] = $2_1;
      HEAP32[($2_1 + 24 | 0) >> 2] = $5_1;
     }
     HEAP32[($1_1 + 4 | 0) >> 2] = $0_1 | 1 | 0;
     HEAP32[($1_1 + $0_1 | 0) >> 2] = $0_1;
     if (($1_1 | 0) != (HEAP32[(0 + 4744 | 0) >> 2] | 0 | 0)) {
      break label$16
     }
     HEAP32[(0 + 4732 | 0) >> 2] = $0_1;
     return;
    }
    HEAP32[($3_1 + 4 | 0) >> 2] = $2_1 & -2 | 0;
    HEAP32[($1_1 + 4 | 0) >> 2] = $0_1 | 1 | 0;
    HEAP32[($1_1 + $0_1 | 0) >> 2] = $0_1;
   }
   label$33 : {
    if ($0_1 >>> 0 > 255 >>> 0) {
     break label$33
    }
    $2_1 = $0_1 >>> 3 | 0;
    $0_1 = ($2_1 << 3 | 0) + 4764 | 0;
    label$34 : {
     label$35 : {
      $4_1 = HEAP32[(0 + 4724 | 0) >> 2] | 0;
      $2_1 = 1 << $2_1 | 0;
      if ($4_1 & $2_1 | 0) {
       break label$35
      }
      HEAP32[(0 + 4724 | 0) >> 2] = $4_1 | $2_1 | 0;
      $2_1 = $0_1;
      break label$34;
     }
     $2_1 = HEAP32[($0_1 + 8 | 0) >> 2] | 0;
    }
    HEAP32[($0_1 + 8 | 0) >> 2] = $1_1;
    HEAP32[($2_1 + 12 | 0) >> 2] = $1_1;
    HEAP32[($1_1 + 12 | 0) >> 2] = $0_1;
    HEAP32[($1_1 + 8 | 0) >> 2] = $2_1;
    return;
   }
   $2_1 = 0;
   label$36 : {
    $4_1 = $0_1 >>> 8 | 0;
    if (!$4_1) {
     break label$36
    }
    $2_1 = 31;
    if ($0_1 >>> 0 > 16777215 >>> 0) {
     break label$36
    }
    $2_1 = (($4_1 + 1048320 | 0) >>> 16 | 0) & 8 | 0;
    $4_1 = $4_1 << $2_1 | 0;
    $408 = $4_1;
    $4_1 = (($4_1 + 520192 | 0) >>> 16 | 0) & 4 | 0;
    $5_1 = $408 << $4_1 | 0;
    $415 = $5_1;
    $5_1 = (($5_1 + 245760 | 0) >>> 16 | 0) & 2 | 0;
    $2_1 = (($415 << $5_1 | 0) >>> 15 | 0) - ($4_1 | $2_1 | 0 | $5_1 | 0) | 0;
    $2_1 = ($2_1 << 1 | 0 | (($0_1 >>> ($2_1 + 21 | 0) | 0) & 1 | 0) | 0) + 28 | 0;
   }
   HEAP32[($1_1 + 16 | 0) >> 2] = 0;
   HEAP32[($1_1 + 20 | 0) >> 2] = 0;
   HEAP32[($1_1 + 28 | 0) >> 2] = $2_1;
   $4_1 = ($2_1 << 2 | 0) + 5028 | 0;
   label$37 : {
    label$38 : {
     label$39 : {
      label$40 : {
       $5_1 = HEAP32[(0 + 4728 | 0) >> 2] | 0;
       $3_1 = 1 << $2_1 | 0;
       if ($5_1 & $3_1 | 0) {
        break label$40
       }
       HEAP32[(0 + 4728 | 0) >> 2] = $5_1 | $3_1 | 0;
       HEAP32[$4_1 >> 2] = $1_1;
       HEAP32[($1_1 + 24 | 0) >> 2] = $4_1;
       break label$39;
      }
      $2_1 = $0_1 << (($2_1 | 0) == (31 | 0) ? 0 : 25 - ($2_1 >>> 1 | 0) | 0) | 0;
      $5_1 = HEAP32[$4_1 >> 2] | 0;
      label$41 : while (1) {
       $4_1 = $5_1;
       if (((HEAP32[($5_1 + 4 | 0) >> 2] | 0) & -8 | 0 | 0) == ($0_1 | 0)) {
        break label$38
       }
       $5_1 = $2_1 >>> 29 | 0;
       $2_1 = $2_1 << 1 | 0;
       $3_1 = ($4_1 + ($5_1 & 4 | 0) | 0) + 16 | 0;
       $5_1 = HEAP32[$3_1 >> 2] | 0;
       if ($5_1) {
        continue label$41
       }
       break label$41;
      };
      HEAP32[$3_1 >> 2] = $1_1;
      HEAP32[($1_1 + 24 | 0) >> 2] = $4_1;
     }
     HEAP32[($1_1 + 12 | 0) >> 2] = $1_1;
     HEAP32[($1_1 + 8 | 0) >> 2] = $1_1;
     break label$37;
    }
    $0_1 = HEAP32[($4_1 + 8 | 0) >> 2] | 0;
    HEAP32[($0_1 + 12 | 0) >> 2] = $1_1;
    HEAP32[($4_1 + 8 | 0) >> 2] = $1_1;
    HEAP32[($1_1 + 24 | 0) >> 2] = 0;
    HEAP32[($1_1 + 12 | 0) >> 2] = $4_1;
    HEAP32[($1_1 + 8 | 0) >> 2] = $0_1;
   }
   $1_1 = (HEAP32[(0 + 4756 | 0) >> 2] | 0) + -1 | 0;
   HEAP32[(0 + 4756 | 0) >> 2] = $1_1;
   if ($1_1) {
    break label$1
   }
   $1_1 = 5180;
   label$42 : while (1) {
    $0_1 = HEAP32[$1_1 >> 2] | 0;
    $1_1 = $0_1 + 8 | 0;
    if ($0_1) {
     continue label$42
    }
    break label$42;
   };
   HEAP32[(0 + 4756 | 0) >> 2] = -1;
  }
 }
 
 function $373($0_1) {
  $0_1 = $0_1 | 0;
  var $2_1 = 0, $1_1 = 0;
  $1_1 = ($0_1 + 3 | 0) & -4 | 0;
  label$1 : {
   $0_1 = HEAP32[(0 + 5220 | 0) >> 2] | 0;
   if ($0_1) {
    break label$1
   }
   $0_1 = 5248128;
   HEAP32[(0 + 5220 | 0) >> 2] = 5248128;
  }
  $2_1 = $0_1 + $1_1 | 0;
  label$2 : {
   label$3 : {
    if (($1_1 | 0) < (1 | 0)) {
     break label$3
    }
    if ($2_1 >>> 0 <= $0_1 >>> 0) {
     break label$2
    }
   }
   label$4 : {
    if ($2_1 >>> 0 <= (__wasm_memory_size() << 16 | 0) >>> 0) {
     break label$4
    }
    if (!(fimport$17($2_1 | 0) | 0)) {
     break label$2
    }
   }
   HEAP32[(0 + 5220 | 0) >> 2] = $2_1;
   return $0_1 | 0;
  }
  HEAP32[($370() | 0) >> 2] = 48;
  return -1 | 0;
 }
 
 function $374($0_1, $1_1) {
  $0_1 = +$0_1;
  $1_1 = $1_1 | 0;
  var i64toi32_i32$4 = 0, i64toi32_i32$2 = 0, i64toi32_i32$1 = 0, i64toi32_i32$3 = 0, $8_1 = 0, $32_1 = 0.0, i64toi32_i32$0 = 0;
  label$1 : {
   label$2 : {
    if (($1_1 | 0) < (1024 | 0)) {
     break label$2
    }
    $0_1 = $0_1 * 8988465674311579538646525.0e283;
    label$3 : {
     if (($1_1 | 0) >= (2047 | 0)) {
      break label$3
     }
     $1_1 = $1_1 + -1023 | 0;
     break label$1;
    }
    $0_1 = $0_1 * 8988465674311579538646525.0e283;
    $1_1 = (($1_1 | 0) < (3069 | 0) ? $1_1 : 3069) + -2046 | 0;
    break label$1;
   }
   if (($1_1 | 0) > (-1023 | 0)) {
    break label$1
   }
   $0_1 = $0_1 * 2.2250738585072014e-308;
   label$4 : {
    if (($1_1 | 0) <= (-2045 | 0)) {
     break label$4
    }
    $1_1 = $1_1 + 1022 | 0;
    break label$1;
   }
   $0_1 = $0_1 * 2.2250738585072014e-308;
   $1_1 = (($1_1 | 0) > (-3066 | 0) ? $1_1 : -3066) + 2044 | 0;
  }
  $32_1 = $0_1;
  i64toi32_i32$0 = 0;
  i64toi32_i32$2 = $1_1 + 1023 | 0;
  i64toi32_i32$1 = 0;
  i64toi32_i32$3 = 52;
  i64toi32_i32$4 = i64toi32_i32$3 & 31 | 0;
  if (32 >>> 0 <= (i64toi32_i32$3 & 63 | 0) >>> 0) {
   i64toi32_i32$1 = i64toi32_i32$2 << i64toi32_i32$4 | 0;
   $8_1 = 0;
  } else {
   i64toi32_i32$1 = ((1 << i64toi32_i32$4 | 0) - 1 | 0) & (i64toi32_i32$2 >>> (32 - i64toi32_i32$4 | 0) | 0) | 0 | (i64toi32_i32$0 << i64toi32_i32$4 | 0) | 0;
   $8_1 = i64toi32_i32$2 << i64toi32_i32$4 | 0;
  }
  wasm2js_scratch_store_i32(0 | 0, $8_1 | 0);
  wasm2js_scratch_store_i32(1 | 0, i64toi32_i32$1 | 0);
  return +($32_1 * +wasm2js_scratch_load_f64());
 }
 
 function $375($0_1, $1_1, $2_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  var $4_1 = 0, $3_1 = 0, $5_1 = 0;
  label$1 : {
   if ($2_1 >>> 0 < 512 >>> 0) {
    break label$1
   }
   fimport$18($0_1 | 0, $1_1 | 0, $2_1 | 0) | 0;
   return $0_1 | 0;
  }
  $3_1 = $0_1 + $2_1 | 0;
  label$2 : {
   label$3 : {
    if (($1_1 ^ $0_1 | 0) & 3 | 0) {
     break label$3
    }
    label$4 : {
     label$5 : {
      if (($2_1 | 0) >= (1 | 0)) {
       break label$5
      }
      $2_1 = $0_1;
      break label$4;
     }
     label$6 : {
      if ($0_1 & 3 | 0) {
       break label$6
      }
      $2_1 = $0_1;
      break label$4;
     }
     $2_1 = $0_1;
     label$7 : while (1) {
      HEAP8[$2_1 >> 0] = HEAPU8[$1_1 >> 0] | 0;
      $1_1 = $1_1 + 1 | 0;
      $2_1 = $2_1 + 1 | 0;
      if ($2_1 >>> 0 >= $3_1 >>> 0) {
       break label$4
      }
      if ($2_1 & 3 | 0) {
       continue label$7
      }
      break label$7;
     };
    }
    label$8 : {
     $4_1 = $3_1 & -4 | 0;
     if ($4_1 >>> 0 < 64 >>> 0) {
      break label$8
     }
     $5_1 = $4_1 + -64 | 0;
     if ($2_1 >>> 0 > $5_1 >>> 0) {
      break label$8
     }
     label$9 : while (1) {
      HEAP32[$2_1 >> 2] = HEAP32[$1_1 >> 2] | 0;
      HEAP32[($2_1 + 4 | 0) >> 2] = HEAP32[($1_1 + 4 | 0) >> 2] | 0;
      HEAP32[($2_1 + 8 | 0) >> 2] = HEAP32[($1_1 + 8 | 0) >> 2] | 0;
      HEAP32[($2_1 + 12 | 0) >> 2] = HEAP32[($1_1 + 12 | 0) >> 2] | 0;
      HEAP32[($2_1 + 16 | 0) >> 2] = HEAP32[($1_1 + 16 | 0) >> 2] | 0;
      HEAP32[($2_1 + 20 | 0) >> 2] = HEAP32[($1_1 + 20 | 0) >> 2] | 0;
      HEAP32[($2_1 + 24 | 0) >> 2] = HEAP32[($1_1 + 24 | 0) >> 2] | 0;
      HEAP32[($2_1 + 28 | 0) >> 2] = HEAP32[($1_1 + 28 | 0) >> 2] | 0;
      HEAP32[($2_1 + 32 | 0) >> 2] = HEAP32[($1_1 + 32 | 0) >> 2] | 0;
      HEAP32[($2_1 + 36 | 0) >> 2] = HEAP32[($1_1 + 36 | 0) >> 2] | 0;
      HEAP32[($2_1 + 40 | 0) >> 2] = HEAP32[($1_1 + 40 | 0) >> 2] | 0;
      HEAP32[($2_1 + 44 | 0) >> 2] = HEAP32[($1_1 + 44 | 0) >> 2] | 0;
      HEAP32[($2_1 + 48 | 0) >> 2] = HEAP32[($1_1 + 48 | 0) >> 2] | 0;
      HEAP32[($2_1 + 52 | 0) >> 2] = HEAP32[($1_1 + 52 | 0) >> 2] | 0;
      HEAP32[($2_1 + 56 | 0) >> 2] = HEAP32[($1_1 + 56 | 0) >> 2] | 0;
      HEAP32[($2_1 + 60 | 0) >> 2] = HEAP32[($1_1 + 60 | 0) >> 2] | 0;
      $1_1 = $1_1 + 64 | 0;
      $2_1 = $2_1 + 64 | 0;
      if ($2_1 >>> 0 <= $5_1 >>> 0) {
       continue label$9
      }
      break label$9;
     };
    }
    if ($2_1 >>> 0 >= $4_1 >>> 0) {
     break label$2
    }
    label$10 : while (1) {
     HEAP32[$2_1 >> 2] = HEAP32[$1_1 >> 2] | 0;
     $1_1 = $1_1 + 4 | 0;
     $2_1 = $2_1 + 4 | 0;
     if ($2_1 >>> 0 < $4_1 >>> 0) {
      continue label$10
     }
     break label$2;
    };
   }
   label$11 : {
    if ($3_1 >>> 0 >= 4 >>> 0) {
     break label$11
    }
    $2_1 = $0_1;
    break label$2;
   }
   label$12 : {
    $4_1 = $3_1 + -4 | 0;
    if ($4_1 >>> 0 >= $0_1 >>> 0) {
     break label$12
    }
    $2_1 = $0_1;
    break label$2;
   }
   $2_1 = $0_1;
   label$13 : while (1) {
    HEAP8[$2_1 >> 0] = HEAPU8[$1_1 >> 0] | 0;
    HEAP8[($2_1 + 1 | 0) >> 0] = HEAPU8[($1_1 + 1 | 0) >> 0] | 0;
    HEAP8[($2_1 + 2 | 0) >> 0] = HEAPU8[($1_1 + 2 | 0) >> 0] | 0;
    HEAP8[($2_1 + 3 | 0) >> 0] = HEAPU8[($1_1 + 3 | 0) >> 0] | 0;
    $1_1 = $1_1 + 4 | 0;
    $2_1 = $2_1 + 4 | 0;
    if ($2_1 >>> 0 <= $4_1 >>> 0) {
     continue label$13
    }
    break label$13;
   };
  }
  label$14 : {
   if ($2_1 >>> 0 >= $3_1 >>> 0) {
    break label$14
   }
   label$15 : while (1) {
    HEAP8[$2_1 >> 0] = HEAPU8[$1_1 >> 0] | 0;
    $1_1 = $1_1 + 1 | 0;
    $2_1 = $2_1 + 1 | 0;
    if (($2_1 | 0) != ($3_1 | 0)) {
     continue label$15
    }
    break label$15;
   };
  }
  return $0_1 | 0;
 }
 
 function $376($0_1, $1_1, $2_1) {
  $0_1 = $0_1 | 0;
  $1_1 = $1_1 | 0;
  $2_1 = $2_1 | 0;
  var $3_1 = 0, i64toi32_i32$2 = 0, i64toi32_i32$0 = 0, $4_1 = 0, $6_1 = 0, i64toi32_i32$1 = 0, i64toi32_i32$4 = 0, $6$hi = 0, i64toi32_i32$3 = 0, $5_1 = 0, $14_1 = 0, $104$hi = 0;
  label$1 : {
   if (!$2_1) {
    break label$1
   }
   $3_1 = $2_1 + $0_1 | 0;
   HEAP8[($3_1 + -1 | 0) >> 0] = $1_1;
   HEAP8[$0_1 >> 0] = $1_1;
   if ($2_1 >>> 0 < 3 >>> 0) {
    break label$1
   }
   HEAP8[($3_1 + -2 | 0) >> 0] = $1_1;
   HEAP8[($0_1 + 1 | 0) >> 0] = $1_1;
   HEAP8[($3_1 + -3 | 0) >> 0] = $1_1;
   HEAP8[($0_1 + 2 | 0) >> 0] = $1_1;
   if ($2_1 >>> 0 < 7 >>> 0) {
    break label$1
   }
   HEAP8[($3_1 + -4 | 0) >> 0] = $1_1;
   HEAP8[($0_1 + 3 | 0) >> 0] = $1_1;
   if ($2_1 >>> 0 < 9 >>> 0) {
    break label$1
   }
   $4_1 = (0 - $0_1 | 0) & 3 | 0;
   $3_1 = $0_1 + $4_1 | 0;
   $1_1 = Math_imul($1_1 & 255 | 0, 16843009);
   HEAP32[$3_1 >> 2] = $1_1;
   $4_1 = ($2_1 - $4_1 | 0) & -4 | 0;
   $2_1 = $3_1 + $4_1 | 0;
   HEAP32[($2_1 + -4 | 0) >> 2] = $1_1;
   if ($4_1 >>> 0 < 9 >>> 0) {
    break label$1
   }
   HEAP32[($3_1 + 8 | 0) >> 2] = $1_1;
   HEAP32[($3_1 + 4 | 0) >> 2] = $1_1;
   HEAP32[($2_1 + -8 | 0) >> 2] = $1_1;
   HEAP32[($2_1 + -12 | 0) >> 2] = $1_1;
   if ($4_1 >>> 0 < 25 >>> 0) {
    break label$1
   }
   HEAP32[($3_1 + 24 | 0) >> 2] = $1_1;
   HEAP32[($3_1 + 20 | 0) >> 2] = $1_1;
   HEAP32[($3_1 + 16 | 0) >> 2] = $1_1;
   HEAP32[($3_1 + 12 | 0) >> 2] = $1_1;
   HEAP32[($2_1 + -16 | 0) >> 2] = $1_1;
   HEAP32[($2_1 + -20 | 0) >> 2] = $1_1;
   HEAP32[($2_1 + -24 | 0) >> 2] = $1_1;
   HEAP32[($2_1 + -28 | 0) >> 2] = $1_1;
   $5_1 = $3_1 & 4 | 0 | 24 | 0;
   $2_1 = $4_1 - $5_1 | 0;
   if ($2_1 >>> 0 < 32 >>> 0) {
    break label$1
   }
   i64toi32_i32$0 = 0;
   $6_1 = $1_1;
   $6$hi = i64toi32_i32$0;
   i64toi32_i32$2 = $1_1;
   i64toi32_i32$1 = 0;
   i64toi32_i32$3 = 32;
   i64toi32_i32$4 = i64toi32_i32$3 & 31 | 0;
   if (32 >>> 0 <= (i64toi32_i32$3 & 63 | 0) >>> 0) {
    i64toi32_i32$1 = i64toi32_i32$2 << i64toi32_i32$4 | 0;
    $14_1 = 0;
   } else {
    i64toi32_i32$1 = ((1 << i64toi32_i32$4 | 0) - 1 | 0) & (i64toi32_i32$2 >>> (32 - i64toi32_i32$4 | 0) | 0) | 0 | (i64toi32_i32$0 << i64toi32_i32$4 | 0) | 0;
    $14_1 = i64toi32_i32$2 << i64toi32_i32$4 | 0;
   }
   $104$hi = i64toi32_i32$1;
   i64toi32_i32$1 = $6$hi;
   i64toi32_i32$1 = $104$hi;
   i64toi32_i32$0 = $14_1;
   i64toi32_i32$2 = $6$hi;
   i64toi32_i32$3 = $6_1;
   i64toi32_i32$2 = i64toi32_i32$1 | i64toi32_i32$2 | 0;
   $6_1 = i64toi32_i32$0 | $6_1 | 0;
   $6$hi = i64toi32_i32$2;
   $1_1 = $3_1 + $5_1 | 0;
   label$2 : while (1) {
    i64toi32_i32$2 = $6$hi;
    i64toi32_i32$0 = $1_1;
    HEAP32[($1_1 + 24 | 0) >> 2] = $6_1;
    HEAP32[($1_1 + 28 | 0) >> 2] = i64toi32_i32$2;
    i64toi32_i32$0 = $1_1;
    HEAP32[($1_1 + 16 | 0) >> 2] = $6_1;
    HEAP32[($1_1 + 20 | 0) >> 2] = i64toi32_i32$2;
    i64toi32_i32$0 = $1_1;
    HEAP32[($1_1 + 8 | 0) >> 2] = $6_1;
    HEAP32[($1_1 + 12 | 0) >> 2] = i64toi32_i32$2;
    i64toi32_i32$0 = $1_1;
    HEAP32[$1_1 >> 2] = $6_1;
    HEAP32[($1_1 + 4 | 0) >> 2] = i64toi32_i32$2;
    $1_1 = $1_1 + 32 | 0;
    $2_1 = $2_1 + -32 | 0;
    if ($2_1 >>> 0 > 31 >>> 0) {
     continue label$2
    }
    break label$2;
   };
  }
  return $0_1 | 0;
 }
 
 function $377($0_1) {
  $0_1 = $0_1 | 0;
  var $1_1 = 0, $2_1 = 0, $3_1 = 0;
  $1_1 = $0_1;
  label$1 : {
   label$2 : {
    if (!($0_1 & 3 | 0)) {
     break label$2
    }
    label$3 : {
     if (HEAPU8[$0_1 >> 0] | 0) {
      break label$3
     }
     return $0_1 - $0_1 | 0 | 0;
    }
    $1_1 = $0_1;
    label$4 : while (1) {
     $1_1 = $1_1 + 1 | 0;
     if (!($1_1 & 3 | 0)) {
      break label$2
     }
     if (!(HEAPU8[$1_1 >> 0] | 0)) {
      break label$1
     }
     continue label$4;
    };
   }
   label$5 : while (1) {
    $2_1 = $1_1;
    $1_1 = $1_1 + 4 | 0;
    $3_1 = HEAP32[$2_1 >> 2] | 0;
    if (!((($3_1 ^ -1 | 0) & ($3_1 + -16843009 | 0) | 0) & -2139062144 | 0)) {
     continue label$5
    }
    break label$5;
   };
   label$6 : {
    if ($3_1 & 255 | 0) {
     break label$6
    }
    return $2_1 - $0_1 | 0 | 0;
   }
   label$7 : while (1) {
    $3_1 = HEAPU8[($2_1 + 1 | 0) >> 0] | 0;
    $1_1 = $2_1 + 1 | 0;
    $2_1 = $1_1;
    if ($3_1) {
     continue label$7
    }
    break label$7;
   };
  }
  return $1_1 - $0_1 | 0 | 0;
 }
 
 function $378() {
  return global$0 | 0;
 }
 
 function $379($0_1) {
  $0_1 = $0_1 | 0;
  global$0 = $0_1;
 }
 
 function $380($0_1) {
  $0_1 = $0_1 | 0;
  var $1_1 = 0;
  $1_1 = (global$0 - $0_1 | 0) & -16 | 0;
  global$0 = $1_1;
  return $1_1 | 0;
 }
 
 function $381($0_1) {
  $0_1 = $0_1 | 0;
  return 1 | 0;
 }
 
 function $382($0_1) {
  $0_1 = $0_1 | 0;
 }
 
 function $383($0_1) {
  $0_1 = $0_1 | 0;
 }
 
 function $384($0_1) {
  $0_1 = $0_1 | 0;
 }
 
 function $385() {
  $383(5224 | 0);
  return 5232 | 0;
 }
 
 function $386() {
  $384(5224 | 0);
 }
 
 function $387($0_1) {
  $0_1 = $0_1 | 0;
  var $2_1 = 0, $1_1 = 0;
  label$1 : {
   label$2 : {
    if (!$0_1) {
     break label$2
    }
    label$3 : {
     if ((HEAP32[($0_1 + 76 | 0) >> 2] | 0 | 0) > (-1 | 0)) {
      break label$3
     }
     return $388($0_1 | 0) | 0 | 0;
    }
    $1_1 = $381($0_1 | 0) | 0;
    $2_1 = $388($0_1 | 0) | 0;
    if (!$1_1) {
     break label$1
    }
    $382($0_1 | 0);
    return $2_1 | 0;
   }
   $2_1 = 0;
   label$4 : {
    if (!(HEAP32[(0 + 5236 | 0) >> 2] | 0)) {
     break label$4
    }
    $2_1 = $387(HEAP32[(0 + 5236 | 0) >> 2] | 0 | 0) | 0;
   }
   label$5 : {
    $0_1 = HEAP32[($385() | 0) >> 2] | 0;
    if (!$0_1) {
     break label$5
    }
    label$6 : while (1) {
     $1_1 = 0;
     label$7 : {
      if ((HEAP32[($0_1 + 76 | 0) >> 2] | 0 | 0) < (0 | 0)) {
       break label$7
      }
      $1_1 = $381($0_1 | 0) | 0;
     }
     label$8 : {
      if ((HEAP32[($0_1 + 20 | 0) >> 2] | 0) >>> 0 <= (HEAP32[($0_1 + 28 | 0) >> 2] | 0) >>> 0) {
       break label$8
      }
      $2_1 = $388($0_1 | 0) | 0 | $2_1 | 0;
     }
     label$9 : {
      if (!$1_1) {
       break label$9
      }
      $382($0_1 | 0);
     }
     $0_1 = HEAP32[($0_1 + 56 | 0) >> 2] | 0;
     if ($0_1) {
      continue label$6
     }
     break label$6;
    };
   }
   $386();
  }
  return $2_1 | 0;
 }
 
 function $388($0_1) {
  $0_1 = $0_1 | 0;
  var i64toi32_i32$1 = 0, i64toi32_i32$0 = 0, $1_1 = 0, $2_1 = 0;
  label$1 : {
   if ((HEAP32[($0_1 + 20 | 0) >> 2] | 0) >>> 0 <= (HEAP32[($0_1 + 28 | 0) >> 2] | 0) >>> 0) {
    break label$1
   }
   FUNCTION_TABLE[HEAP32[($0_1 + 36 | 0) >> 2] | 0 | 0]($0_1, 0, 0) | 0;
   if (HEAP32[($0_1 + 20 | 0) >> 2] | 0) {
    break label$1
   }
   return -1 | 0;
  }
  label$2 : {
   $1_1 = HEAP32[($0_1 + 4 | 0) >> 2] | 0;
   $2_1 = HEAP32[($0_1 + 8 | 0) >> 2] | 0;
   if ($1_1 >>> 0 >= $2_1 >>> 0) {
    break label$2
   }
   i64toi32_i32$1 = $1_1 - $2_1 | 0;
   i64toi32_i32$0 = i64toi32_i32$1 >> 31 | 0;
   i64toi32_i32$0 = FUNCTION_TABLE[HEAP32[($0_1 + 40 | 0) >> 2] | 0 | 0]($0_1, i64toi32_i32$1, i64toi32_i32$0, 1) | 0;
   i64toi32_i32$1 = i64toi32_i32$HIGH_BITS;
  }
  HEAP32[($0_1 + 28 | 0) >> 2] = 0;
  i64toi32_i32$0 = $0_1;
  i64toi32_i32$1 = 0;
  HEAP32[($0_1 + 16 | 0) >> 2] = 0;
  HEAP32[($0_1 + 20 | 0) >> 2] = i64toi32_i32$1;
  i64toi32_i32$0 = $0_1;
  i64toi32_i32$1 = 0;
  HEAP32[($0_1 + 4 | 0) >> 2] = 0;
  HEAP32[($0_1 + 8 | 0) >> 2] = i64toi32_i32$1;
  return 0 | 0;
 }
 
 function $389($0_1) {
  $0_1 = $0_1 | 0;
  return abort() | 0;
 }
 
 function __wasm_rotl_i32(var$0, var$1) {
  var$0 = var$0 | 0;
  var$1 = var$1 | 0;
  var var$2 = 0;
  var$2 = var$1 & 31 | 0;
  var$1 = (0 - var$1 | 0) & 31 | 0;
  return ((-1 >>> var$2 | 0) & var$0 | 0) << var$2 | 0 | (((-1 << var$1 | 0) & var$0 | 0) >>> var$1 | 0) | 0 | 0;
 }
 
 // EMSCRIPTEN_END_FUNCS
;
 FUNCTION_TABLE[1] = $2;
 FUNCTION_TABLE[2] = $14;
 FUNCTION_TABLE[3] = $6;
 FUNCTION_TABLE[4] = $9;
 FUNCTION_TABLE[5] = $15;
 FUNCTION_TABLE[6] = $16;
 FUNCTION_TABLE[7] = $17;
 FUNCTION_TABLE[8] = $18;
 FUNCTION_TABLE[9] = $19;
 FUNCTION_TABLE[10] = $20;
 FUNCTION_TABLE[11] = $21;
 FUNCTION_TABLE[12] = $22;
 FUNCTION_TABLE[13] = $23;
 FUNCTION_TABLE[14] = $24;
 FUNCTION_TABLE[15] = $25;
 FUNCTION_TABLE[16] = $26;
 FUNCTION_TABLE[17] = $27;
 FUNCTION_TABLE[18] = $28;
 FUNCTION_TABLE[19] = $29;
 FUNCTION_TABLE[20] = $96;
 FUNCTION_TABLE[21] = $103;
 FUNCTION_TABLE[22] = $118;
 FUNCTION_TABLE[23] = $124;
 FUNCTION_TABLE[24] = $133;
 FUNCTION_TABLE[25] = $64;
 FUNCTION_TABLE[26] = $67;
 FUNCTION_TABLE[27] = $145;
 FUNCTION_TABLE[28] = $153;
 FUNCTION_TABLE[29] = $155;
 FUNCTION_TABLE[30] = $158;
 FUNCTION_TABLE[31] = $162;
 FUNCTION_TABLE[32] = $147;
 FUNCTION_TABLE[33] = $164;
 FUNCTION_TABLE[34] = $148;
 FUNCTION_TABLE[35] = $149;
 FUNCTION_TABLE[36] = $146;
 FUNCTION_TABLE[37] = $168;
 FUNCTION_TABLE[38] = $151;
 FUNCTION_TABLE[39] = $177;
 FUNCTION_TABLE[40] = $188;
 FUNCTION_TABLE[41] = $286;
 FUNCTION_TABLE[42] = $336;
 FUNCTION_TABLE[43] = $339;
 FUNCTION_TABLE[44] = $337;
 FUNCTION_TABLE[45] = $338;
 FUNCTION_TABLE[46] = $345;
 FUNCTION_TABLE[47] = $340;
 FUNCTION_TABLE[48] = $348;
 FUNCTION_TABLE[49] = $341;
 FUNCTION_TABLE[50] = $349;
 FUNCTION_TABLE[51] = $369;
 FUNCTION_TABLE[52] = $366;
 FUNCTION_TABLE[53] = $352;
 FUNCTION_TABLE[54] = $342;
 FUNCTION_TABLE[55] = $368;
 FUNCTION_TABLE[56] = $365;
 FUNCTION_TABLE[57] = $353;
 FUNCTION_TABLE[58] = $343;
 FUNCTION_TABLE[59] = $367;
 FUNCTION_TABLE[60] = $362;
 FUNCTION_TABLE[61] = $355;
 FUNCTION_TABLE[62] = $344;
 FUNCTION_TABLE[63] = $357;
 function __wasm_memory_size() {
  return buffer.byteLength / 65536 | 0;
 }
 
 return {
  "__wasm_call_ctors": $0, 
  "__getTypeName": $197, 
  "__embind_register_native_and_builtin_types": $199, 
  "__errno_location": $370, 
  "malloc": $371, 
  "fflush": $387, 
  "stackSave": $378, 
  "stackRestore": $379, 
  "stackAlloc": $380, 
  "free": $372, 
  "__growWasmMemory": $389
 };
}

var bufferView = new Uint8Array(wasmMemory.buffer);
for (var base64ReverseLookup = new Uint8Array(123/*'z'+1*/), i = 25; i >= 0; --i) {
    base64ReverseLookup[48+i] = 52+i; // '0-9'
    base64ReverseLookup[65+i] = i; // 'A-Z'
    base64ReverseLookup[97+i] = 26+i; // 'a-z'
  }
  base64ReverseLookup[43] = 62; // '+'
  base64ReverseLookup[47] = 63; // '/'
  /** @noinline Inlining this function would mean expanding the base64 string 4x times in the source code, which Closure seems to be happy to do. */
  function base64DecodeToExistingUint8Array(uint8Array, offset, b64) {
    var b1, b2, i = 0, j = offset, bLength = b64.length, end = offset + (bLength*3>>2) - (b64[bLength-2] == '=') - (b64[bLength-1] == '=');
    for (; i < bLength; i += 4) {
      b1 = base64ReverseLookup[b64.charCodeAt(i+1)];
      b2 = base64ReverseLookup[b64.charCodeAt(i+2)];
      uint8Array[j++] = base64ReverseLookup[b64.charCodeAt(i)] << 2 | b1 >> 4;
      if (j < end) uint8Array[j++] = b1 << 4 | b2 >> 2;
      if (j < end) uint8Array[j++] = b2 << 6 | base64ReverseLookup[b64.charCodeAt(i+3)];
    } 
  }
  base64DecodeToExistingUint8Array(bufferView, 1024, "TU9VU0UATEVGVABNSURETEUAUklHSFQAVmVjdG9yMgB4AHkAVmVjdG9yMwB6AE1hdDQAbGVuZ3RoAGxvb2tBdAA1TU9VU0UAKBEAAEEEAABpIDwgdGhpcy0+bGVuZ3RoKCkAZ2xtL2dsbS9kZXRhaWwvdHlwZV9tYXQ0eDQuaW5sAG9wZXJhdG9yW10AaSA+PSAwICYmIHN0YXRpY19jYXN0PGRldGFpbDo6Y29tcG9uZW50X2NvdW50X3Q+KGkpIDwgZGV0YWlsOjpjb21wb25lbnRfY291bnQoKnRoaXMpAGdsbS9nbG0vZGV0YWlsL3R5cGVfdmVjNC5pbmwATjNnbG01dHZlYzJJZkxOU185cHJlY2lzaW9uRTBFRUUAdBEAAP8EAABQTjNnbG01dHZlYzJJZkxOU185cHJlY2lzaW9uRTBFRUUAAABUEgAAKAUAAAAAAAAgBQAAUEtOM2dsbTV0dmVjMklmTE5TXzlwcmVjaXNpb25FMEVFRQAAVBIAAFwFAAABAAAAIAUAAGlpAHYAdmkATAUAAEwFAAAYEQAAGBEAAGlpZGQAZmlpAHZpaWYATjNnbG01dHZlYzNJZkxOU185cHJlY2lzaW9uRTBFRUUAAHQRAAC2BQAAUE4zZ2xtNXR2ZWMzSWZMTlNfOXByZWNpc2lvbkUwRUVFAAAAVBIAAOAFAAAAAAAA2AUAAFBLTjNnbG01dHZlYzNJZkxOU185cHJlY2lzaW9uRTBFRUUAAFQSAAAUBgAAAQAAANgFAAAEBgAAAAAAAAQGAAAYEQAAGBEAABgRAABpaWRkZABOM2dsbTd0bWF0NHg0SWZMTlNfOXByZWNpc2lvbkUwRUVFAAAAAHQRAABmBgAAUE4zZ2xtN3RtYXQ0eDRJZkxOU185cHJlY2lzaW9uRTBFRUUAVBIAAJQGAAAAAAAAjAYAAFBLTjNnbG03dG1hdDR4NElmTE5TXzlwcmVjaXNpb25FMEVFRQAAAABUEgAAyAYAAAEAAACMBgAAuAYAABgRAABpaWQADBEAANgFAAAAAAAAAAAAAAAAAACMBgAA2AUAANgFAADYBQAAaWlpaWkAQ29sb3IAcgBnAGIAaXNDb2xvcgBjb252ZXJ0TGluZWFyVG9TUkdCADVDb2xvcgAAAAB0EQAAXgcAAFA1Q29sb3IAVBIAAHAHAAAAAAAAaAcAAFBLNUNvbG9yAAAAAFQSAACIBwAAAQAAAGgHAAAAAAAAAAAAAAAAAAB4BwAAGBEAABgRAAAYEQAAZGlpAGlpaQBoBwAAeAcAAHZvaWQAYm9vbABjaGFyAHNpZ25lZCBjaGFyAHVuc2lnbmVkIGNoYXIAc2hvcnQAdW5zaWduZWQgc2hvcnQAaW50AHVuc2lnbmVkIGludABsb25nAHVuc2lnbmVkIGxvbmcAZmxvYXQAZG91YmxlAHN0ZDo6c3RyaW5nAHN0ZDo6YmFzaWNfc3RyaW5nPHVuc2lnbmVkIGNoYXI+AHN0ZDo6d3N0cmluZwBzdGQ6OnUxNnN0cmluZwBzdGQ6OnUzMnN0cmluZwBlbXNjcmlwdGVuOjp2YWwAZW1zY3JpcHRlbjo6bWVtb3J5X3ZpZXc8Y2hhcj4AZW1zY3JpcHRlbjo6bWVtb3J5X3ZpZXc8c2lnbmVkIGNoYXI+AGVtc2NyaXB0ZW46Om1lbW9yeV92aWV3PHVuc2lnbmVkIGNoYXI+AGVtc2NyaXB0ZW46Om1lbW9yeV92aWV3PHNob3J0PgBlbXNjcmlwdGVuOjptZW1vcnlfdmlldzx1bnNpZ25lZCBzaG9ydD4AZW1zY3JpcHRlbjo6bWVtb3J5X3ZpZXc8aW50PgBlbXNjcmlwdGVuOjptZW1vcnlfdmlldzx1bnNpZ25lZCBpbnQ+AGVtc2NyaXB0ZW46Om1lbW9yeV92aWV3PGxvbmc+AGVtc2NyaXB0ZW46Om1lbW9yeV92aWV3PHVuc2lnbmVkIGxvbmc+AGVtc2NyaXB0ZW46Om1lbW9yeV92aWV3PGludDhfdD4AZW1zY3JpcHRlbjo6bWVtb3J5X3ZpZXc8dWludDhfdD4AZW1zY3JpcHRlbjo6bWVtb3J5X3ZpZXc8aW50MTZfdD4AZW1zY3JpcHRlbjo6bWVtb3J5X3ZpZXc8dWludDE2X3Q+AGVtc2NyaXB0ZW46Om1lbW9yeV92aWV3PGludDMyX3Q+AGVtc2NyaXB0ZW46Om1lbW9yeV92aWV3PHVpbnQzMl90PgBlbXNjcmlwdGVuOjptZW1vcnlfdmlldzxmbG9hdD4AZW1zY3JpcHRlbjo6bWVtb3J5X3ZpZXc8ZG91YmxlPgBOU3QzX18yMTJiYXNpY19zdHJpbmdJY05TXzExY2hhcl90cmFpdHNJY0VFTlNfOWFsbG9jYXRvckljRUVFRQBOU3QzX18yMjFfX2Jhc2ljX3N0cmluZ19jb21tb25JTGIxRUVFAAB0EQAAJQsAAPgRAADmCgAAAAAAAAEAAABMCwAAAAAAAE5TdDNfXzIxMmJhc2ljX3N0cmluZ0loTlNfMTFjaGFyX3RyYWl0c0loRUVOU185YWxsb2NhdG9ySWhFRUVFAAD4EQAAbAsAAAAAAAABAAAATAsAAAAAAABOU3QzX18yMTJiYXNpY19zdHJpbmdJd05TXzExY2hhcl90cmFpdHNJd0VFTlNfOWFsbG9jYXRvckl3RUVFRQAA+BEAAMQLAAAAAAAAAQAAAEwLAAAAAAAATlN0M19fMjEyYmFzaWNfc3RyaW5nSURzTlNfMTFjaGFyX3RyYWl0c0lEc0VFTlNfOWFsbG9jYXRvcklEc0VFRUUAAAD4EQAAHAwAAAAAAAABAAAATAsAAAAAAABOU3QzX18yMTJiYXNpY19zdHJpbmdJRGlOU18xMWNoYXJfdHJhaXRzSURpRUVOU185YWxsb2NhdG9ySURpRUVFRQAAAPgRAAB4DAAAAAAAAAEAAABMCwAAAAAAAE4xMGVtc2NyaXB0ZW4zdmFsRQAAdBEAANQMAABOMTBlbXNjcmlwdGVuMTFtZW1vcnlfdmlld0ljRUUAAHQRAADwDAAATjEwZW1zY3JpcHRlbjExbWVtb3J5X3ZpZXdJYUVFAAB0EQAAGA0AAE4xMGVtc2NyaXB0ZW4xMW1lbW9yeV92aWV3SWhFRQAAdBEAAEANAABOMTBlbXNjcmlwdGVuMTFtZW1vcnlfdmlld0lzRUUAAHQRAABoDQAATjEwZW1zY3JpcHRlbjExbWVtb3J5X3ZpZXdJdEVFAAB0EQAAkA0AAE4xMGVtc2NyaXB0ZW4xMW1lbW9yeV92aWV3SWlFRQAAdBEAALgNAABOMTBlbXNjcmlwdGVuMTFtZW1vcnlfdmlld0lqRUUAAHQRAADgDQAATjEwZW1zY3JpcHRlbjExbWVtb3J5X3ZpZXdJbEVFAAB0EQAACA4AAE4xMGVtc2NyaXB0ZW4xMW1lbW9yeV92aWV3SW1FRQAAdBEAADAOAABOMTBlbXNjcmlwdGVuMTFtZW1vcnlfdmlld0lmRUUAAHQRAABYDgAATjEwZW1zY3JpcHRlbjExbWVtb3J5X3ZpZXdJZEVFAAB0EQAAgA4AAAAAAAAAAAAAAAAAAAAA8D8AAAAAAAD4PwAAAAAAAAAABtDPQ+v9TD4AAAAAAAAAAAAAAEADuOI/U3Q5dHlwZV9pbmZvAAAAAHQRAADgDgAATjEwX19jeHhhYml2MTE2X19zaGltX3R5cGVfaW5mb0UAAAAAnBEAAPgOAADwDgAATjEwX19jeHhhYml2MTE3X19jbGFzc190eXBlX2luZm9FAAAAnBEAACgPAAAcDwAATjEwX19jeHhhYml2MTE3X19wYmFzZV90eXBlX2luZm9FAAAAnBEAAFgPAAAcDwAATjEwX19jeHhhYml2MTE5X19wb2ludGVyX3R5cGVfaW5mb0UAnBEAAIgPAAB8DwAATjEwX19jeHhhYml2MTIwX19mdW5jdGlvbl90eXBlX2luZm9FAAAAAJwRAAC4DwAAHA8AAE4xMF9fY3h4YWJpdjEyOV9fcG9pbnRlcl90b19tZW1iZXJfdHlwZV9pbmZvRQAAAJwRAADsDwAAfA8AAAAAAABsEAAAKgAAACsAAAAsAAAALQAAAC4AAABOMTBfX2N4eGFiaXYxMjNfX2Z1bmRhbWVudGFsX3R5cGVfaW5mb0UAnBEAAEQQAAAcDwAAdgAAADAQAAB4EAAARG4AADAQAACEEAAAYgAAADAQAACQEAAAYwAAADAQAACcEAAAaAAAADAQAACoEAAAYQAAADAQAAC0EAAAcwAAADAQAADAEAAAdAAAADAQAADMEAAAaQAAADAQAADYEAAAagAAADAQAADkEAAAbAAAADAQAADwEAAAbQAAADAQAAD8EAAAZgAAADAQAAAIEQAAZAAAADAQAAAUEQAAAAAAAGARAAAqAAAALwAAACwAAAAtAAAAMAAAAE4xMF9fY3h4YWJpdjExNl9fZW51bV90eXBlX2luZm9FAAAAAJwRAAA8EQAAHA8AAAAAAABMDwAAKgAAADEAAAAsAAAALQAAADIAAAAzAAAANAAAADUAAAAAAAAA5BEAACoAAAA2AAAALAAAAC0AAAAyAAAANwAAADgAAAA5AAAATjEwX19jeHhhYml2MTIwX19zaV9jbGFzc190eXBlX2luZm9FAAAAAJwRAAC8EQAATA8AAAAAAABAEgAAKgAAADoAAAAsAAAALQAAADIAAAA7AAAAPAAAAD0AAABOMTBfX2N4eGFiaXYxMjFfX3ZtaV9jbGFzc190eXBlX2luZm9FAAAAnBEAABgSAABMDwAAAAAAAKwPAAAqAAAAPgAAACwAAAAtAAAAPwAAAA==");
base64DecodeToExistingUint8Array(bufferView, 4712, "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
return asmFunc({
    'Int8Array': Int8Array,
    'Int16Array': Int16Array,
    'Int32Array': Int32Array,
    'Uint8Array': Uint8Array,
    'Uint16Array': Uint16Array,
    'Uint32Array': Uint32Array,
    'Float32Array': Float32Array,
    'Float64Array': Float64Array,
    'NaN': NaN,
    'Infinity': Infinity,
    'Math': Math
  },
  asmLibraryArg,
  wasmMemory.buffer
)

}
)(asmLibraryArg, wasmMemory, wasmTable);
  },

  instantiate: /** @suppress{checkTypes} */ function(binary, info) {
    return {
      then: function(ok) {
        var module = new WebAssembly.Module(binary);
        ok({
          'instance': new WebAssembly.Instance(module)
        });
        // Emulate a simple WebAssembly.instantiate(..).then(()=>{}).catch(()=>{}) syntax.
        return { catch: function() {} };
      }
    };
  },

  RuntimeError: Error
};

// We don't need to actually download a wasm binary, mark it as present but empty.
wasmBinary = [];



if (typeof WebAssembly !== 'object') {
  abort('no native wasm support detected');
}




// In MINIMAL_RUNTIME, setValue() and getValue() are only available when building with safe heap enabled, for heap safety checking.
// In traditional runtime, setValue() and getValue() are always available (although their use is highly discouraged due to perf penalties)

/** @param {number} ptr
    @param {number} value
    @param {string} type
    @param {number|boolean=} noSafe */
function setValue(ptr, value, type, noSafe) {
  type = type || 'i8';
  if (type.charAt(type.length-1) === '*') type = 'i32'; // pointers are 32-bit
    switch(type) {
      case 'i1': HEAP8[((ptr)>>0)]=value; break;
      case 'i8': HEAP8[((ptr)>>0)]=value; break;
      case 'i16': HEAP16[((ptr)>>1)]=value; break;
      case 'i32': HEAP32[((ptr)>>2)]=value; break;
      case 'i64': (tempI64 = [value>>>0,(tempDouble=value,(+(Math_abs(tempDouble))) >= 1.0 ? (tempDouble > 0.0 ? ((Math_min((+(Math_floor((tempDouble)/4294967296.0))), 4294967295.0))|0)>>>0 : (~~((+(Math_ceil((tempDouble - +(((~~(tempDouble)))>>>0))/4294967296.0)))))>>>0) : 0)],HEAP32[((ptr)>>2)]=tempI64[0],HEAP32[(((ptr)+(4))>>2)]=tempI64[1]); break;
      case 'float': HEAPF32[((ptr)>>2)]=value; break;
      case 'double': HEAPF64[((ptr)>>3)]=value; break;
      default: abort('invalid type for setValue: ' + type);
    }
}

/** @param {number} ptr
    @param {string} type
    @param {number|boolean=} noSafe */
function getValue(ptr, type, noSafe) {
  type = type || 'i8';
  if (type.charAt(type.length-1) === '*') type = 'i32'; // pointers are 32-bit
    switch(type) {
      case 'i1': return HEAP8[((ptr)>>0)];
      case 'i8': return HEAP8[((ptr)>>0)];
      case 'i16': return HEAP16[((ptr)>>1)];
      case 'i32': return HEAP32[((ptr)>>2)];
      case 'i64': return HEAP32[((ptr)>>2)];
      case 'float': return HEAPF32[((ptr)>>2)];
      case 'double': return HEAPF64[((ptr)>>3)];
      default: abort('invalid type for getValue: ' + type);
    }
  return null;
}






// Wasm globals

var wasmMemory;

// In fastcomp asm.js, we don't need a wasm Table at all.
// In the wasm backend, we polyfill the WebAssembly object,
// so this creates a (non-native-wasm) table for us.

var wasmTable = new WebAssembly.Table({
  'initial': 64,
  'maximum': 64,
  'element': 'anyfunc'
});




//========================================
// Runtime essentials
//========================================

// whether we are quitting the application. no code should run after this.
// set in exit() and abort()
var ABORT = false;

// set by exit() and abort().  Passed to 'onExit' handler.
// NOTE: This is also used as the process return code code in shell environments
// but only when noExitRuntime is false.
var EXITSTATUS = 0;

/** @type {function(*, string=)} */
function assert(condition, text) {
  if (!condition) {
    abort('Assertion failed: ' + text);
  }
}

// Returns the C function with a specified identifier (for C++, you need to do manual name mangling)
function getCFunc(ident) {
  var func = Module['_' + ident]; // closure exported function
  assert(func, 'Cannot call unknown function ' + ident + ', make sure it is exported');
  return func;
}

// C calling interface.
/** @param {string|null=} returnType
    @param {Array=} argTypes
    @param {Arguments|Array=} args
    @param {Object=} opts */
function ccall(ident, returnType, argTypes, args, opts) {
  // For fast lookup of conversion functions
  var toC = {
    'string': function(str) {
      var ret = 0;
      if (str !== null && str !== undefined && str !== 0) { // null string
        // at most 4 bytes per UTF-8 code point, +1 for the trailing '\0'
        var len = (str.length << 2) + 1;
        ret = stackAlloc(len);
        stringToUTF8(str, ret, len);
      }
      return ret;
    },
    'array': function(arr) {
      var ret = stackAlloc(arr.length);
      writeArrayToMemory(arr, ret);
      return ret;
    }
  };

  function convertReturnValue(ret) {
    if (returnType === 'string') return UTF8ToString(ret);
    if (returnType === 'boolean') return Boolean(ret);
    return ret;
  }

  var func = getCFunc(ident);
  var cArgs = [];
  var stack = 0;
  assert(returnType !== 'array', 'Return type should not be "array".');
  if (args) {
    for (var i = 0; i < args.length; i++) {
      var converter = toC[argTypes[i]];
      if (converter) {
        if (stack === 0) stack = stackSave();
        cArgs[i] = converter(args[i]);
      } else {
        cArgs[i] = args[i];
      }
    }
  }
  var ret = func.apply(null, cArgs);

  ret = convertReturnValue(ret);
  if (stack !== 0) stackRestore(stack);
  return ret;
}

/** @param {string=} returnType
    @param {Array=} argTypes
    @param {Object=} opts */
function cwrap(ident, returnType, argTypes, opts) {
  return function() {
    return ccall(ident, returnType, argTypes, arguments, opts);
  }
}

var ALLOC_NORMAL = 0; // Tries to use _malloc()
var ALLOC_STACK = 1; // Lives for the duration of the current function call
var ALLOC_NONE = 2; // Do not allocate

// allocate(): This is for internal use. You can use it yourself as well, but the interface
//             is a little tricky (see docs right below). The reason is that it is optimized
//             for multiple syntaxes to save space in generated code. So you should
//             normally not use allocate(), and instead allocate memory using _malloc(),
//             initialize it with setValue(), and so forth.
// @slab: An array of data, or a number. If a number, then the size of the block to allocate,
//        in *bytes* (note that this is sometimes confusing: the next parameter does not
//        affect this!)
// @types: Either an array of types, one for each byte (or 0 if no type at that position),
//         or a single type which is used for the entire block. This only matters if there
//         is initial data - if @slab is a number, then this does not matter at all and is
//         ignored.
// @allocator: How to allocate memory, see ALLOC_*
/** @type {function((TypedArray|Array<number>|number), string, number, number=)} */
function allocate(slab, types, allocator, ptr) {
  var zeroinit, size;
  if (typeof slab === 'number') {
    zeroinit = true;
    size = slab;
  } else {
    zeroinit = false;
    size = slab.length;
  }

  var singleType = typeof types === 'string' ? types : null;

  var ret;
  if (allocator == ALLOC_NONE) {
    ret = ptr;
  } else {
    ret = [_malloc,
    stackAlloc,
    ][allocator](Math.max(size, singleType ? 1 : types.length));
  }

  if (zeroinit) {
    var stop;
    ptr = ret;
    assert((ret & 3) == 0);
    stop = ret + (size & ~3);
    for (; ptr < stop; ptr += 4) {
      HEAP32[((ptr)>>2)]=0;
    }
    stop = ret + size;
    while (ptr < stop) {
      HEAP8[((ptr++)>>0)]=0;
    }
    return ret;
  }

  if (singleType === 'i8') {
    if (slab.subarray || slab.slice) {
      HEAPU8.set(/** @type {!Uint8Array} */ (slab), ret);
    } else {
      HEAPU8.set(new Uint8Array(slab), ret);
    }
    return ret;
  }

  var i = 0, type, typeSize, previousType;
  while (i < size) {
    var curr = slab[i];

    type = singleType || types[i];
    if (type === 0) {
      i++;
      continue;
    }
    assert(type, 'Must know what type to store in allocate!');

    if (type == 'i64') type = 'i32'; // special case: we have one i32 here, and one i32 later

    setValue(ret+i, curr, type);

    // no need to look up size unless type changes, so cache it
    if (previousType !== type) {
      typeSize = getNativeTypeSize(type);
      previousType = type;
    }
    i += typeSize;
  }

  return ret;
}




// runtime_strings.js: Strings related runtime functions that are part of both MINIMAL_RUNTIME and regular runtime.

// Given a pointer 'ptr' to a null-terminated UTF8-encoded string in the given array that contains uint8 values, returns
// a copy of that string as a Javascript String object.

var UTF8Decoder = typeof TextDecoder !== 'undefined' ? new TextDecoder('utf8') : undefined;

/**
 * @param {number} idx
 * @param {number=} maxBytesToRead
 * @return {string}
 */
function UTF8ArrayToString(heap, idx, maxBytesToRead) {
  var endIdx = idx + maxBytesToRead;
  var endPtr = idx;
  // TextDecoder needs to know the byte length in advance, it doesn't stop on null terminator by itself.
  // Also, use the length info to avoid running tiny strings through TextDecoder, since .subarray() allocates garbage.
  // (As a tiny code save trick, compare endPtr against endIdx using a negation, so that undefined means Infinity)
  while (heap[endPtr] && !(endPtr >= endIdx)) ++endPtr;

  if (endPtr - idx > 16 && heap.subarray && UTF8Decoder) {
    return UTF8Decoder.decode(heap.subarray(idx, endPtr));
  } else {
    var str = '';
    // If building with TextDecoder, we have already computed the string length above, so test loop end condition against that
    while (idx < endPtr) {
      // For UTF8 byte structure, see:
      // http://en.wikipedia.org/wiki/UTF-8#Description
      // https://www.ietf.org/rfc/rfc2279.txt
      // https://tools.ietf.org/html/rfc3629
      var u0 = heap[idx++];
      if (!(u0 & 0x80)) { str += String.fromCharCode(u0); continue; }
      var u1 = heap[idx++] & 63;
      if ((u0 & 0xE0) == 0xC0) { str += String.fromCharCode(((u0 & 31) << 6) | u1); continue; }
      var u2 = heap[idx++] & 63;
      if ((u0 & 0xF0) == 0xE0) {
        u0 = ((u0 & 15) << 12) | (u1 << 6) | u2;
      } else {
        if ((u0 & 0xF8) != 0xF0) warnOnce('Invalid UTF-8 leading byte 0x' + u0.toString(16) + ' encountered when deserializing a UTF-8 string on the asm.js/wasm heap to a JS string!');
        u0 = ((u0 & 7) << 18) | (u1 << 12) | (u2 << 6) | (heap[idx++] & 63);
      }

      if (u0 < 0x10000) {
        str += String.fromCharCode(u0);
      } else {
        var ch = u0 - 0x10000;
        str += String.fromCharCode(0xD800 | (ch >> 10), 0xDC00 | (ch & 0x3FF));
      }
    }
  }
  return str;
}

// Given a pointer 'ptr' to a null-terminated UTF8-encoded string in the emscripten HEAP, returns a
// copy of that string as a Javascript String object.
// maxBytesToRead: an optional length that specifies the maximum number of bytes to read. You can omit
//                 this parameter to scan the string until the first \0 byte. If maxBytesToRead is
//                 passed, and the string at [ptr, ptr+maxBytesToReadr[ contains a null byte in the
//                 middle, then the string will cut short at that byte index (i.e. maxBytesToRead will
//                 not produce a string of exact length [ptr, ptr+maxBytesToRead[)
//                 N.B. mixing frequent uses of UTF8ToString() with and without maxBytesToRead may
//                 throw JS JIT optimizations off, so it is worth to consider consistently using one
//                 style or the other.
/**
 * @param {number} ptr
 * @param {number=} maxBytesToRead
 * @return {string}
 */
function UTF8ToString(ptr, maxBytesToRead) {
  return ptr ? UTF8ArrayToString(HEAPU8, ptr, maxBytesToRead) : '';
}

// Copies the given Javascript String object 'str' to the given byte array at address 'outIdx',
// encoded in UTF8 form and null-terminated. The copy will require at most str.length*4+1 bytes of space in the HEAP.
// Use the function lengthBytesUTF8 to compute the exact number of bytes (excluding null terminator) that this function will write.
// Parameters:
//   str: the Javascript string to copy.
//   heap: the array to copy to. Each index in this array is assumed to be one 8-byte element.
//   outIdx: The starting offset in the array to begin the copying.
//   maxBytesToWrite: The maximum number of bytes this function can write to the array.
//                    This count should include the null terminator,
//                    i.e. if maxBytesToWrite=1, only the null terminator will be written and nothing else.
//                    maxBytesToWrite=0 does not write any bytes to the output, not even the null terminator.
// Returns the number of bytes written, EXCLUDING the null terminator.

function stringToUTF8Array(str, heap, outIdx, maxBytesToWrite) {
  if (!(maxBytesToWrite > 0)) // Parameter maxBytesToWrite is not optional. Negative values, 0, null, undefined and false each don't write out any bytes.
    return 0;

  var startIdx = outIdx;
  var endIdx = outIdx + maxBytesToWrite - 1; // -1 for string null terminator.
  for (var i = 0; i < str.length; ++i) {
    // Gotcha: charCodeAt returns a 16-bit word that is a UTF-16 encoded code unit, not a Unicode code point of the character! So decode UTF16->UTF32->UTF8.
    // See http://unicode.org/faq/utf_bom.html#utf16-3
    // For UTF8 byte structure, see http://en.wikipedia.org/wiki/UTF-8#Description and https://www.ietf.org/rfc/rfc2279.txt and https://tools.ietf.org/html/rfc3629
    var u = str.charCodeAt(i); // possibly a lead surrogate
    if (u >= 0xD800 && u <= 0xDFFF) {
      var u1 = str.charCodeAt(++i);
      u = 0x10000 + ((u & 0x3FF) << 10) | (u1 & 0x3FF);
    }
    if (u <= 0x7F) {
      if (outIdx >= endIdx) break;
      heap[outIdx++] = u;
    } else if (u <= 0x7FF) {
      if (outIdx + 1 >= endIdx) break;
      heap[outIdx++] = 0xC0 | (u >> 6);
      heap[outIdx++] = 0x80 | (u & 63);
    } else if (u <= 0xFFFF) {
      if (outIdx + 2 >= endIdx) break;
      heap[outIdx++] = 0xE0 | (u >> 12);
      heap[outIdx++] = 0x80 | ((u >> 6) & 63);
      heap[outIdx++] = 0x80 | (u & 63);
    } else {
      if (outIdx + 3 >= endIdx) break;
      if (u >= 0x200000) warnOnce('Invalid Unicode code point 0x' + u.toString(16) + ' encountered when serializing a JS string to an UTF-8 string on the asm.js/wasm heap! (Valid unicode code points should be in range 0-0x1FFFFF).');
      heap[outIdx++] = 0xF0 | (u >> 18);
      heap[outIdx++] = 0x80 | ((u >> 12) & 63);
      heap[outIdx++] = 0x80 | ((u >> 6) & 63);
      heap[outIdx++] = 0x80 | (u & 63);
    }
  }
  // Null-terminate the pointer to the buffer.
  heap[outIdx] = 0;
  return outIdx - startIdx;
}

// Copies the given Javascript String object 'str' to the emscripten HEAP at address 'outPtr',
// null-terminated and encoded in UTF8 form. The copy will require at most str.length*4+1 bytes of space in the HEAP.
// Use the function lengthBytesUTF8 to compute the exact number of bytes (excluding null terminator) that this function will write.
// Returns the number of bytes written, EXCLUDING the null terminator.

function stringToUTF8(str, outPtr, maxBytesToWrite) {
  assert(typeof maxBytesToWrite == 'number', 'stringToUTF8(str, outPtr, maxBytesToWrite) is missing the third parameter that specifies the length of the output buffer!');
  return stringToUTF8Array(str, HEAPU8,outPtr, maxBytesToWrite);
}

// Returns the number of bytes the given Javascript string takes if encoded as a UTF8 byte array, EXCLUDING the null terminator byte.
function lengthBytesUTF8(str) {
  var len = 0;
  for (var i = 0; i < str.length; ++i) {
    // Gotcha: charCodeAt returns a 16-bit word that is a UTF-16 encoded code unit, not a Unicode code point of the character! So decode UTF16->UTF32->UTF8.
    // See http://unicode.org/faq/utf_bom.html#utf16-3
    var u = str.charCodeAt(i); // possibly a lead surrogate
    if (u >= 0xD800 && u <= 0xDFFF) u = 0x10000 + ((u & 0x3FF) << 10) | (str.charCodeAt(++i) & 0x3FF);
    if (u <= 0x7F) ++len;
    else if (u <= 0x7FF) len += 2;
    else if (u <= 0xFFFF) len += 3;
    else len += 4;
  }
  return len;
}





// runtime_strings_extra.js: Strings related runtime functions that are available only in regular runtime.

// Given a pointer 'ptr' to a null-terminated ASCII-encoded string in the emscripten HEAP, returns
// a copy of that string as a Javascript String object.

function AsciiToString(ptr) {
  var str = '';
  while (1) {
    var ch = HEAPU8[((ptr++)>>0)];
    if (!ch) return str;
    str += String.fromCharCode(ch);
  }
}

// Copies the given Javascript String object 'str' to the emscripten HEAP at address 'outPtr',
// null-terminated and encoded in ASCII form. The copy will require at most str.length+1 bytes of space in the HEAP.

function stringToAscii(str, outPtr) {
  return writeAsciiToMemory(str, outPtr, false);
}

// Given a pointer 'ptr' to a null-terminated UTF16LE-encoded string in the emscripten HEAP, returns
// a copy of that string as a Javascript String object.

var UTF16Decoder = typeof TextDecoder !== 'undefined' ? new TextDecoder('utf-16le') : undefined;

function UTF16ToString(ptr, maxBytesToRead) {
  assert(ptr % 2 == 0, 'Pointer passed to UTF16ToString must be aligned to two bytes!');
  var endPtr = ptr;
  // TextDecoder needs to know the byte length in advance, it doesn't stop on null terminator by itself.
  // Also, use the length info to avoid running tiny strings through TextDecoder, since .subarray() allocates garbage.
  var idx = endPtr >> 1;
  var maxIdx = idx + maxBytesToRead / 2;
  // If maxBytesToRead is not passed explicitly, it will be undefined, and this
  // will always evaluate to true. This saves on code size.
  while (!(idx >= maxIdx) && HEAPU16[idx]) ++idx;
  endPtr = idx << 1;

  if (endPtr - ptr > 32 && UTF16Decoder) {
    return UTF16Decoder.decode(HEAPU8.subarray(ptr, endPtr));
  } else {
    var i = 0;

    var str = '';
    while (1) {
      var codeUnit = HEAP16[(((ptr)+(i*2))>>1)];
      if (codeUnit == 0 || i == maxBytesToRead / 2) return str;
      ++i;
      // fromCharCode constructs a character from a UTF-16 code unit, so we can pass the UTF16 string right through.
      str += String.fromCharCode(codeUnit);
    }
  }
}

// Copies the given Javascript String object 'str' to the emscripten HEAP at address 'outPtr',
// null-terminated and encoded in UTF16 form. The copy will require at most str.length*4+2 bytes of space in the HEAP.
// Use the function lengthBytesUTF16() to compute the exact number of bytes (excluding null terminator) that this function will write.
// Parameters:
//   str: the Javascript string to copy.
//   outPtr: Byte address in Emscripten HEAP where to write the string to.
//   maxBytesToWrite: The maximum number of bytes this function can write to the array. This count should include the null
//                    terminator, i.e. if maxBytesToWrite=2, only the null terminator will be written and nothing else.
//                    maxBytesToWrite<2 does not write any bytes to the output, not even the null terminator.
// Returns the number of bytes written, EXCLUDING the null terminator.

function stringToUTF16(str, outPtr, maxBytesToWrite) {
  assert(outPtr % 2 == 0, 'Pointer passed to stringToUTF16 must be aligned to two bytes!');
  assert(typeof maxBytesToWrite == 'number', 'stringToUTF16(str, outPtr, maxBytesToWrite) is missing the third parameter that specifies the length of the output buffer!');
  // Backwards compatibility: if max bytes is not specified, assume unsafe unbounded write is allowed.
  if (maxBytesToWrite === undefined) {
    maxBytesToWrite = 0x7FFFFFFF;
  }
  if (maxBytesToWrite < 2) return 0;
  maxBytesToWrite -= 2; // Null terminator.
  var startPtr = outPtr;
  var numCharsToWrite = (maxBytesToWrite < str.length*2) ? (maxBytesToWrite / 2) : str.length;
  for (var i = 0; i < numCharsToWrite; ++i) {
    // charCodeAt returns a UTF-16 encoded code unit, so it can be directly written to the HEAP.
    var codeUnit = str.charCodeAt(i); // possibly a lead surrogate
    HEAP16[((outPtr)>>1)]=codeUnit;
    outPtr += 2;
  }
  // Null-terminate the pointer to the HEAP.
  HEAP16[((outPtr)>>1)]=0;
  return outPtr - startPtr;
}

// Returns the number of bytes the given Javascript string takes if encoded as a UTF16 byte array, EXCLUDING the null terminator byte.

function lengthBytesUTF16(str) {
  return str.length*2;
}

function UTF32ToString(ptr, maxBytesToRead) {
  assert(ptr % 4 == 0, 'Pointer passed to UTF32ToString must be aligned to four bytes!');
  var i = 0;

  var str = '';
  // If maxBytesToRead is not passed explicitly, it will be undefined, and this
  // will always evaluate to true. This saves on code size.
  while (!(i >= maxBytesToRead / 4)) {
    var utf32 = HEAP32[(((ptr)+(i*4))>>2)];
    if (utf32 == 0) break;
    ++i;
    // Gotcha: fromCharCode constructs a character from a UTF-16 encoded code (pair), not from a Unicode code point! So encode the code point to UTF-16 for constructing.
    // See http://unicode.org/faq/utf_bom.html#utf16-3
    if (utf32 >= 0x10000) {
      var ch = utf32 - 0x10000;
      str += String.fromCharCode(0xD800 | (ch >> 10), 0xDC00 | (ch & 0x3FF));
    } else {
      str += String.fromCharCode(utf32);
    }
  }
  return str;
}

// Copies the given Javascript String object 'str' to the emscripten HEAP at address 'outPtr',
// null-terminated and encoded in UTF32 form. The copy will require at most str.length*4+4 bytes of space in the HEAP.
// Use the function lengthBytesUTF32() to compute the exact number of bytes (excluding null terminator) that this function will write.
// Parameters:
//   str: the Javascript string to copy.
//   outPtr: Byte address in Emscripten HEAP where to write the string to.
//   maxBytesToWrite: The maximum number of bytes this function can write to the array. This count should include the null
//                    terminator, i.e. if maxBytesToWrite=4, only the null terminator will be written and nothing else.
//                    maxBytesToWrite<4 does not write any bytes to the output, not even the null terminator.
// Returns the number of bytes written, EXCLUDING the null terminator.

function stringToUTF32(str, outPtr, maxBytesToWrite) {
  assert(outPtr % 4 == 0, 'Pointer passed to stringToUTF32 must be aligned to four bytes!');
  assert(typeof maxBytesToWrite == 'number', 'stringToUTF32(str, outPtr, maxBytesToWrite) is missing the third parameter that specifies the length of the output buffer!');
  // Backwards compatibility: if max bytes is not specified, assume unsafe unbounded write is allowed.
  if (maxBytesToWrite === undefined) {
    maxBytesToWrite = 0x7FFFFFFF;
  }
  if (maxBytesToWrite < 4) return 0;
  var startPtr = outPtr;
  var endPtr = startPtr + maxBytesToWrite - 4;
  for (var i = 0; i < str.length; ++i) {
    // Gotcha: charCodeAt returns a 16-bit word that is a UTF-16 encoded code unit, not a Unicode code point of the character! We must decode the string to UTF-32 to the heap.
    // See http://unicode.org/faq/utf_bom.html#utf16-3
    var codeUnit = str.charCodeAt(i); // possibly a lead surrogate
    if (codeUnit >= 0xD800 && codeUnit <= 0xDFFF) {
      var trailSurrogate = str.charCodeAt(++i);
      codeUnit = 0x10000 + ((codeUnit & 0x3FF) << 10) | (trailSurrogate & 0x3FF);
    }
    HEAP32[((outPtr)>>2)]=codeUnit;
    outPtr += 4;
    if (outPtr + 4 > endPtr) break;
  }
  // Null-terminate the pointer to the HEAP.
  HEAP32[((outPtr)>>2)]=0;
  return outPtr - startPtr;
}

// Returns the number of bytes the given Javascript string takes if encoded as a UTF16 byte array, EXCLUDING the null terminator byte.

function lengthBytesUTF32(str) {
  var len = 0;
  for (var i = 0; i < str.length; ++i) {
    // Gotcha: charCodeAt returns a 16-bit word that is a UTF-16 encoded code unit, not a Unicode code point of the character! We must decode the string to UTF-32 to the heap.
    // See http://unicode.org/faq/utf_bom.html#utf16-3
    var codeUnit = str.charCodeAt(i);
    if (codeUnit >= 0xD800 && codeUnit <= 0xDFFF) ++i; // possibly a lead surrogate, so skip over the tail surrogate.
    len += 4;
  }

  return len;
}

// Allocate heap space for a JS string, and write it there.
// It is the responsibility of the caller to free() that memory.
function allocateUTF8(str) {
  var size = lengthBytesUTF8(str) + 1;
  var ret = _malloc(size);
  if (ret) stringToUTF8Array(str, HEAP8, ret, size);
  return ret;
}

// Allocate stack space for a JS string, and write it there.
function allocateUTF8OnStack(str) {
  var size = lengthBytesUTF8(str) + 1;
  var ret = stackAlloc(size);
  stringToUTF8Array(str, HEAP8, ret, size);
  return ret;
}

// Deprecated: This function should not be called because it is unsafe and does not provide
// a maximum length limit of how many bytes it is allowed to write. Prefer calling the
// function stringToUTF8Array() instead, which takes in a maximum length that can be used
// to be secure from out of bounds writes.
/** @deprecated
    @param {boolean=} dontAddNull */
function writeStringToMemory(string, buffer, dontAddNull) {
  warnOnce('writeStringToMemory is deprecated and should not be called! Use stringToUTF8() instead!');

  var /** @type {number} */ lastChar, /** @type {number} */ end;
  if (dontAddNull) {
    // stringToUTF8Array always appends null. If we don't want to do that, remember the
    // character that existed at the location where the null will be placed, and restore
    // that after the write (below).
    end = buffer + lengthBytesUTF8(string);
    lastChar = HEAP8[end];
  }
  stringToUTF8(string, buffer, Infinity);
  if (dontAddNull) HEAP8[end] = lastChar; // Restore the value under the null character.
}

function writeArrayToMemory(array, buffer) {
  assert(array.length >= 0, 'writeArrayToMemory array must have a length (should be an array or typed array)')
  HEAP8.set(array, buffer);
}

/** @param {boolean=} dontAddNull */
function writeAsciiToMemory(str, buffer, dontAddNull) {
  for (var i = 0; i < str.length; ++i) {
    assert(str.charCodeAt(i) === str.charCodeAt(i)&0xff);
    HEAP8[((buffer++)>>0)]=str.charCodeAt(i);
  }
  // Null-terminate the pointer to the HEAP.
  if (!dontAddNull) HEAP8[((buffer)>>0)]=0;
}



// Memory management

var PAGE_SIZE = 16384;
var WASM_PAGE_SIZE = 65536;

function alignUp(x, multiple) {
  if (x % multiple > 0) {
    x += multiple - (x % multiple);
  }
  return x;
}

var HEAP,
/** @type {ArrayBuffer} */
  buffer,
/** @type {Int8Array} */
  HEAP8,
/** @type {Uint8Array} */
  HEAPU8,
/** @type {Int16Array} */
  HEAP16,
/** @type {Uint16Array} */
  HEAPU16,
/** @type {Int32Array} */
  HEAP32,
/** @type {Uint32Array} */
  HEAPU32,
/** @type {Float32Array} */
  HEAPF32,
/** @type {Float64Array} */
  HEAPF64;

function updateGlobalBufferAndViews(buf) {
  buffer = buf;
  Module['HEAP8'] = HEAP8 = new Int8Array(buf);
  Module['HEAP16'] = HEAP16 = new Int16Array(buf);
  Module['HEAP32'] = HEAP32 = new Int32Array(buf);
  Module['HEAPU8'] = HEAPU8 = new Uint8Array(buf);
  Module['HEAPU16'] = HEAPU16 = new Uint16Array(buf);
  Module['HEAPU32'] = HEAPU32 = new Uint32Array(buf);
  Module['HEAPF32'] = HEAPF32 = new Float32Array(buf);
  Module['HEAPF64'] = HEAPF64 = new Float64Array(buf);
}

var STATIC_BASE = 1024,
    STACK_BASE = 5248128,
    STACKTOP = STACK_BASE,
    STACK_MAX = 5248,
    DYNAMIC_BASE = 5248128;

assert(STACK_BASE % 16 === 0, 'stack must start aligned');
assert(DYNAMIC_BASE % 16 === 0, 'heap must start aligned');



var TOTAL_STACK = 5242880;
if (Module['TOTAL_STACK']) assert(TOTAL_STACK === Module['TOTAL_STACK'], 'the stack size can no longer be determined at runtime')

var INITIAL_INITIAL_MEMORY = Module['INITIAL_MEMORY'] || 16777216;if (!Object.getOwnPropertyDescriptor(Module, 'INITIAL_MEMORY')) Object.defineProperty(Module, 'INITIAL_MEMORY', { configurable: true, get: function() { abort('Module.INITIAL_MEMORY has been replaced with plain INITIAL_INITIAL_MEMORY (the initial value can be provided on Module, but after startup the value is only looked for on a local variable of that name)') } });

assert(INITIAL_INITIAL_MEMORY >= TOTAL_STACK, 'INITIAL_MEMORY should be larger than TOTAL_STACK, was ' + INITIAL_INITIAL_MEMORY + '! (TOTAL_STACK=' + TOTAL_STACK + ')');

// check for full engine support (use string 'subarray' to avoid closure compiler confusion)
assert(typeof Int32Array !== 'undefined' && typeof Float64Array !== 'undefined' && Int32Array.prototype.subarray !== undefined && Int32Array.prototype.set !== undefined,
       'JS engine does not provide full typed array support');


// In non-standalone/normal mode, we create the memory here.



// Create the main memory. (Note: this isn't used in STANDALONE_WASM mode since the wasm
// memory is created in the wasm, not in JS.)

  if (Module['wasmMemory']) {
    wasmMemory = Module['wasmMemory'];
  } else
  {
    wasmMemory = new WebAssembly.Memory({
      'initial': INITIAL_INITIAL_MEMORY / WASM_PAGE_SIZE
      ,
      'maximum': INITIAL_INITIAL_MEMORY / WASM_PAGE_SIZE
    });
  }


if (wasmMemory) {
  buffer = wasmMemory.buffer;
}

// If the user provides an incorrect length, just use that length instead rather than providing the user to
// specifically provide the memory length with Module['INITIAL_MEMORY'].
INITIAL_INITIAL_MEMORY = buffer.byteLength;
assert(INITIAL_INITIAL_MEMORY % WASM_PAGE_SIZE === 0);
updateGlobalBufferAndViews(buffer);







// Initializes the stack cookie. Called at the startup of main and at the startup of each thread in pthreads mode.
function writeStackCookie() {
  assert((STACK_MAX & 3) == 0);
  // The stack grows downwards
  HEAPU32[(STACK_MAX >> 2)+1] = 0x2135467;
  HEAPU32[(STACK_MAX >> 2)+2] = 0x89BACDFE;
  // Also test the global address 0 for integrity.
  // We don't do this with ASan because ASan does its own checks for this.
  HEAP32[0] = 0x63736d65; /* 'emsc' */
}

function checkStackCookie() {
  var cookie1 = HEAPU32[(STACK_MAX >> 2)+1];
  var cookie2 = HEAPU32[(STACK_MAX >> 2)+2];
  if (cookie1 != 0x2135467 || cookie2 != 0x89BACDFE) {
    abort('Stack overflow! Stack cookie has been overwritten, expected hex dwords 0x89BACDFE and 0x2135467, but received 0x' + cookie2.toString(16) + ' ' + cookie1.toString(16));
  }
  // Also test the global address 0 for integrity.
  // We don't do this with ASan because ASan does its own checks for this.
  if (HEAP32[0] !== 0x63736d65 /* 'emsc' */) abort('Runtime error: The application has corrupted its heap memory area (address zero)!');
}





// Endianness check (note: assumes compiler arch was little-endian)
(function() {
  var h16 = new Int16Array(1);
  var h8 = new Int8Array(h16.buffer);
  h16[0] = 0x6373;
  if (h8[0] !== 0x73 || h8[1] !== 0x63) throw 'Runtime error: expected the system to be little-endian!';
})();

function abortFnPtrError(ptr, sig) {
	abort("Invalid function pointer " + ptr + " called with signature '" + sig + "'. Perhaps this is an invalid value (e.g. caused by calling a virtual method on a NULL pointer)? Or calling a function with an incorrect type, which will fail? (it is worth building your source files with -Werror (warnings are errors), as warnings can indicate undefined behavior which can cause this). Build with ASSERTIONS=2 for more info.");
}



var __ATPRERUN__  = []; // functions called before the runtime is initialized
var __ATINIT__    = []; // functions called during startup
var __ATMAIN__    = []; // functions called when main() is to be run
var __ATEXIT__    = []; // functions called during shutdown
var __ATPOSTRUN__ = []; // functions called after the main() is called

var runtimeInitialized = false;
var runtimeExited = false;


function preRun() {

  if (Module['preRun']) {
    if (typeof Module['preRun'] == 'function') Module['preRun'] = [Module['preRun']];
    while (Module['preRun'].length) {
      addOnPreRun(Module['preRun'].shift());
    }
  }

  callRuntimeCallbacks(__ATPRERUN__);
}

function initRuntime() {
  checkStackCookie();
  assert(!runtimeInitialized);
  runtimeInitialized = true;
  
  callRuntimeCallbacks(__ATINIT__);
}

function preMain() {
  checkStackCookie();
  
  callRuntimeCallbacks(__ATMAIN__);
}

function exitRuntime() {
  checkStackCookie();
  runtimeExited = true;
}

function postRun() {
  checkStackCookie();

  if (Module['postRun']) {
    if (typeof Module['postRun'] == 'function') Module['postRun'] = [Module['postRun']];
    while (Module['postRun'].length) {
      addOnPostRun(Module['postRun'].shift());
    }
  }

  callRuntimeCallbacks(__ATPOSTRUN__);
}

function addOnPreRun(cb) {
  __ATPRERUN__.unshift(cb);
}

function addOnInit(cb) {
  __ATINIT__.unshift(cb);
}

function addOnPreMain(cb) {
  __ATMAIN__.unshift(cb);
}

function addOnExit(cb) {
}

function addOnPostRun(cb) {
  __ATPOSTRUN__.unshift(cb);
}




// https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Math/imul

// https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Math/fround

// https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Math/clz32

// https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Math/trunc

assert(Math.imul, 'This browser does not support Math.imul(), build with LEGACY_VM_SUPPORT or POLYFILL_OLD_MATH_FUNCTIONS to add in a polyfill');
assert(Math.fround, 'This browser does not support Math.fround(), build with LEGACY_VM_SUPPORT or POLYFILL_OLD_MATH_FUNCTIONS to add in a polyfill');
assert(Math.clz32, 'This browser does not support Math.clz32(), build with LEGACY_VM_SUPPORT or POLYFILL_OLD_MATH_FUNCTIONS to add in a polyfill');
assert(Math.trunc, 'This browser does not support Math.trunc(), build with LEGACY_VM_SUPPORT or POLYFILL_OLD_MATH_FUNCTIONS to add in a polyfill');

var Math_abs = Math.abs;
var Math_cos = Math.cos;
var Math_sin = Math.sin;
var Math_tan = Math.tan;
var Math_acos = Math.acos;
var Math_asin = Math.asin;
var Math_atan = Math.atan;
var Math_atan2 = Math.atan2;
var Math_exp = Math.exp;
var Math_log = Math.log;
var Math_sqrt = Math.sqrt;
var Math_ceil = Math.ceil;
var Math_floor = Math.floor;
var Math_pow = Math.pow;
var Math_imul = Math.imul;
var Math_fround = Math.fround;
var Math_round = Math.round;
var Math_min = Math.min;
var Math_max = Math.max;
var Math_clz32 = Math.clz32;
var Math_trunc = Math.trunc;



// A counter of dependencies for calling run(). If we need to
// do asynchronous work before running, increment this and
// decrement it. Incrementing must happen in a place like
// Module.preRun (used by emcc to add file preloading).
// Note that you can add dependencies in preRun, even though
// it happens right before run - run will be postponed until
// the dependencies are met.
var runDependencies = 0;
var runDependencyWatcher = null;
var dependenciesFulfilled = null; // overridden to take different actions when all run dependencies are fulfilled
var runDependencyTracking = {};

function getUniqueRunDependency(id) {
  var orig = id;
  while (1) {
    if (!runDependencyTracking[id]) return id;
    id = orig + Math.random();
  }
}

function addRunDependency(id) {
  runDependencies++;

  if (Module['monitorRunDependencies']) {
    Module['monitorRunDependencies'](runDependencies);
  }

  if (id) {
    assert(!runDependencyTracking[id]);
    runDependencyTracking[id] = 1;
    if (runDependencyWatcher === null && typeof setInterval !== 'undefined') {
      // Check for missing dependencies every few seconds
      runDependencyWatcher = setInterval(function() {
        if (ABORT) {
          clearInterval(runDependencyWatcher);
          runDependencyWatcher = null;
          return;
        }
        var shown = false;
        for (var dep in runDependencyTracking) {
          if (!shown) {
            shown = true;
            err('still waiting on run dependencies:');
          }
          err('dependency: ' + dep);
        }
        if (shown) {
          err('(end of list)');
        }
      }, 10000);
    }
  } else {
    err('warning: run dependency added without ID');
  }
}

function removeRunDependency(id) {
  runDependencies--;

  if (Module['monitorRunDependencies']) {
    Module['monitorRunDependencies'](runDependencies);
  }

  if (id) {
    assert(runDependencyTracking[id]);
    delete runDependencyTracking[id];
  } else {
    err('warning: run dependency removed without ID');
  }
  if (runDependencies == 0) {
    if (runDependencyWatcher !== null) {
      clearInterval(runDependencyWatcher);
      runDependencyWatcher = null;
    }
    if (dependenciesFulfilled) {
      var callback = dependenciesFulfilled;
      dependenciesFulfilled = null;
      callback(); // can add another dependenciesFulfilled
    }
  }
}

Module["preloadedImages"] = {}; // maps url to image data
Module["preloadedAudios"] = {}; // maps url to audio data

/** @param {string|number=} what */
function abort(what) {
  if (Module['onAbort']) {
    Module['onAbort'](what);
  }

  what += '';
  err(what);

  ABORT = true;
  EXITSTATUS = 1;

  var output = 'abort(' + what + ') at ' + stackTrace();
  what = output;

  // Use a wasm runtime error, because a JS error might be seen as a foreign
  // exception, which means we'd run destructors on it. We need the error to
  // simply make the program stop.
  var e = new WebAssembly.RuntimeError(what);

  // Throw the error whether or not MODULARIZE is set because abort is used
  // in code paths apart from instantiation where an exception is expected
  // to be thrown when abort is called.
  throw e;
}

var memoryInitializer = null;







// show errors on likely calls to FS when it was not included
var FS = {
  error: function() {
    abort('Filesystem support (FS) was not included. The problem is that you are using files from JS, but files were not used from C/C++, so filesystem support was not auto-included. You can force-include filesystem support with  -s FORCE_FILESYSTEM=1');
  },
  init: function() { FS.error() },
  createDataFile: function() { FS.error() },
  createPreloadedFile: function() { FS.error() },
  createLazyFile: function() { FS.error() },
  open: function() { FS.error() },
  mkdev: function() { FS.error() },
  registerDevice: function() { FS.error() },
  analyzePath: function() { FS.error() },
  loadFilesFromDB: function() { FS.error() },

  ErrnoError: function ErrnoError() { FS.error() },
};
Module['FS_createDataFile'] = FS.createDataFile;
Module['FS_createPreloadedFile'] = FS.createPreloadedFile;




function hasPrefix(str, prefix) {
  return String.prototype.startsWith ?
      str.startsWith(prefix) :
      str.indexOf(prefix) === 0;
}

// Prefix of data URIs emitted by SINGLE_FILE and related options.
var dataURIPrefix = 'data:application/octet-stream;base64,';

// Indicates whether filename is a base64 data URI.
function isDataURI(filename) {
  return hasPrefix(filename, dataURIPrefix);
}

var fileURIPrefix = "file://";

// Indicates whether filename is delivered via file protocol (as opposed to http/https)
function isFileURI(filename) {
  return hasPrefix(filename, fileURIPrefix);
}



function createExportWrapper(name, fixedasm) {
  return function() {
    var displayName = name;
    var asm = fixedasm;
    if (!fixedasm) {
      asm = Module['asm'];
    }
    assert(runtimeInitialized, 'native function `' + displayName + '` called before runtime initialization');
    assert(!runtimeExited, 'native function `' + displayName + '` called after runtime exit (use NO_EXIT_RUNTIME to keep it alive after main() exits)');
    if (!asm[name]) {
      assert(asm[name], 'exported native function `' + displayName + '` not found');
    }
    return asm[name].apply(null, arguments);
  };
}


var wasmBinaryFile = 'my_lib.wasm';
if (!isDataURI(wasmBinaryFile)) {
  wasmBinaryFile = locateFile(wasmBinaryFile);
}

function getBinary() {
  try {
    if (wasmBinary) {
      return new Uint8Array(wasmBinary);
    }

    var binary = tryParseAsDataURI(wasmBinaryFile);
    if (binary) {
      return binary;
    }
    if (readBinary) {
      return readBinary(wasmBinaryFile);
    } else {
      throw "both async and sync fetching of the wasm failed";
    }
  }
  catch (err) {
    abort(err);
  }
}

function getBinaryPromise() {
  // If we don't have the binary yet, and have the Fetch api, use that;
  // in some environments, like Electron's render process, Fetch api may be present, but have a different context than expected, let's only use it on the Web
  if (!wasmBinary && (ENVIRONMENT_IS_WEB || ENVIRONMENT_IS_WORKER) && typeof fetch === 'function'
      // Let's not use fetch to get objects over file:// as it's most likely Cordova which doesn't support fetch for file://
      && !isFileURI(wasmBinaryFile)
      ) {
    return fetch(wasmBinaryFile, { credentials: 'same-origin' }).then(function(response) {
      if (!response['ok']) {
        throw "failed to load wasm binary file at '" + wasmBinaryFile + "'";
      }
      return response['arrayBuffer']();
    }).catch(function () {
      return getBinary();
    });
  }
  // Otherwise, getBinary should be able to get it synchronously
  return Promise.resolve().then(getBinary);
}



// Create the wasm instance.
// Receives the wasm imports, returns the exports.
function createWasm() {
  // prepare imports
  var info = {
    'env': asmLibraryArg,
    'wasi_snapshot_preview1': asmLibraryArg
  };
  // Load the wasm module and create an instance of using native support in the JS engine.
  // handle a generated wasm instance, receiving its exports and
  // performing other necessary setup
  /** @param {WebAssembly.Module=} module*/
  function receiveInstance(instance, module) {
    var exports = instance.exports;
    Module['asm'] = exports;
    removeRunDependency('wasm-instantiate');
  }
  // we can't run yet (except in a pthread, where we have a custom sync instantiator)
  addRunDependency('wasm-instantiate');


  // Async compilation can be confusing when an error on the page overwrites Module
  // (for example, if the order of elements is wrong, and the one defining Module is
  // later), so we save Module and check it later.
  var trueModule = Module;
  function receiveInstantiatedSource(output) {
    // 'output' is a WebAssemblyInstantiatedSource object which has both the module and instance.
    // receiveInstance() will swap in the exports (to Module.asm) so they can be called
    assert(Module === trueModule, 'the Module object should not be replaced during async compilation - perhaps the order of HTML elements is wrong?');
    trueModule = null;
    // TODO: Due to Closure regression https://github.com/google/closure-compiler/issues/3193, the above line no longer optimizes out down to the following line.
    // When the regression is fixed, can restore the above USE_PTHREADS-enabled path.
    receiveInstance(output['instance']);
  }


  function instantiateArrayBuffer(receiver) {
    return getBinaryPromise().then(function(binary) {
      return WebAssembly.instantiate(binary, info);
    }).then(receiver, function(reason) {
      err('failed to asynchronously prepare wasm: ' + reason);


      abort(reason);
    });
  }

  // Prefer streaming instantiation if available.
  function instantiateAsync() {
    if (!wasmBinary &&
        typeof WebAssembly.instantiateStreaming === 'function' &&
        !isDataURI(wasmBinaryFile) &&
        // Don't use streaming for file:// delivered objects in a webview, fetch them synchronously.
        !isFileURI(wasmBinaryFile) &&
        typeof fetch === 'function') {
      fetch(wasmBinaryFile, { credentials: 'same-origin' }).then(function (response) {
        var result = WebAssembly.instantiateStreaming(response, info);
        return result.then(receiveInstantiatedSource, function(reason) {
            // We expect the most common failure cause to be a bad MIME type for the binary,
            // in which case falling back to ArrayBuffer instantiation should work.
            err('wasm streaming compile failed: ' + reason);
            err('falling back to ArrayBuffer instantiation');
            return instantiateArrayBuffer(receiveInstantiatedSource);
          });
      });
    } else {
      return instantiateArrayBuffer(receiveInstantiatedSource);
    }
  }
  // User shell pages can write their own Module.instantiateWasm = function(imports, successCallback) callback
  // to manually instantiate the Wasm module themselves. This allows pages to run the instantiation parallel
  // to any other async startup actions they are performing.
  if (Module['instantiateWasm']) {
    try {
      var exports = Module['instantiateWasm'](info, receiveInstance);
      return exports;
    } catch(e) {
      err('Module.instantiateWasm callback failed with error: ' + e);
      return false;
    }
  }

  instantiateAsync();
  return {}; // no exports yet; we'll fill them in later
}

// Globals used by JS i64 conversions
var tempDouble;
var tempI64;

// === Body ===

var ASM_CONSTS = {
  
};




// STATICTOP = STATIC_BASE + 4224;
/* global initializers */  __ATINIT__.push({ func: function() { ___wasm_call_ctors() } });




/* no memory initializer */
// {{PRE_LIBRARY}}


  function abortStackOverflow(allocSize) {
      abort('Stack overflow! Attempted to allocate ' + allocSize + ' bytes on the stack, but stack has only ' + (STACK_MAX - stackSave() + allocSize) + ' bytes available!');
    }

  function callRuntimeCallbacks(callbacks) {
      while(callbacks.length > 0) {
        var callback = callbacks.shift();
        if (typeof callback == 'function') {
          callback(Module); // Pass the module as the first argument.
          continue;
        }
        var func = callback.func;
        if (typeof func === 'number') {
          if (callback.arg === undefined) {
            wasmTable.get(func)();
          } else {
            wasmTable.get(func)(callback.arg);
          }
        } else {
          func(callback.arg === undefined ? null : callback.arg);
        }
      }
    }

  function demangle(func) {
      warnOnce('warning: build with  -s DEMANGLE_SUPPORT=1  to link in libcxxabi demangling');
      return func;
    }

  function demangleAll(text) {
      var regex =
        /\b_Z[\w\d_]+/g;
      return text.replace(regex,
        function(x) {
          var y = demangle(x);
          return x === y ? x : (y + ' [' + x + ']');
        });
    }

  
  function dynCallLegacy(sig, ptr, args) {
      assert(('dynCall_' + sig) in Module, 'bad function pointer type - no table for sig \'' + sig + '\'');
      if (args && args.length) {
        // j (64-bit integer) must be passed in as two numbers [low 32, high 32].
        assert(args.length === sig.substring(1).replace(/j/g, '--').length);
      } else {
        assert(sig.length == 1);
      }
      if (args && args.length) {
        return Module['dynCall_' + sig].apply(null, [ptr].concat(args));
      }
      return Module['dynCall_' + sig].call(null, ptr);
    }function dynCall(sig, ptr, args) {
      // Without WASM_BIGINT support we cannot directly call function with i64 as
      // part of thier signature, so we rely the dynCall functions generated by
      // wasm-emscripten-finalize
      if (sig.indexOf('j') != -1) {
        return dynCallLegacy(sig, ptr, args);
      }
  
      return wasmTable.get(ptr).apply(null, args)
    }

  function jsStackTrace() {
      var error = new Error();
      if (!error.stack) {
        // IE10+ special cases: It does have callstack info, but it is only populated if an Error object is thrown,
        // so try that as a special-case.
        try {
          throw new Error();
        } catch(e) {
          error = e;
        }
        if (!error.stack) {
          return '(no stack trace available)';
        }
      }
      return error.stack.toString();
    }

  function stackTrace() {
      var js = jsStackTrace();
      if (Module['extraStackTrace']) js += '\n' + Module['extraStackTrace']();
      return demangleAll(js);
    }

  function ___assert_fail(condition, filename, line, func) {
      abort('Assertion failed: ' + UTF8ToString(condition) + ', at: ' + [filename ? UTF8ToString(filename) : 'unknown filename', line, func ? UTF8ToString(func) : 'unknown function']);
    }

  
  function getShiftFromSize(size) {
      switch (size) {
          case 1: return 0;
          case 2: return 1;
          case 4: return 2;
          case 8: return 3;
          default:
              throw new TypeError('Unknown type size: ' + size);
      }
    }
  
  
  
  function embind_init_charCodes() {
      var codes = new Array(256);
      for (var i = 0; i < 256; ++i) {
          codes[i] = String.fromCharCode(i);
      }
      embind_charCodes = codes;
    }var embind_charCodes=undefined;function readLatin1String(ptr) {
      var ret = "";
      var c = ptr;
      while (HEAPU8[c]) {
          ret += embind_charCodes[HEAPU8[c++]];
      }
      return ret;
    }
  
  
  var awaitingDependencies={};
  
  var registeredTypes={};
  
  var typeDependencies={};
  
  
  
  
  
  
  var char_0=48;
  
  var char_9=57;function makeLegalFunctionName(name) {
      if (undefined === name) {
          return '_unknown';
      }
      name = name.replace(/[^a-zA-Z0-9_]/g, '$');
      var f = name.charCodeAt(0);
      if (f >= char_0 && f <= char_9) {
          return '_' + name;
      } else {
          return name;
      }
    }function createNamedFunction(name, body) {
      name = makeLegalFunctionName(name);
      /*jshint evil:true*/
      return new Function(
          "body",
          "return function " + name + "() {\n" +
          "    \"use strict\";" +
          "    return body.apply(this, arguments);\n" +
          "};\n"
      )(body);
    }function extendError(baseErrorType, errorName) {
      var errorClass = createNamedFunction(errorName, function(message) {
          this.name = errorName;
          this.message = message;
  
          var stack = (new Error(message)).stack;
          if (stack !== undefined) {
              this.stack = this.toString() + '\n' +
                  stack.replace(/^Error(:[^\n]*)?\n/, '');
          }
      });
      errorClass.prototype = Object.create(baseErrorType.prototype);
      errorClass.prototype.constructor = errorClass;
      errorClass.prototype.toString = function() {
          if (this.message === undefined) {
              return this.name;
          } else {
              return this.name + ': ' + this.message;
          }
      };
  
      return errorClass;
    }var BindingError=undefined;function throwBindingError(message) {
      throw new BindingError(message);
    }
  
  
  
  var InternalError=undefined;function throwInternalError(message) {
      throw new InternalError(message);
    }function whenDependentTypesAreResolved(myTypes, dependentTypes, getTypeConverters) {
      myTypes.forEach(function(type) {
          typeDependencies[type] = dependentTypes;
      });
  
      function onComplete(typeConverters) {
          var myTypeConverters = getTypeConverters(typeConverters);
          if (myTypeConverters.length !== myTypes.length) {
              throwInternalError('Mismatched type converter count');
          }
          for (var i = 0; i < myTypes.length; ++i) {
              registerType(myTypes[i], myTypeConverters[i]);
          }
      }
  
      var typeConverters = new Array(dependentTypes.length);
      var unregisteredTypes = [];
      var registered = 0;
      dependentTypes.forEach(function(dt, i) {
          if (registeredTypes.hasOwnProperty(dt)) {
              typeConverters[i] = registeredTypes[dt];
          } else {
              unregisteredTypes.push(dt);
              if (!awaitingDependencies.hasOwnProperty(dt)) {
                  awaitingDependencies[dt] = [];
              }
              awaitingDependencies[dt].push(function() {
                  typeConverters[i] = registeredTypes[dt];
                  ++registered;
                  if (registered === unregisteredTypes.length) {
                      onComplete(typeConverters);
                  }
              });
          }
      });
      if (0 === unregisteredTypes.length) {
          onComplete(typeConverters);
      }
    }/** @param {Object=} options */
  function registerType(rawType, registeredInstance, options) {
      options = options || {};
  
      if (!('argPackAdvance' in registeredInstance)) {
          throw new TypeError('registerType registeredInstance requires argPackAdvance');
      }
  
      var name = registeredInstance.name;
      if (!rawType) {
          throwBindingError('type "' + name + '" must have a positive integer typeid pointer');
      }
      if (registeredTypes.hasOwnProperty(rawType)) {
          if (options.ignoreDuplicateRegistrations) {
              return;
          } else {
              throwBindingError("Cannot register type '" + name + "' twice");
          }
      }
  
      registeredTypes[rawType] = registeredInstance;
      delete typeDependencies[rawType];
  
      if (awaitingDependencies.hasOwnProperty(rawType)) {
          var callbacks = awaitingDependencies[rawType];
          delete awaitingDependencies[rawType];
          callbacks.forEach(function(cb) {
              cb();
          });
      }
    }function __embind_register_bool(rawType, name, size, trueValue, falseValue) {
      var shift = getShiftFromSize(size);
  
      name = readLatin1String(name);
      registerType(rawType, {
          name: name,
          'fromWireType': function(wt) {
              // ambiguous emscripten ABI: sometimes return values are
              // true or false, and sometimes integers (0 or 1)
              return !!wt;
          },
          'toWireType': function(destructors, o) {
              return o ? trueValue : falseValue;
          },
          'argPackAdvance': 8,
          'readValueFromPointer': function(pointer) {
              // TODO: if heap is fixed (like in asm.js) this could be executed outside
              var heap;
              if (size === 1) {
                  heap = HEAP8;
              } else if (size === 2) {
                  heap = HEAP16;
              } else if (size === 4) {
                  heap = HEAP32;
              } else {
                  throw new TypeError("Unknown boolean type size: " + name);
              }
              return this['fromWireType'](heap[pointer >> shift]);
          },
          destructorFunction: null, // This type does not need a destructor
      });
    }

  
  
  
  function ClassHandle_isAliasOf(other) {
      if (!(this instanceof ClassHandle)) {
          return false;
      }
      if (!(other instanceof ClassHandle)) {
          return false;
      }
  
      var leftClass = this.$$.ptrType.registeredClass;
      var left = this.$$.ptr;
      var rightClass = other.$$.ptrType.registeredClass;
      var right = other.$$.ptr;
  
      while (leftClass.baseClass) {
          left = leftClass.upcast(left);
          leftClass = leftClass.baseClass;
      }
  
      while (rightClass.baseClass) {
          right = rightClass.upcast(right);
          rightClass = rightClass.baseClass;
      }
  
      return leftClass === rightClass && left === right;
    }
  
  
  function shallowCopyInternalPointer(o) {
      return {
          count: o.count,
          deleteScheduled: o.deleteScheduled,
          preservePointerOnDelete: o.preservePointerOnDelete,
          ptr: o.ptr,
          ptrType: o.ptrType,
          smartPtr: o.smartPtr,
          smartPtrType: o.smartPtrType,
      };
    }
  
  function throwInstanceAlreadyDeleted(obj) {
      function getInstanceTypeName(handle) {
        return handle.$$.ptrType.registeredClass.name;
      }
      throwBindingError(getInstanceTypeName(obj) + ' instance already deleted');
    }
  
  
  var finalizationGroup=false;
  
  function detachFinalizer(handle) {}
  
  
  function runDestructor($$) {
      if ($$.smartPtr) {
          $$.smartPtrType.rawDestructor($$.smartPtr);
      } else {
          $$.ptrType.registeredClass.rawDestructor($$.ptr);
      }
    }function releaseClassHandle($$) {
      $$.count.value -= 1;
      var toDelete = 0 === $$.count.value;
      if (toDelete) {
          runDestructor($$);
      }
    }function attachFinalizer(handle) {
      if ('undefined' === typeof FinalizationGroup) {
          attachFinalizer = function (handle) { return handle; };
          return handle;
      }
      // If the running environment has a FinalizationGroup (see
      // https://github.com/tc39/proposal-weakrefs), then attach finalizers
      // for class handles.  We check for the presence of FinalizationGroup
      // at run-time, not build-time.
      finalizationGroup = new FinalizationGroup(function (iter) {
          for (var result = iter.next(); !result.done; result = iter.next()) {
              var $$ = result.value;
              if (!$$.ptr) {
                  console.warn('object already deleted: ' + $$.ptr);
              } else {
                  releaseClassHandle($$);
              }
          }
      });
      attachFinalizer = function(handle) {
          finalizationGroup.register(handle, handle.$$, handle.$$);
          return handle;
      };
      detachFinalizer = function(handle) {
          finalizationGroup.unregister(handle.$$);
      };
      return attachFinalizer(handle);
    }function ClassHandle_clone() {
      if (!this.$$.ptr) {
          throwInstanceAlreadyDeleted(this);
      }
  
      if (this.$$.preservePointerOnDelete) {
          this.$$.count.value += 1;
          return this;
      } else {
          var clone = attachFinalizer(Object.create(Object.getPrototypeOf(this), {
              $$: {
                  value: shallowCopyInternalPointer(this.$$),
              }
          }));
  
          clone.$$.count.value += 1;
          clone.$$.deleteScheduled = false;
          return clone;
      }
    }
  
  function ClassHandle_delete() {
      if (!this.$$.ptr) {
          throwInstanceAlreadyDeleted(this);
      }
  
      if (this.$$.deleteScheduled && !this.$$.preservePointerOnDelete) {
          throwBindingError('Object already scheduled for deletion');
      }
  
      detachFinalizer(this);
      releaseClassHandle(this.$$);
  
      if (!this.$$.preservePointerOnDelete) {
          this.$$.smartPtr = undefined;
          this.$$.ptr = undefined;
      }
    }
  
  function ClassHandle_isDeleted() {
      return !this.$$.ptr;
    }
  
  
  var delayFunction=undefined;
  
  var deletionQueue=[];
  
  function flushPendingDeletes() {
      while (deletionQueue.length) {
          var obj = deletionQueue.pop();
          obj.$$.deleteScheduled = false;
          obj['delete']();
      }
    }function ClassHandle_deleteLater() {
      if (!this.$$.ptr) {
          throwInstanceAlreadyDeleted(this);
      }
      if (this.$$.deleteScheduled && !this.$$.preservePointerOnDelete) {
          throwBindingError('Object already scheduled for deletion');
      }
      deletionQueue.push(this);
      if (deletionQueue.length === 1 && delayFunction) {
          delayFunction(flushPendingDeletes);
      }
      this.$$.deleteScheduled = true;
      return this;
    }function init_ClassHandle() {
      ClassHandle.prototype['isAliasOf'] = ClassHandle_isAliasOf;
      ClassHandle.prototype['clone'] = ClassHandle_clone;
      ClassHandle.prototype['delete'] = ClassHandle_delete;
      ClassHandle.prototype['isDeleted'] = ClassHandle_isDeleted;
      ClassHandle.prototype['deleteLater'] = ClassHandle_deleteLater;
    }function ClassHandle() {
    }
  
  var registeredPointers={};
  
  
  function ensureOverloadTable(proto, methodName, humanName) {
      if (undefined === proto[methodName].overloadTable) {
          var prevFunc = proto[methodName];
          // Inject an overload resolver function that routes to the appropriate overload based on the number of arguments.
          proto[methodName] = function() {
              // TODO This check can be removed in -O3 level "unsafe" optimizations.
              if (!proto[methodName].overloadTable.hasOwnProperty(arguments.length)) {
                  throwBindingError("Function '" + humanName + "' called with an invalid number of arguments (" + arguments.length + ") - expects one of (" + proto[methodName].overloadTable + ")!");
              }
              return proto[methodName].overloadTable[arguments.length].apply(this, arguments);
          };
          // Move the previous function into the overload table.
          proto[methodName].overloadTable = [];
          proto[methodName].overloadTable[prevFunc.argCount] = prevFunc;
      }
    }/** @param {number=} numArguments */
  function exposePublicSymbol(name, value, numArguments) {
      if (Module.hasOwnProperty(name)) {
          if (undefined === numArguments || (undefined !== Module[name].overloadTable && undefined !== Module[name].overloadTable[numArguments])) {
              throwBindingError("Cannot register public name '" + name + "' twice");
          }
  
          // We are exposing a function with the same name as an existing function. Create an overload table and a function selector
          // that routes between the two.
          ensureOverloadTable(Module, name, name);
          if (Module.hasOwnProperty(numArguments)) {
              throwBindingError("Cannot register multiple overloads of a function with the same number of arguments (" + numArguments + ")!");
          }
          // Add the new function into the overload table.
          Module[name].overloadTable[numArguments] = value;
      }
      else {
          Module[name] = value;
          if (undefined !== numArguments) {
              Module[name].numArguments = numArguments;
          }
      }
    }
  
  /** @constructor */
  function RegisteredClass(
      name,
      constructor,
      instancePrototype,
      rawDestructor,
      baseClass,
      getActualType,
      upcast,
      downcast
    ) {
      this.name = name;
      this.constructor = constructor;
      this.instancePrototype = instancePrototype;
      this.rawDestructor = rawDestructor;
      this.baseClass = baseClass;
      this.getActualType = getActualType;
      this.upcast = upcast;
      this.downcast = downcast;
      this.pureVirtualFunctions = [];
    }
  
  
  
  function upcastPointer(ptr, ptrClass, desiredClass) {
      while (ptrClass !== desiredClass) {
          if (!ptrClass.upcast) {
              throwBindingError("Expected null or instance of " + desiredClass.name + ", got an instance of " + ptrClass.name);
          }
          ptr = ptrClass.upcast(ptr);
          ptrClass = ptrClass.baseClass;
      }
      return ptr;
    }function constNoSmartPtrRawPointerToWireType(destructors, handle) {
      if (handle === null) {
          if (this.isReference) {
              throwBindingError('null is not a valid ' + this.name);
          }
          return 0;
      }
  
      if (!handle.$$) {
          throwBindingError('Cannot pass "' + _embind_repr(handle) + '" as a ' + this.name);
      }
      if (!handle.$$.ptr) {
          throwBindingError('Cannot pass deleted object as a pointer of type ' + this.name);
      }
      var handleClass = handle.$$.ptrType.registeredClass;
      var ptr = upcastPointer(handle.$$.ptr, handleClass, this.registeredClass);
      return ptr;
    }
  
  function genericPointerToWireType(destructors, handle) {
      var ptr;
      if (handle === null) {
          if (this.isReference) {
              throwBindingError('null is not a valid ' + this.name);
          }
  
          if (this.isSmartPointer) {
              ptr = this.rawConstructor();
              if (destructors !== null) {
                  destructors.push(this.rawDestructor, ptr);
              }
              return ptr;
          } else {
              return 0;
          }
      }
  
      if (!handle.$$) {
          throwBindingError('Cannot pass "' + _embind_repr(handle) + '" as a ' + this.name);
      }
      if (!handle.$$.ptr) {
          throwBindingError('Cannot pass deleted object as a pointer of type ' + this.name);
      }
      if (!this.isConst && handle.$$.ptrType.isConst) {
          throwBindingError('Cannot convert argument of type ' + (handle.$$.smartPtrType ? handle.$$.smartPtrType.name : handle.$$.ptrType.name) + ' to parameter type ' + this.name);
      }
      var handleClass = handle.$$.ptrType.registeredClass;
      ptr = upcastPointer(handle.$$.ptr, handleClass, this.registeredClass);
  
      if (this.isSmartPointer) {
          // TODO: this is not strictly true
          // We could support BY_EMVAL conversions from raw pointers to smart pointers
          // because the smart pointer can hold a reference to the handle
          if (undefined === handle.$$.smartPtr) {
              throwBindingError('Passing raw pointer to smart pointer is illegal');
          }
  
          switch (this.sharingPolicy) {
              case 0: // NONE
                  // no upcasting
                  if (handle.$$.smartPtrType === this) {
                      ptr = handle.$$.smartPtr;
                  } else {
                      throwBindingError('Cannot convert argument of type ' + (handle.$$.smartPtrType ? handle.$$.smartPtrType.name : handle.$$.ptrType.name) + ' to parameter type ' + this.name);
                  }
                  break;
  
              case 1: // INTRUSIVE
                  ptr = handle.$$.smartPtr;
                  break;
  
              case 2: // BY_EMVAL
                  if (handle.$$.smartPtrType === this) {
                      ptr = handle.$$.smartPtr;
                  } else {
                      var clonedHandle = handle['clone']();
                      ptr = this.rawShare(
                          ptr,
                          __emval_register(function() {
                              clonedHandle['delete']();
                          })
                      );
                      if (destructors !== null) {
                          destructors.push(this.rawDestructor, ptr);
                      }
                  }
                  break;
  
              default:
                  throwBindingError('Unsupporting sharing policy');
          }
      }
      return ptr;
    }
  
  function nonConstNoSmartPtrRawPointerToWireType(destructors, handle) {
      if (handle === null) {
          if (this.isReference) {
              throwBindingError('null is not a valid ' + this.name);
          }
          return 0;
      }
  
      if (!handle.$$) {
          throwBindingError('Cannot pass "' + _embind_repr(handle) + '" as a ' + this.name);
      }
      if (!handle.$$.ptr) {
          throwBindingError('Cannot pass deleted object as a pointer of type ' + this.name);
      }
      if (handle.$$.ptrType.isConst) {
          throwBindingError('Cannot convert argument of type ' + handle.$$.ptrType.name + ' to parameter type ' + this.name);
      }
      var handleClass = handle.$$.ptrType.registeredClass;
      var ptr = upcastPointer(handle.$$.ptr, handleClass, this.registeredClass);
      return ptr;
    }
  
  
  function simpleReadValueFromPointer(pointer) {
      return this['fromWireType'](HEAPU32[pointer >> 2]);
    }
  
  function RegisteredPointer_getPointee(ptr) {
      if (this.rawGetPointee) {
          ptr = this.rawGetPointee(ptr);
      }
      return ptr;
    }
  
  function RegisteredPointer_destructor(ptr) {
      if (this.rawDestructor) {
          this.rawDestructor(ptr);
      }
    }
  
  function RegisteredPointer_deleteObject(handle) {
      if (handle !== null) {
          handle['delete']();
      }
    }
  
  
  function downcastPointer(ptr, ptrClass, desiredClass) {
      if (ptrClass === desiredClass) {
          return ptr;
      }
      if (undefined === desiredClass.baseClass) {
          return null; // no conversion
      }
  
      var rv = downcastPointer(ptr, ptrClass, desiredClass.baseClass);
      if (rv === null) {
          return null;
      }
      return desiredClass.downcast(rv);
    }
  
  
  
  
  function getInheritedInstanceCount() {
      return Object.keys(registeredInstances).length;
    }
  
  function getLiveInheritedInstances() {
      var rv = [];
      for (var k in registeredInstances) {
          if (registeredInstances.hasOwnProperty(k)) {
              rv.push(registeredInstances[k]);
          }
      }
      return rv;
    }
  
  function setDelayFunction(fn) {
      delayFunction = fn;
      if (deletionQueue.length && delayFunction) {
          delayFunction(flushPendingDeletes);
      }
    }function init_embind() {
      Module['getInheritedInstanceCount'] = getInheritedInstanceCount;
      Module['getLiveInheritedInstances'] = getLiveInheritedInstances;
      Module['flushPendingDeletes'] = flushPendingDeletes;
      Module['setDelayFunction'] = setDelayFunction;
    }var registeredInstances={};
  
  function getBasestPointer(class_, ptr) {
      if (ptr === undefined) {
          throwBindingError('ptr should not be undefined');
      }
      while (class_.baseClass) {
          ptr = class_.upcast(ptr);
          class_ = class_.baseClass;
      }
      return ptr;
    }function getInheritedInstance(class_, ptr) {
      ptr = getBasestPointer(class_, ptr);
      return registeredInstances[ptr];
    }
  
  function makeClassHandle(prototype, record) {
      if (!record.ptrType || !record.ptr) {
          throwInternalError('makeClassHandle requires ptr and ptrType');
      }
      var hasSmartPtrType = !!record.smartPtrType;
      var hasSmartPtr = !!record.smartPtr;
      if (hasSmartPtrType !== hasSmartPtr) {
          throwInternalError('Both smartPtrType and smartPtr must be specified');
      }
      record.count = { value: 1 };
      return attachFinalizer(Object.create(prototype, {
          $$: {
              value: record,
          },
      }));
    }function RegisteredPointer_fromWireType(ptr) {
      // ptr is a raw pointer (or a raw smartpointer)
  
      // rawPointer is a maybe-null raw pointer
      var rawPointer = this.getPointee(ptr);
      if (!rawPointer) {
          this.destructor(ptr);
          return null;
      }
  
      var registeredInstance = getInheritedInstance(this.registeredClass, rawPointer);
      if (undefined !== registeredInstance) {
          // JS object has been neutered, time to repopulate it
          if (0 === registeredInstance.$$.count.value) {
              registeredInstance.$$.ptr = rawPointer;
              registeredInstance.$$.smartPtr = ptr;
              return registeredInstance['clone']();
          } else {
              // else, just increment reference count on existing object
              // it already has a reference to the smart pointer
              var rv = registeredInstance['clone']();
              this.destructor(ptr);
              return rv;
          }
      }
  
      function makeDefaultHandle() {
          if (this.isSmartPointer) {
              return makeClassHandle(this.registeredClass.instancePrototype, {
                  ptrType: this.pointeeType,
                  ptr: rawPointer,
                  smartPtrType: this,
                  smartPtr: ptr,
              });
          } else {
              return makeClassHandle(this.registeredClass.instancePrototype, {
                  ptrType: this,
                  ptr: ptr,
              });
          }
      }
  
      var actualType = this.registeredClass.getActualType(rawPointer);
      var registeredPointerRecord = registeredPointers[actualType];
      if (!registeredPointerRecord) {
          return makeDefaultHandle.call(this);
      }
  
      var toType;
      if (this.isConst) {
          toType = registeredPointerRecord.constPointerType;
      } else {
          toType = registeredPointerRecord.pointerType;
      }
      var dp = downcastPointer(
          rawPointer,
          this.registeredClass,
          toType.registeredClass);
      if (dp === null) {
          return makeDefaultHandle.call(this);
      }
      if (this.isSmartPointer) {
          return makeClassHandle(toType.registeredClass.instancePrototype, {
              ptrType: toType,
              ptr: dp,
              smartPtrType: this,
              smartPtr: ptr,
          });
      } else {
          return makeClassHandle(toType.registeredClass.instancePrototype, {
              ptrType: toType,
              ptr: dp,
          });
      }
    }function init_RegisteredPointer() {
      RegisteredPointer.prototype.getPointee = RegisteredPointer_getPointee;
      RegisteredPointer.prototype.destructor = RegisteredPointer_destructor;
      RegisteredPointer.prototype['argPackAdvance'] = 8;
      RegisteredPointer.prototype['readValueFromPointer'] = simpleReadValueFromPointer;
      RegisteredPointer.prototype['deleteObject'] = RegisteredPointer_deleteObject;
      RegisteredPointer.prototype['fromWireType'] = RegisteredPointer_fromWireType;
    }/** @constructor
      @param {*=} pointeeType,
      @param {*=} sharingPolicy,
      @param {*=} rawGetPointee,
      @param {*=} rawConstructor,
      @param {*=} rawShare,
      @param {*=} rawDestructor,
       */
  function RegisteredPointer(
      name,
      registeredClass,
      isReference,
      isConst,
  
      // smart pointer properties
      isSmartPointer,
      pointeeType,
      sharingPolicy,
      rawGetPointee,
      rawConstructor,
      rawShare,
      rawDestructor
    ) {
      this.name = name;
      this.registeredClass = registeredClass;
      this.isReference = isReference;
      this.isConst = isConst;
  
      // smart pointer properties
      this.isSmartPointer = isSmartPointer;
      this.pointeeType = pointeeType;
      this.sharingPolicy = sharingPolicy;
      this.rawGetPointee = rawGetPointee;
      this.rawConstructor = rawConstructor;
      this.rawShare = rawShare;
      this.rawDestructor = rawDestructor;
  
      if (!isSmartPointer && registeredClass.baseClass === undefined) {
          if (isConst) {
              this['toWireType'] = constNoSmartPtrRawPointerToWireType;
              this.destructorFunction = null;
          } else {
              this['toWireType'] = nonConstNoSmartPtrRawPointerToWireType;
              this.destructorFunction = null;
          }
      } else {
          this['toWireType'] = genericPointerToWireType;
          // Here we must leave this.destructorFunction undefined, since whether genericPointerToWireType returns
          // a pointer that needs to be freed up is runtime-dependent, and cannot be evaluated at registration time.
          // TODO: Create an alternative mechanism that allows removing the use of var destructors = []; array in
          //       craftInvokerFunction altogether.
      }
    }
  
  /** @param {number=} numArguments */
  function replacePublicSymbol(name, value, numArguments) {
      if (!Module.hasOwnProperty(name)) {
          throwInternalError('Replacing nonexistant public symbol');
      }
      // If there's an overload table for this symbol, replace the symbol in the overload table instead.
      if (undefined !== Module[name].overloadTable && undefined !== numArguments) {
          Module[name].overloadTable[numArguments] = value;
      }
      else {
          Module[name] = value;
          Module[name].argCount = numArguments;
      }
    }
  
  
  function getDynCaller(sig, ptr) {
      assert(sig.indexOf('j') >= 0, 'getDynCaller should only be called with i64 sigs')
      var argCache = [];
      return function() {
        argCache.length = arguments.length;
        for (var i = 0; i < arguments.length; i++) {
          argCache[i] = arguments[i];
        }
        return dynCall(sig, ptr, argCache);
      };
    }function embind__requireFunction(signature, rawFunction) {
      signature = readLatin1String(signature);
  
      function makeDynCaller() {
        if (signature.indexOf('j') != -1) {
          return getDynCaller(signature, rawFunction);
        }
        return wasmTable.get(rawFunction);
      }
  
      var fp = makeDynCaller();
      if (typeof fp !== "function") {
          throwBindingError("unknown function pointer with signature " + signature + ": " + rawFunction);
      }
      return fp;
    }
  
  
  var UnboundTypeError=undefined;
  
  function getTypeName(type) {
      var ptr = ___getTypeName(type);
      var rv = readLatin1String(ptr);
      _free(ptr);
      return rv;
    }function throwUnboundTypeError(message, types) {
      var unboundTypes = [];
      var seen = {};
      function visit(type) {
          if (seen[type]) {
              return;
          }
          if (registeredTypes[type]) {
              return;
          }
          if (typeDependencies[type]) {
              typeDependencies[type].forEach(visit);
              return;
          }
          unboundTypes.push(type);
          seen[type] = true;
      }
      types.forEach(visit);
  
      throw new UnboundTypeError(message + ': ' + unboundTypes.map(getTypeName).join([', ']));
    }function __embind_register_class(
      rawType,
      rawPointerType,
      rawConstPointerType,
      baseClassRawType,
      getActualTypeSignature,
      getActualType,
      upcastSignature,
      upcast,
      downcastSignature,
      downcast,
      name,
      destructorSignature,
      rawDestructor
    ) {
      name = readLatin1String(name);
      getActualType = embind__requireFunction(getActualTypeSignature, getActualType);
      if (upcast) {
          upcast = embind__requireFunction(upcastSignature, upcast);
      }
      if (downcast) {
          downcast = embind__requireFunction(downcastSignature, downcast);
      }
      rawDestructor = embind__requireFunction(destructorSignature, rawDestructor);
      var legalFunctionName = makeLegalFunctionName(name);
  
      exposePublicSymbol(legalFunctionName, function() {
          // this code cannot run if baseClassRawType is zero
          throwUnboundTypeError('Cannot construct ' + name + ' due to unbound types', [baseClassRawType]);
      });
  
      whenDependentTypesAreResolved(
          [rawType, rawPointerType, rawConstPointerType],
          baseClassRawType ? [baseClassRawType] : [],
          function(base) {
              base = base[0];
  
              var baseClass;
              var basePrototype;
              if (baseClassRawType) {
                  baseClass = base.registeredClass;
                  basePrototype = baseClass.instancePrototype;
              } else {
                  basePrototype = ClassHandle.prototype;
              }
  
              var constructor = createNamedFunction(legalFunctionName, function() {
                  if (Object.getPrototypeOf(this) !== instancePrototype) {
                      throw new BindingError("Use 'new' to construct " + name);
                  }
                  if (undefined === registeredClass.constructor_body) {
                      throw new BindingError(name + " has no accessible constructor");
                  }
                  var body = registeredClass.constructor_body[arguments.length];
                  if (undefined === body) {
                      throw new BindingError("Tried to invoke ctor of " + name + " with invalid number of parameters (" + arguments.length + ") - expected (" + Object.keys(registeredClass.constructor_body).toString() + ") parameters instead!");
                  }
                  return body.apply(this, arguments);
              });
  
              var instancePrototype = Object.create(basePrototype, {
                  constructor: { value: constructor },
              });
  
              constructor.prototype = instancePrototype;
  
              var registeredClass = new RegisteredClass(
                  name,
                  constructor,
                  instancePrototype,
                  rawDestructor,
                  baseClass,
                  getActualType,
                  upcast,
                  downcast);
  
              var referenceConverter = new RegisteredPointer(
                  name,
                  registeredClass,
                  true,
                  false,
                  false);
  
              var pointerConverter = new RegisteredPointer(
                  name + '*',
                  registeredClass,
                  false,
                  false,
                  false);
  
              var constPointerConverter = new RegisteredPointer(
                  name + ' const*',
                  registeredClass,
                  false,
                  true,
                  false);
  
              registeredPointers[rawType] = {
                  pointerType: pointerConverter,
                  constPointerType: constPointerConverter
              };
  
              replacePublicSymbol(legalFunctionName, constructor);
  
              return [referenceConverter, pointerConverter, constPointerConverter];
          }
      );
    }

  
  function heap32VectorToArray(count, firstElement) {
      var array = [];
      for (var i = 0; i < count; i++) {
          array.push(HEAP32[(firstElement >> 2) + i]);
      }
      return array;
    }
  
  function runDestructors(destructors) {
      while (destructors.length) {
          var ptr = destructors.pop();
          var del = destructors.pop();
          del(ptr);
      }
    }function __embind_register_class_constructor(
      rawClassType,
      argCount,
      rawArgTypesAddr,
      invokerSignature,
      invoker,
      rawConstructor
    ) {
      assert(argCount > 0);
      var rawArgTypes = heap32VectorToArray(argCount, rawArgTypesAddr);
      invoker = embind__requireFunction(invokerSignature, invoker);
      var args = [rawConstructor];
      var destructors = [];
  
      whenDependentTypesAreResolved([], [rawClassType], function(classType) {
          classType = classType[0];
          var humanName = 'constructor ' + classType.name;
  
          if (undefined === classType.registeredClass.constructor_body) {
              classType.registeredClass.constructor_body = [];
          }
          if (undefined !== classType.registeredClass.constructor_body[argCount - 1]) {
              throw new BindingError("Cannot register multiple constructors with identical number of parameters (" + (argCount-1) + ") for class '" + classType.name + "'! Overload resolution is currently only performed using the parameter count, not actual type info!");
          }
          classType.registeredClass.constructor_body[argCount - 1] = function unboundTypeHandler() {
              throwUnboundTypeError('Cannot construct ' + classType.name + ' due to unbound types', rawArgTypes);
          };
  
          whenDependentTypesAreResolved([], rawArgTypes, function(argTypes) {
              classType.registeredClass.constructor_body[argCount - 1] = function constructor_body() {
                  if (arguments.length !== argCount - 1) {
                      throwBindingError(humanName + ' called with ' + arguments.length + ' arguments, expected ' + (argCount-1));
                  }
                  destructors.length = 0;
                  args.length = argCount;
                  for (var i = 1; i < argCount; ++i) {
                      args[i] = argTypes[i]['toWireType'](destructors, arguments[i - 1]);
                  }
  
                  var ptr = invoker.apply(null, args);
                  runDestructors(destructors);
  
                  return argTypes[0]['fromWireType'](ptr);
              };
              return [];
          });
          return [];
      });
    }

  
  
  function new_(constructor, argumentList) {
      if (!(constructor instanceof Function)) {
          throw new TypeError('new_ called with constructor type ' + typeof(constructor) + " which is not a function");
      }
  
      /*
       * Previously, the following line was just:
  
       function dummy() {};
  
       * Unfortunately, Chrome was preserving 'dummy' as the object's name, even though at creation, the 'dummy' has the
       * correct constructor name.  Thus, objects created with IMVU.new would show up in the debugger as 'dummy', which
       * isn't very helpful.  Using IMVU.createNamedFunction addresses the issue.  Doublely-unfortunately, there's no way
       * to write a test for this behavior.  -NRD 2013.02.22
       */
      var dummy = createNamedFunction(constructor.name || 'unknownFunctionName', function(){});
      dummy.prototype = constructor.prototype;
      var obj = new dummy;
  
      var r = constructor.apply(obj, argumentList);
      return (r instanceof Object) ? r : obj;
    }function craftInvokerFunction(humanName, argTypes, classType, cppInvokerFunc, cppTargetFunc) {
      // humanName: a human-readable string name for the function to be generated.
      // argTypes: An array that contains the embind type objects for all types in the function signature.
      //    argTypes[0] is the type object for the function return value.
      //    argTypes[1] is the type object for function this object/class type, or null if not crafting an invoker for a class method.
      //    argTypes[2...] are the actual function parameters.
      // classType: The embind type object for the class to be bound, or null if this is not a method of a class.
      // cppInvokerFunc: JS Function object to the C++-side function that interops into C++ code.
      // cppTargetFunc: Function pointer (an integer to FUNCTION_TABLE) to the target C++ function the cppInvokerFunc will end up calling.
      var argCount = argTypes.length;
  
      if (argCount < 2) {
          throwBindingError("argTypes array size mismatch! Must at least get return value and 'this' types!");
      }
  
      var isClassMethodFunc = (argTypes[1] !== null && classType !== null);
  
      // Free functions with signature "void function()" do not need an invoker that marshalls between wire types.
  // TODO: This omits argument count check - enable only at -O3 or similar.
  //    if (ENABLE_UNSAFE_OPTS && argCount == 2 && argTypes[0].name == "void" && !isClassMethodFunc) {
  //       return FUNCTION_TABLE[fn];
  //    }
  
  
      // Determine if we need to use a dynamic stack to store the destructors for the function parameters.
      // TODO: Remove this completely once all function invokers are being dynamically generated.
      var needsDestructorStack = false;
  
      for(var i = 1; i < argTypes.length; ++i) { // Skip return value at index 0 - it's not deleted here.
          if (argTypes[i] !== null && argTypes[i].destructorFunction === undefined) { // The type does not define a destructor function - must use dynamic stack
              needsDestructorStack = true;
              break;
          }
      }
  
      var returns = (argTypes[0].name !== "void");
  
      var argsList = "";
      var argsListWired = "";
      for(var i = 0; i < argCount - 2; ++i) {
          argsList += (i!==0?", ":"")+"arg"+i;
          argsListWired += (i!==0?", ":"")+"arg"+i+"Wired";
      }
  
      var invokerFnBody =
          "return function "+makeLegalFunctionName(humanName)+"("+argsList+") {\n" +
          "if (arguments.length !== "+(argCount - 2)+") {\n" +
              "throwBindingError('function "+humanName+" called with ' + arguments.length + ' arguments, expected "+(argCount - 2)+" args!');\n" +
          "}\n";
  
  
      if (needsDestructorStack) {
          invokerFnBody +=
              "var destructors = [];\n";
      }
  
      var dtorStack = needsDestructorStack ? "destructors" : "null";
      var args1 = ["throwBindingError", "invoker", "fn", "runDestructors", "retType", "classParam"];
      var args2 = [throwBindingError, cppInvokerFunc, cppTargetFunc, runDestructors, argTypes[0], argTypes[1]];
  
  
      if (isClassMethodFunc) {
          invokerFnBody += "var thisWired = classParam.toWireType("+dtorStack+", this);\n";
      }
  
      for(var i = 0; i < argCount - 2; ++i) {
          invokerFnBody += "var arg"+i+"Wired = argType"+i+".toWireType("+dtorStack+", arg"+i+"); // "+argTypes[i+2].name+"\n";
          args1.push("argType"+i);
          args2.push(argTypes[i+2]);
      }
  
      if (isClassMethodFunc) {
          argsListWired = "thisWired" + (argsListWired.length > 0 ? ", " : "") + argsListWired;
      }
  
      invokerFnBody +=
          (returns?"var rv = ":"") + "invoker(fn"+(argsListWired.length>0?", ":"")+argsListWired+");\n";
  
      if (needsDestructorStack) {
          invokerFnBody += "runDestructors(destructors);\n";
      } else {
          for(var i = isClassMethodFunc?1:2; i < argTypes.length; ++i) { // Skip return value at index 0 - it's not deleted here. Also skip class type if not a method.
              var paramName = (i === 1 ? "thisWired" : ("arg"+(i - 2)+"Wired"));
              if (argTypes[i].destructorFunction !== null) {
                  invokerFnBody += paramName+"_dtor("+paramName+"); // "+argTypes[i].name+"\n";
                  args1.push(paramName+"_dtor");
                  args2.push(argTypes[i].destructorFunction);
              }
          }
      }
  
      if (returns) {
          invokerFnBody += "var ret = retType.fromWireType(rv);\n" +
                           "return ret;\n";
      } else {
      }
      invokerFnBody += "}\n";
  
      args1.push(invokerFnBody);
  
      var invokerFunction = new_(Function, args1).apply(null, args2);
      return invokerFunction;
    }function __embind_register_class_function(
      rawClassType,
      methodName,
      argCount,
      rawArgTypesAddr, // [ReturnType, ThisType, Args...]
      invokerSignature,
      rawInvoker,
      context,
      isPureVirtual
    ) {
      var rawArgTypes = heap32VectorToArray(argCount, rawArgTypesAddr);
      methodName = readLatin1String(methodName);
      rawInvoker = embind__requireFunction(invokerSignature, rawInvoker);
  
      whenDependentTypesAreResolved([], [rawClassType], function(classType) {
          classType = classType[0];
          var humanName = classType.name + '.' + methodName;
  
          if (isPureVirtual) {
              classType.registeredClass.pureVirtualFunctions.push(methodName);
          }
  
          function unboundTypesHandler() {
              throwUnboundTypeError('Cannot call ' + humanName + ' due to unbound types', rawArgTypes);
          }
  
          var proto = classType.registeredClass.instancePrototype;
          var method = proto[methodName];
          if (undefined === method || (undefined === method.overloadTable && method.className !== classType.name && method.argCount === argCount - 2)) {
              // This is the first overload to be registered, OR we are replacing a function in the base class with a function in the derived class.
              unboundTypesHandler.argCount = argCount - 2;
              unboundTypesHandler.className = classType.name;
              proto[methodName] = unboundTypesHandler;
          } else {
              // There was an existing function with the same name registered. Set up a function overload routing table.
              ensureOverloadTable(proto, methodName, humanName);
              proto[methodName].overloadTable[argCount - 2] = unboundTypesHandler;
          }
  
          whenDependentTypesAreResolved([], rawArgTypes, function(argTypes) {
  
              var memberFunction = craftInvokerFunction(humanName, argTypes, classType, rawInvoker, context);
  
              // Replace the initial unbound-handler-stub function with the appropriate member function, now that all types
              // are resolved. If multiple overloads are registered for this function, the function goes into an overload table.
              if (undefined === proto[methodName].overloadTable) {
                  // Set argCount in case an overload is registered later
                  memberFunction.argCount = argCount - 2;
                  proto[methodName] = memberFunction;
              } else {
                  proto[methodName].overloadTable[argCount - 2] = memberFunction;
              }
  
              return [];
          });
          return [];
      });
    }

  
  function validateThis(this_, classType, humanName) {
      if (!(this_ instanceof Object)) {
          throwBindingError(humanName + ' with invalid "this": ' + this_);
      }
      if (!(this_ instanceof classType.registeredClass.constructor)) {
          throwBindingError(humanName + ' incompatible with "this" of type ' + this_.constructor.name);
      }
      if (!this_.$$.ptr) {
          throwBindingError('cannot call emscripten binding method ' + humanName + ' on deleted object');
      }
  
      // todo: kill this
      return upcastPointer(
          this_.$$.ptr,
          this_.$$.ptrType.registeredClass,
          classType.registeredClass);
    }function __embind_register_class_property(
      classType,
      fieldName,
      getterReturnType,
      getterSignature,
      getter,
      getterContext,
      setterArgumentType,
      setterSignature,
      setter,
      setterContext
    ) {
      fieldName = readLatin1String(fieldName);
      getter = embind__requireFunction(getterSignature, getter);
  
      whenDependentTypesAreResolved([], [classType], function(classType) {
          classType = classType[0];
          var humanName = classType.name + '.' + fieldName;
          var desc = {
              get: function() {
                  throwUnboundTypeError('Cannot access ' + humanName + ' due to unbound types', [getterReturnType, setterArgumentType]);
              },
              enumerable: true,
              configurable: true
          };
          if (setter) {
              desc.set = function() {
                  throwUnboundTypeError('Cannot access ' + humanName + ' due to unbound types', [getterReturnType, setterArgumentType]);
              };
          } else {
              desc.set = function(v) {
                  throwBindingError(humanName + ' is a read-only property');
              };
          }
  
          Object.defineProperty(classType.registeredClass.instancePrototype, fieldName, desc);
  
          whenDependentTypesAreResolved(
              [],
              (setter ? [getterReturnType, setterArgumentType] : [getterReturnType]),
          function(types) {
              var getterReturnType = types[0];
              var desc = {
                  get: function() {
                      var ptr = validateThis(this, classType, humanName + ' getter');
                      return getterReturnType['fromWireType'](getter(getterContext, ptr));
                  },
                  enumerable: true
              };
  
              if (setter) {
                  setter = embind__requireFunction(setterSignature, setter);
                  var setterArgumentType = types[1];
                  desc.set = function(v) {
                      var ptr = validateThis(this, classType, humanName + ' setter');
                      var destructors = [];
                      setter(setterContext, ptr, setterArgumentType['toWireType'](destructors, v));
                      runDestructors(destructors);
                  };
              }
  
              Object.defineProperty(classType.registeredClass.instancePrototype, fieldName, desc);
              return [];
          });
  
          return [];
      });
    }

  
  
  var emval_free_list=[];
  
  var emval_handle_array=[{},{value:undefined},{value:null},{value:true},{value:false}];function __emval_decref(handle) {
      if (handle > 4 && 0 === --emval_handle_array[handle].refcount) {
          emval_handle_array[handle] = undefined;
          emval_free_list.push(handle);
      }
    }
  
  
  
  function count_emval_handles() {
      var count = 0;
      for (var i = 5; i < emval_handle_array.length; ++i) {
          if (emval_handle_array[i] !== undefined) {
              ++count;
          }
      }
      return count;
    }
  
  function get_first_emval() {
      for (var i = 5; i < emval_handle_array.length; ++i) {
          if (emval_handle_array[i] !== undefined) {
              return emval_handle_array[i];
          }
      }
      return null;
    }function init_emval() {
      Module['count_emval_handles'] = count_emval_handles;
      Module['get_first_emval'] = get_first_emval;
    }function __emval_register(value) {
  
      switch(value){
        case undefined :{ return 1; }
        case null :{ return 2; }
        case true :{ return 3; }
        case false :{ return 4; }
        default:{
          var handle = emval_free_list.length ?
              emval_free_list.pop() :
              emval_handle_array.length;
  
          emval_handle_array[handle] = {refcount: 1, value: value};
          return handle;
          }
        }
    }function __embind_register_emval(rawType, name) {
      name = readLatin1String(name);
      registerType(rawType, {
          name: name,
          'fromWireType': function(handle) {
              var rv = emval_handle_array[handle].value;
              __emval_decref(handle);
              return rv;
          },
          'toWireType': function(destructors, value) {
              return __emval_register(value);
          },
          'argPackAdvance': 8,
          'readValueFromPointer': simpleReadValueFromPointer,
          destructorFunction: null, // This type does not need a destructor
  
          // TODO: do we need a deleteObject here?  write a test where
          // emval is passed into JS via an interface
      });
    }

  
  function enumReadValueFromPointer(name, shift, signed) {
      switch (shift) {
          case 0: return function(pointer) {
              var heap = signed ? HEAP8 : HEAPU8;
              return this['fromWireType'](heap[pointer]);
          };
          case 1: return function(pointer) {
              var heap = signed ? HEAP16 : HEAPU16;
              return this['fromWireType'](heap[pointer >> 1]);
          };
          case 2: return function(pointer) {
              var heap = signed ? HEAP32 : HEAPU32;
              return this['fromWireType'](heap[pointer >> 2]);
          };
          default:
              throw new TypeError("Unknown integer type: " + name);
      }
    }function __embind_register_enum(
      rawType,
      name,
      size,
      isSigned
    ) {
      var shift = getShiftFromSize(size);
      name = readLatin1String(name);
  
      function ctor() {
      }
      ctor.values = {};
  
      registerType(rawType, {
          name: name,
          constructor: ctor,
          'fromWireType': function(c) {
              return this.constructor.values[c];
          },
          'toWireType': function(destructors, c) {
              return c.value;
          },
          'argPackAdvance': 8,
          'readValueFromPointer': enumReadValueFromPointer(name, shift, isSigned),
          destructorFunction: null,
      });
      exposePublicSymbol(name, ctor);
    }

  
  function requireRegisteredType(rawType, humanName) {
      var impl = registeredTypes[rawType];
      if (undefined === impl) {
          throwBindingError(humanName + " has unknown type " + getTypeName(rawType));
      }
      return impl;
    }function __embind_register_enum_value(
      rawEnumType,
      name,
      enumValue
    ) {
      var enumType = requireRegisteredType(rawEnumType, 'enum');
      name = readLatin1String(name);
  
      var Enum = enumType.constructor;
  
      var Value = Object.create(enumType.constructor.prototype, {
          value: {value: enumValue},
          constructor: {value: createNamedFunction(enumType.name + '_' + name, function() {})},
      });
      Enum.values[enumValue] = Value;
      Enum[name] = Value;
    }

  
  function _embind_repr(v) {
      if (v === null) {
          return 'null';
      }
      var t = typeof v;
      if (t === 'object' || t === 'array' || t === 'function') {
          return v.toString();
      } else {
          return '' + v;
      }
    }
  
  function floatReadValueFromPointer(name, shift) {
      switch (shift) {
          case 2: return function(pointer) {
              return this['fromWireType'](HEAPF32[pointer >> 2]);
          };
          case 3: return function(pointer) {
              return this['fromWireType'](HEAPF64[pointer >> 3]);
          };
          default:
              throw new TypeError("Unknown float type: " + name);
      }
    }function __embind_register_float(rawType, name, size) {
      var shift = getShiftFromSize(size);
      name = readLatin1String(name);
      registerType(rawType, {
          name: name,
          'fromWireType': function(value) {
              return value;
          },
          'toWireType': function(destructors, value) {
              // todo: Here we have an opportunity for -O3 level "unsafe" optimizations: we could
              // avoid the following if() and assume value is of proper type.
              if (typeof value !== "number" && typeof value !== "boolean") {
                  throw new TypeError('Cannot convert "' + _embind_repr(value) + '" to ' + this.name);
              }
              return value;
          },
          'argPackAdvance': 8,
          'readValueFromPointer': floatReadValueFromPointer(name, shift),
          destructorFunction: null, // This type does not need a destructor
      });
    }

  function __embind_register_function(name, argCount, rawArgTypesAddr, signature, rawInvoker, fn) {
      var argTypes = heap32VectorToArray(argCount, rawArgTypesAddr);
      name = readLatin1String(name);
  
      rawInvoker = embind__requireFunction(signature, rawInvoker);
  
      exposePublicSymbol(name, function() {
          throwUnboundTypeError('Cannot call ' + name + ' due to unbound types', argTypes);
      }, argCount - 1);
  
      whenDependentTypesAreResolved([], argTypes, function(argTypes) {
          var invokerArgsArray = [argTypes[0] /* return value */, null /* no class 'this'*/].concat(argTypes.slice(1) /* actual params */);
          replacePublicSymbol(name, craftInvokerFunction(name, invokerArgsArray, null /* no class 'this'*/, rawInvoker, fn), argCount - 1);
          return [];
      });
    }

  
  function integerReadValueFromPointer(name, shift, signed) {
      // integers are quite common, so generate very specialized functions
      switch (shift) {
          case 0: return signed ?
              function readS8FromPointer(pointer) { return HEAP8[pointer]; } :
              function readU8FromPointer(pointer) { return HEAPU8[pointer]; };
          case 1: return signed ?
              function readS16FromPointer(pointer) { return HEAP16[pointer >> 1]; } :
              function readU16FromPointer(pointer) { return HEAPU16[pointer >> 1]; };
          case 2: return signed ?
              function readS32FromPointer(pointer) { return HEAP32[pointer >> 2]; } :
              function readU32FromPointer(pointer) { return HEAPU32[pointer >> 2]; };
          default:
              throw new TypeError("Unknown integer type: " + name);
      }
    }function __embind_register_integer(primitiveType, name, size, minRange, maxRange) {
      name = readLatin1String(name);
      if (maxRange === -1) { // LLVM doesn't have signed and unsigned 32-bit types, so u32 literals come out as 'i32 -1'. Always treat those as max u32.
          maxRange = 4294967295;
      }
  
      var shift = getShiftFromSize(size);
  
      var fromWireType = function(value) {
          return value;
      };
  
      if (minRange === 0) {
          var bitshift = 32 - 8*size;
          fromWireType = function(value) {
              return (value << bitshift) >>> bitshift;
          };
      }
  
      var isUnsignedType = (name.indexOf('unsigned') != -1);
  
      registerType(primitiveType, {
          name: name,
          'fromWireType': fromWireType,
          'toWireType': function(destructors, value) {
              // todo: Here we have an opportunity for -O3 level "unsafe" optimizations: we could
              // avoid the following two if()s and assume value is of proper type.
              if (typeof value !== "number" && typeof value !== "boolean") {
                  throw new TypeError('Cannot convert "' + _embind_repr(value) + '" to ' + this.name);
              }
              if (value < minRange || value > maxRange) {
                  throw new TypeError('Passing a number "' + _embind_repr(value) + '" from JS side to C/C++ side to an argument of type "' + name + '", which is outside the valid range [' + minRange + ', ' + maxRange + ']!');
              }
              return isUnsignedType ? (value >>> 0) : (value | 0);
          },
          'argPackAdvance': 8,
          'readValueFromPointer': integerReadValueFromPointer(name, shift, minRange !== 0),
          destructorFunction: null, // This type does not need a destructor
      });
    }

  function __embind_register_memory_view(rawType, dataTypeIndex, name) {
      var typeMapping = [
          Int8Array,
          Uint8Array,
          Int16Array,
          Uint16Array,
          Int32Array,
          Uint32Array,
          Float32Array,
          Float64Array,
      ];
  
      var TA = typeMapping[dataTypeIndex];
  
      function decodeMemoryView(handle) {
          handle = handle >> 2;
          var heap = HEAPU32;
          var size = heap[handle]; // in elements
          var data = heap[handle + 1]; // byte offset into emscripten heap
          return new TA(buffer, data, size);
      }
  
      name = readLatin1String(name);
      registerType(rawType, {
          name: name,
          'fromWireType': decodeMemoryView,
          'argPackAdvance': 8,
          'readValueFromPointer': decodeMemoryView,
      }, {
          ignoreDuplicateRegistrations: true,
      });
    }

  function __embind_register_std_string(rawType, name) {
      name = readLatin1String(name);
      var stdStringIsUTF8
      //process only std::string bindings with UTF8 support, in contrast to e.g. std::basic_string<unsigned char>
      = (name === "std::string");
  
      registerType(rawType, {
          name: name,
          'fromWireType': function(value) {
              var length = HEAPU32[value >> 2];
  
              var str;
              if (stdStringIsUTF8) {
                  var decodeStartPtr = value + 4;
                  // Looping here to support possible embedded '0' bytes
                  for (var i = 0; i <= length; ++i) {
                      var currentBytePtr = value + 4 + i;
                      if (i == length || HEAPU8[currentBytePtr] == 0) {
                          var maxRead = currentBytePtr - decodeStartPtr;
                          var stringSegment = UTF8ToString(decodeStartPtr, maxRead);
                          if (str === undefined) {
                              str = stringSegment;
                          } else {
                              str += String.fromCharCode(0);
                              str += stringSegment;
                          }
                          decodeStartPtr = currentBytePtr + 1;
                      }
                  }
              } else {
                  var a = new Array(length);
                  for (var i = 0; i < length; ++i) {
                      a[i] = String.fromCharCode(HEAPU8[value + 4 + i]);
                  }
                  str = a.join('');
              }
  
              _free(value);
  
              return str;
          },
          'toWireType': function(destructors, value) {
              if (value instanceof ArrayBuffer) {
                  value = new Uint8Array(value);
              }
  
              var getLength;
              var valueIsOfTypeString = (typeof value === 'string');
  
              if (!(valueIsOfTypeString || value instanceof Uint8Array || value instanceof Uint8ClampedArray || value instanceof Int8Array)) {
                  throwBindingError('Cannot pass non-string to std::string');
              }
              if (stdStringIsUTF8 && valueIsOfTypeString) {
                  getLength = function() {return lengthBytesUTF8(value);};
              } else {
                  getLength = function() {return value.length;};
              }
  
              // assumes 4-byte alignment
              var length = getLength();
              var ptr = _malloc(4 + length + 1);
              HEAPU32[ptr >> 2] = length;
              if (stdStringIsUTF8 && valueIsOfTypeString) {
                  stringToUTF8(value, ptr + 4, length + 1);
              } else {
                  if (valueIsOfTypeString) {
                      for (var i = 0; i < length; ++i) {
                          var charCode = value.charCodeAt(i);
                          if (charCode > 255) {
                              _free(ptr);
                              throwBindingError('String has UTF-16 code units that do not fit in 8 bits');
                          }
                          HEAPU8[ptr + 4 + i] = charCode;
                      }
                  } else {
                      for (var i = 0; i < length; ++i) {
                          HEAPU8[ptr + 4 + i] = value[i];
                      }
                  }
              }
  
              if (destructors !== null) {
                  destructors.push(_free, ptr);
              }
              return ptr;
          },
          'argPackAdvance': 8,
          'readValueFromPointer': simpleReadValueFromPointer,
          destructorFunction: function(ptr) { _free(ptr); },
      });
    }

  function __embind_register_std_wstring(rawType, charSize, name) {
      name = readLatin1String(name);
      var decodeString, encodeString, getHeap, lengthBytesUTF, shift;
      if (charSize === 2) {
          decodeString = UTF16ToString;
          encodeString = stringToUTF16;
          lengthBytesUTF = lengthBytesUTF16;
          getHeap = function() { return HEAPU16; };
          shift = 1;
      } else if (charSize === 4) {
          decodeString = UTF32ToString;
          encodeString = stringToUTF32;
          lengthBytesUTF = lengthBytesUTF32;
          getHeap = function() { return HEAPU32; };
          shift = 2;
      }
      registerType(rawType, {
          name: name,
          'fromWireType': function(value) {
              // Code mostly taken from _embind_register_std_string fromWireType
              var length = HEAPU32[value >> 2];
              var HEAP = getHeap();
              var str;
  
              var decodeStartPtr = value + 4;
              // Looping here to support possible embedded '0' bytes
              for (var i = 0; i <= length; ++i) {
                  var currentBytePtr = value + 4 + i * charSize;
                  if (i == length || HEAP[currentBytePtr >> shift] == 0) {
                      var maxReadBytes = currentBytePtr - decodeStartPtr;
                      var stringSegment = decodeString(decodeStartPtr, maxReadBytes);
                      if (str === undefined) {
                          str = stringSegment;
                      } else {
                          str += String.fromCharCode(0);
                          str += stringSegment;
                      }
                      decodeStartPtr = currentBytePtr + charSize;
                  }
              }
  
              _free(value);
  
              return str;
          },
          'toWireType': function(destructors, value) {
              if (!(typeof value === 'string')) {
                  throwBindingError('Cannot pass non-string to C++ string type ' + name);
              }
  
              // assumes 4-byte alignment
              var length = lengthBytesUTF(value);
              var ptr = _malloc(4 + length + charSize);
              HEAPU32[ptr >> 2] = length >> shift;
  
              encodeString(value, ptr + 4, length + charSize);
  
              if (destructors !== null) {
                  destructors.push(_free, ptr);
              }
              return ptr;
          },
          'argPackAdvance': 8,
          'readValueFromPointer': simpleReadValueFromPointer,
          destructorFunction: function(ptr) { _free(ptr); },
      });
    }

  function __embind_register_void(rawType, name) {
      name = readLatin1String(name);
      registerType(rawType, {
          isVoid: true, // void return values can be optimized out sometimes
          name: name,
          'argPackAdvance': 0,
          'fromWireType': function() {
              return undefined;
          },
          'toWireType': function(destructors, o) {
              // TODO: assert if anything else is given?
              return undefined;
          },
      });
    }

  function _abort() {
      abort();
    }

  function _emscripten_memcpy_big(dest, src, num) {
      HEAPU8.copyWithin(dest, src, src + num);
    }

  
  function _emscripten_get_heap_size() {
      return HEAPU8.length;
    }
  
  function abortOnCannotGrowMemory(requestedSize) {
      abort('Cannot enlarge memory arrays to size ' + requestedSize + ' bytes (OOM). Either (1) compile with  -s INITIAL_MEMORY=X  with X higher than the current value ' + HEAP8.length + ', (2) compile with  -s ALLOW_MEMORY_GROWTH=1  which allows increasing the size at runtime, or (3) if you want malloc to return NULL (0) instead of this abort, compile with  -s ABORTING_MALLOC=0 ');
    }function _emscripten_resize_heap(requestedSize) {
      requestedSize = requestedSize >>> 0;
      abortOnCannotGrowMemory(requestedSize);
    }
embind_init_charCodes();
BindingError = Module['BindingError'] = extendError(Error, 'BindingError');;
InternalError = Module['InternalError'] = extendError(Error, 'InternalError');;
init_ClassHandle();
init_RegisteredPointer();
init_embind();;
UnboundTypeError = Module['UnboundTypeError'] = extendError(Error, 'UnboundTypeError');;
init_emval();;
var ASSERTIONS = true;



/** @type {function(string, boolean=, number=)} */
function intArrayFromString(stringy, dontAddNull, length) {
  var len = length > 0 ? length : lengthBytesUTF8(stringy)+1;
  var u8array = new Array(len);
  var numBytesWritten = stringToUTF8Array(stringy, u8array, 0, u8array.length);
  if (dontAddNull) u8array.length = numBytesWritten;
  return u8array;
}

function intArrayToString(array) {
  var ret = [];
  for (var i = 0; i < array.length; i++) {
    var chr = array[i];
    if (chr > 0xFF) {
      if (ASSERTIONS) {
        assert(false, 'Character code ' + chr + ' (' + String.fromCharCode(chr) + ')  at offset ' + i + ' not in 0x00-0xFF.');
      }
      chr &= 0xFF;
    }
    ret.push(String.fromCharCode(chr));
  }
  return ret.join('');
}


// Copied from https://github.com/strophe/strophejs/blob/e06d027/src/polyfills.js#L149

// This code was written by Tyler Akins and has been placed in the
// public domain.  It would be nice if you left this header intact.
// Base64 code from Tyler Akins -- http://rumkin.com

/**
 * Decodes a base64 string.
 * @param {string} input The string to decode.
 */
var decodeBase64 = typeof atob === 'function' ? atob : function (input) {
  var keyStr = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/=';

  var output = '';
  var chr1, chr2, chr3;
  var enc1, enc2, enc3, enc4;
  var i = 0;
  // remove all characters that are not A-Z, a-z, 0-9, +, /, or =
  input = input.replace(/[^A-Za-z0-9\+\/\=]/g, '');
  do {
    enc1 = keyStr.indexOf(input.charAt(i++));
    enc2 = keyStr.indexOf(input.charAt(i++));
    enc3 = keyStr.indexOf(input.charAt(i++));
    enc4 = keyStr.indexOf(input.charAt(i++));

    chr1 = (enc1 << 2) | (enc2 >> 4);
    chr2 = ((enc2 & 15) << 4) | (enc3 >> 2);
    chr3 = ((enc3 & 3) << 6) | enc4;

    output = output + String.fromCharCode(chr1);

    if (enc3 !== 64) {
      output = output + String.fromCharCode(chr2);
    }
    if (enc4 !== 64) {
      output = output + String.fromCharCode(chr3);
    }
  } while (i < input.length);
  return output;
};

// Converts a string of base64 into a byte array.
// Throws error on invalid input.
function intArrayFromBase64(s) {
  if (typeof ENVIRONMENT_IS_NODE === 'boolean' && ENVIRONMENT_IS_NODE) {
    var buf;
    try {
      // TODO: Update Node.js externs, Closure does not recognize the following Buffer.from()
      /**@suppress{checkTypes}*/
      buf = Buffer.from(s, 'base64');
    } catch (_) {
      buf = new Buffer(s, 'base64');
    }
    return new Uint8Array(buf['buffer'], buf['byteOffset'], buf['byteLength']);
  }

  try {
    var decoded = decodeBase64(s);
    var bytes = new Uint8Array(decoded.length);
    for (var i = 0 ; i < decoded.length ; ++i) {
      bytes[i] = decoded.charCodeAt(i);
    }
    return bytes;
  } catch (_) {
    throw new Error('Converting base64 string to bytes failed.');
  }
}

// If filename is a base64 data URI, parses and returns data (Buffer on node,
// Uint8Array otherwise). If filename is not a base64 data URI, returns undefined.
function tryParseAsDataURI(filename) {
  if (!isDataURI(filename)) {
    return;
  }

  return intArrayFromBase64(filename.slice(dataURIPrefix.length));
}


var asmLibraryArg = { "__assert_fail": ___assert_fail, "__indirect_function_table": wasmTable, "_embind_register_bool": __embind_register_bool, "_embind_register_class": __embind_register_class, "_embind_register_class_constructor": __embind_register_class_constructor, "_embind_register_class_function": __embind_register_class_function, "_embind_register_class_property": __embind_register_class_property, "_embind_register_emval": __embind_register_emval, "_embind_register_enum": __embind_register_enum, "_embind_register_enum_value": __embind_register_enum_value, "_embind_register_float": __embind_register_float, "_embind_register_function": __embind_register_function, "_embind_register_integer": __embind_register_integer, "_embind_register_memory_view": __embind_register_memory_view, "_embind_register_std_string": __embind_register_std_string, "_embind_register_std_wstring": __embind_register_std_wstring, "_embind_register_void": __embind_register_void, "abort": _abort, "emscripten_memcpy_big": _emscripten_memcpy_big, "emscripten_resize_heap": _emscripten_resize_heap, "getTempRet0": getTempRet0, "memory": wasmMemory, "setTempRet0": setTempRet0 };
var asm = createWasm();
/** @type {function(...*):?} */
var ___wasm_call_ctors = Module["___wasm_call_ctors"] = createExportWrapper("__wasm_call_ctors");

/** @type {function(...*):?} */
var ___getTypeName = Module["___getTypeName"] = createExportWrapper("__getTypeName");

/** @type {function(...*):?} */
var ___embind_register_native_and_builtin_types = Module["___embind_register_native_and_builtin_types"] = createExportWrapper("__embind_register_native_and_builtin_types");

/** @type {function(...*):?} */
var ___errno_location = Module["___errno_location"] = createExportWrapper("__errno_location");

/** @type {function(...*):?} */
var _malloc = Module["_malloc"] = createExportWrapper("malloc");

/** @type {function(...*):?} */
var _fflush = Module["_fflush"] = createExportWrapper("fflush");

/** @type {function(...*):?} */
var stackSave = Module["stackSave"] = createExportWrapper("stackSave");

/** @type {function(...*):?} */
var stackRestore = Module["stackRestore"] = createExportWrapper("stackRestore");

/** @type {function(...*):?} */
var stackAlloc = Module["stackAlloc"] = createExportWrapper("stackAlloc");

/** @type {function(...*):?} */
var _free = Module["_free"] = createExportWrapper("free");

/** @type {function(...*):?} */
var __growWasmMemory = Module["__growWasmMemory"] = createExportWrapper("__growWasmMemory");





// === Auto-generated postamble setup entry stuff ===

if (!Object.getOwnPropertyDescriptor(Module, "intArrayFromString")) Module["intArrayFromString"] = function() { abort("'intArrayFromString' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "intArrayToString")) Module["intArrayToString"] = function() { abort("'intArrayToString' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "ccall")) Module["ccall"] = function() { abort("'ccall' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "cwrap")) Module["cwrap"] = function() { abort("'cwrap' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "setValue")) Module["setValue"] = function() { abort("'setValue' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "getValue")) Module["getValue"] = function() { abort("'getValue' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "allocate")) Module["allocate"] = function() { abort("'allocate' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "getMemory")) Module["getMemory"] = function() { abort("'getMemory' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ). Alternatively, forcing filesystem support (-s FORCE_FILESYSTEM=1) can export this for you") };
if (!Object.getOwnPropertyDescriptor(Module, "UTF8ArrayToString")) Module["UTF8ArrayToString"] = function() { abort("'UTF8ArrayToString' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "UTF8ToString")) Module["UTF8ToString"] = function() { abort("'UTF8ToString' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "stringToUTF8Array")) Module["stringToUTF8Array"] = function() { abort("'stringToUTF8Array' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "stringToUTF8")) Module["stringToUTF8"] = function() { abort("'stringToUTF8' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "lengthBytesUTF8")) Module["lengthBytesUTF8"] = function() { abort("'lengthBytesUTF8' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "stackTrace")) Module["stackTrace"] = function() { abort("'stackTrace' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "addOnPreRun")) Module["addOnPreRun"] = function() { abort("'addOnPreRun' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "addOnInit")) Module["addOnInit"] = function() { abort("'addOnInit' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "addOnPreMain")) Module["addOnPreMain"] = function() { abort("'addOnPreMain' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "addOnExit")) Module["addOnExit"] = function() { abort("'addOnExit' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "addOnPostRun")) Module["addOnPostRun"] = function() { abort("'addOnPostRun' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "writeStringToMemory")) Module["writeStringToMemory"] = function() { abort("'writeStringToMemory' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "writeArrayToMemory")) Module["writeArrayToMemory"] = function() { abort("'writeArrayToMemory' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "writeAsciiToMemory")) Module["writeAsciiToMemory"] = function() { abort("'writeAsciiToMemory' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "addRunDependency")) Module["addRunDependency"] = function() { abort("'addRunDependency' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ). Alternatively, forcing filesystem support (-s FORCE_FILESYSTEM=1) can export this for you") };
if (!Object.getOwnPropertyDescriptor(Module, "removeRunDependency")) Module["removeRunDependency"] = function() { abort("'removeRunDependency' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ). Alternatively, forcing filesystem support (-s FORCE_FILESYSTEM=1) can export this for you") };
if (!Object.getOwnPropertyDescriptor(Module, "FS_createFolder")) Module["FS_createFolder"] = function() { abort("'FS_createFolder' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ). Alternatively, forcing filesystem support (-s FORCE_FILESYSTEM=1) can export this for you") };
if (!Object.getOwnPropertyDescriptor(Module, "FS_createPath")) Module["FS_createPath"] = function() { abort("'FS_createPath' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ). Alternatively, forcing filesystem support (-s FORCE_FILESYSTEM=1) can export this for you") };
if (!Object.getOwnPropertyDescriptor(Module, "FS_createDataFile")) Module["FS_createDataFile"] = function() { abort("'FS_createDataFile' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ). Alternatively, forcing filesystem support (-s FORCE_FILESYSTEM=1) can export this for you") };
if (!Object.getOwnPropertyDescriptor(Module, "FS_createPreloadedFile")) Module["FS_createPreloadedFile"] = function() { abort("'FS_createPreloadedFile' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ). Alternatively, forcing filesystem support (-s FORCE_FILESYSTEM=1) can export this for you") };
if (!Object.getOwnPropertyDescriptor(Module, "FS_createLazyFile")) Module["FS_createLazyFile"] = function() { abort("'FS_createLazyFile' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ). Alternatively, forcing filesystem support (-s FORCE_FILESYSTEM=1) can export this for you") };
if (!Object.getOwnPropertyDescriptor(Module, "FS_createLink")) Module["FS_createLink"] = function() { abort("'FS_createLink' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ). Alternatively, forcing filesystem support (-s FORCE_FILESYSTEM=1) can export this for you") };
if (!Object.getOwnPropertyDescriptor(Module, "FS_createDevice")) Module["FS_createDevice"] = function() { abort("'FS_createDevice' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ). Alternatively, forcing filesystem support (-s FORCE_FILESYSTEM=1) can export this for you") };
if (!Object.getOwnPropertyDescriptor(Module, "FS_unlink")) Module["FS_unlink"] = function() { abort("'FS_unlink' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ). Alternatively, forcing filesystem support (-s FORCE_FILESYSTEM=1) can export this for you") };
if (!Object.getOwnPropertyDescriptor(Module, "dynamicAlloc")) Module["dynamicAlloc"] = function() { abort("'dynamicAlloc' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "getLEB")) Module["getLEB"] = function() { abort("'getLEB' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "getFunctionTables")) Module["getFunctionTables"] = function() { abort("'getFunctionTables' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "alignFunctionTables")) Module["alignFunctionTables"] = function() { abort("'alignFunctionTables' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "registerFunctions")) Module["registerFunctions"] = function() { abort("'registerFunctions' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "addFunction")) Module["addFunction"] = function() { abort("'addFunction' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "removeFunction")) Module["removeFunction"] = function() { abort("'removeFunction' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "getFuncWrapper")) Module["getFuncWrapper"] = function() { abort("'getFuncWrapper' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "prettyPrint")) Module["prettyPrint"] = function() { abort("'prettyPrint' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "makeBigInt")) Module["makeBigInt"] = function() { abort("'makeBigInt' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "dynCall")) Module["dynCall"] = function() { abort("'dynCall' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "getCompilerSetting")) Module["getCompilerSetting"] = function() { abort("'getCompilerSetting' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "print")) Module["print"] = function() { abort("'print' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "printErr")) Module["printErr"] = function() { abort("'printErr' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "getTempRet0")) Module["getTempRet0"] = function() { abort("'getTempRet0' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "setTempRet0")) Module["setTempRet0"] = function() { abort("'setTempRet0' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "callMain")) Module["callMain"] = function() { abort("'callMain' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "abort")) Module["abort"] = function() { abort("'abort' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "stringToNewUTF8")) Module["stringToNewUTF8"] = function() { abort("'stringToNewUTF8' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "abortOnCannotGrowMemory")) Module["abortOnCannotGrowMemory"] = function() { abort("'abortOnCannotGrowMemory' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "emscripten_realloc_buffer")) Module["emscripten_realloc_buffer"] = function() { abort("'emscripten_realloc_buffer' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "ENV")) Module["ENV"] = function() { abort("'ENV' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "ERRNO_CODES")) Module["ERRNO_CODES"] = function() { abort("'ERRNO_CODES' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "ERRNO_MESSAGES")) Module["ERRNO_MESSAGES"] = function() { abort("'ERRNO_MESSAGES' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "setErrNo")) Module["setErrNo"] = function() { abort("'setErrNo' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "DNS")) Module["DNS"] = function() { abort("'DNS' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "GAI_ERRNO_MESSAGES")) Module["GAI_ERRNO_MESSAGES"] = function() { abort("'GAI_ERRNO_MESSAGES' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "Protocols")) Module["Protocols"] = function() { abort("'Protocols' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "Sockets")) Module["Sockets"] = function() { abort("'Sockets' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "traverseStack")) Module["traverseStack"] = function() { abort("'traverseStack' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "UNWIND_CACHE")) Module["UNWIND_CACHE"] = function() { abort("'UNWIND_CACHE' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "withBuiltinMalloc")) Module["withBuiltinMalloc"] = function() { abort("'withBuiltinMalloc' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "readAsmConstArgsArray")) Module["readAsmConstArgsArray"] = function() { abort("'readAsmConstArgsArray' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "readAsmConstArgs")) Module["readAsmConstArgs"] = function() { abort("'readAsmConstArgs' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "mainThreadEM_ASM")) Module["mainThreadEM_ASM"] = function() { abort("'mainThreadEM_ASM' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "jstoi_q")) Module["jstoi_q"] = function() { abort("'jstoi_q' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "jstoi_s")) Module["jstoi_s"] = function() { abort("'jstoi_s' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "getExecutableName")) Module["getExecutableName"] = function() { abort("'getExecutableName' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "listenOnce")) Module["listenOnce"] = function() { abort("'listenOnce' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "autoResumeAudioContext")) Module["autoResumeAudioContext"] = function() { abort("'autoResumeAudioContext' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "dynCallLegacy")) Module["dynCallLegacy"] = function() { abort("'dynCallLegacy' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "getDynCaller")) Module["getDynCaller"] = function() { abort("'getDynCaller' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "dynCall")) Module["dynCall"] = function() { abort("'dynCall' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "callRuntimeCallbacks")) Module["callRuntimeCallbacks"] = function() { abort("'callRuntimeCallbacks' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "abortStackOverflow")) Module["abortStackOverflow"] = function() { abort("'abortStackOverflow' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "reallyNegative")) Module["reallyNegative"] = function() { abort("'reallyNegative' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "unSign")) Module["unSign"] = function() { abort("'unSign' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "reSign")) Module["reSign"] = function() { abort("'reSign' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "formatString")) Module["formatString"] = function() { abort("'formatString' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "PATH")) Module["PATH"] = function() { abort("'PATH' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "PATH_FS")) Module["PATH_FS"] = function() { abort("'PATH_FS' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "SYSCALLS")) Module["SYSCALLS"] = function() { abort("'SYSCALLS' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "syscallMmap2")) Module["syscallMmap2"] = function() { abort("'syscallMmap2' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "syscallMunmap")) Module["syscallMunmap"] = function() { abort("'syscallMunmap' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "flush_NO_FILESYSTEM")) Module["flush_NO_FILESYSTEM"] = function() { abort("'flush_NO_FILESYSTEM' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "JSEvents")) Module["JSEvents"] = function() { abort("'JSEvents' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "specialHTMLTargets")) Module["specialHTMLTargets"] = function() { abort("'specialHTMLTargets' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "maybeCStringToJsString")) Module["maybeCStringToJsString"] = function() { abort("'maybeCStringToJsString' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "findEventTarget")) Module["findEventTarget"] = function() { abort("'findEventTarget' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "findCanvasEventTarget")) Module["findCanvasEventTarget"] = function() { abort("'findCanvasEventTarget' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "polyfillSetImmediate")) Module["polyfillSetImmediate"] = function() { abort("'polyfillSetImmediate' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "demangle")) Module["demangle"] = function() { abort("'demangle' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "demangleAll")) Module["demangleAll"] = function() { abort("'demangleAll' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "jsStackTrace")) Module["jsStackTrace"] = function() { abort("'jsStackTrace' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "stackTrace")) Module["stackTrace"] = function() { abort("'stackTrace' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "getEnvStrings")) Module["getEnvStrings"] = function() { abort("'getEnvStrings' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "checkWasiClock")) Module["checkWasiClock"] = function() { abort("'checkWasiClock' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "writeI53ToI64")) Module["writeI53ToI64"] = function() { abort("'writeI53ToI64' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "writeI53ToI64Clamped")) Module["writeI53ToI64Clamped"] = function() { abort("'writeI53ToI64Clamped' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "writeI53ToI64Signaling")) Module["writeI53ToI64Signaling"] = function() { abort("'writeI53ToI64Signaling' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "writeI53ToU64Clamped")) Module["writeI53ToU64Clamped"] = function() { abort("'writeI53ToU64Clamped' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "writeI53ToU64Signaling")) Module["writeI53ToU64Signaling"] = function() { abort("'writeI53ToU64Signaling' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "readI53FromI64")) Module["readI53FromI64"] = function() { abort("'readI53FromI64' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "readI53FromU64")) Module["readI53FromU64"] = function() { abort("'readI53FromU64' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "convertI32PairToI53")) Module["convertI32PairToI53"] = function() { abort("'convertI32PairToI53' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "convertU32PairToI53")) Module["convertU32PairToI53"] = function() { abort("'convertU32PairToI53' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "exceptionLast")) Module["exceptionLast"] = function() { abort("'exceptionLast' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "exceptionCaught")) Module["exceptionCaught"] = function() { abort("'exceptionCaught' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "exceptionThrowBuf")) Module["exceptionThrowBuf"] = function() { abort("'exceptionThrowBuf' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "ExceptionInfoAttrs")) Module["ExceptionInfoAttrs"] = function() { abort("'ExceptionInfoAttrs' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "ExceptionInfo")) Module["ExceptionInfo"] = function() { abort("'ExceptionInfo' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "CatchInfo")) Module["CatchInfo"] = function() { abort("'CatchInfo' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "exception_addRef")) Module["exception_addRef"] = function() { abort("'exception_addRef' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "exception_decRef")) Module["exception_decRef"] = function() { abort("'exception_decRef' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "Browser")) Module["Browser"] = function() { abort("'Browser' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "funcWrappers")) Module["funcWrappers"] = function() { abort("'funcWrappers' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "getFuncWrapper")) Module["getFuncWrapper"] = function() { abort("'getFuncWrapper' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "setMainLoop")) Module["setMainLoop"] = function() { abort("'setMainLoop' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "FS")) Module["FS"] = function() { abort("'FS' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "MEMFS")) Module["MEMFS"] = function() { abort("'MEMFS' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "TTY")) Module["TTY"] = function() { abort("'TTY' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "PIPEFS")) Module["PIPEFS"] = function() { abort("'PIPEFS' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "SOCKFS")) Module["SOCKFS"] = function() { abort("'SOCKFS' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "tempFixedLengthArray")) Module["tempFixedLengthArray"] = function() { abort("'tempFixedLengthArray' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "miniTempWebGLFloatBuffers")) Module["miniTempWebGLFloatBuffers"] = function() { abort("'miniTempWebGLFloatBuffers' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "heapObjectForWebGLType")) Module["heapObjectForWebGLType"] = function() { abort("'heapObjectForWebGLType' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "heapAccessShiftForWebGLHeap")) Module["heapAccessShiftForWebGLHeap"] = function() { abort("'heapAccessShiftForWebGLHeap' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "GL")) Module["GL"] = function() { abort("'GL' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "emscriptenWebGLGet")) Module["emscriptenWebGLGet"] = function() { abort("'emscriptenWebGLGet' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "computeUnpackAlignedImageSize")) Module["computeUnpackAlignedImageSize"] = function() { abort("'computeUnpackAlignedImageSize' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "emscriptenWebGLGetTexPixelData")) Module["emscriptenWebGLGetTexPixelData"] = function() { abort("'emscriptenWebGLGetTexPixelData' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "emscriptenWebGLGetUniform")) Module["emscriptenWebGLGetUniform"] = function() { abort("'emscriptenWebGLGetUniform' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "emscriptenWebGLGetVertexAttrib")) Module["emscriptenWebGLGetVertexAttrib"] = function() { abort("'emscriptenWebGLGetVertexAttrib' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "writeGLArray")) Module["writeGLArray"] = function() { abort("'writeGLArray' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "AL")) Module["AL"] = function() { abort("'AL' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "SDL_unicode")) Module["SDL_unicode"] = function() { abort("'SDL_unicode' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "SDL_ttfContext")) Module["SDL_ttfContext"] = function() { abort("'SDL_ttfContext' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "SDL_audio")) Module["SDL_audio"] = function() { abort("'SDL_audio' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "SDL")) Module["SDL"] = function() { abort("'SDL' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "SDL_gfx")) Module["SDL_gfx"] = function() { abort("'SDL_gfx' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "GLUT")) Module["GLUT"] = function() { abort("'GLUT' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "EGL")) Module["EGL"] = function() { abort("'EGL' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "GLFW_Window")) Module["GLFW_Window"] = function() { abort("'GLFW_Window' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "GLFW")) Module["GLFW"] = function() { abort("'GLFW' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "GLEW")) Module["GLEW"] = function() { abort("'GLEW' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "IDBStore")) Module["IDBStore"] = function() { abort("'IDBStore' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "runAndAbortIfError")) Module["runAndAbortIfError"] = function() { abort("'runAndAbortIfError' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "emval_handle_array")) Module["emval_handle_array"] = function() { abort("'emval_handle_array' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "emval_free_list")) Module["emval_free_list"] = function() { abort("'emval_free_list' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "emval_symbols")) Module["emval_symbols"] = function() { abort("'emval_symbols' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "init_emval")) Module["init_emval"] = function() { abort("'init_emval' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "count_emval_handles")) Module["count_emval_handles"] = function() { abort("'count_emval_handles' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "get_first_emval")) Module["get_first_emval"] = function() { abort("'get_first_emval' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "getStringOrSymbol")) Module["getStringOrSymbol"] = function() { abort("'getStringOrSymbol' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "requireHandle")) Module["requireHandle"] = function() { abort("'requireHandle' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "emval_newers")) Module["emval_newers"] = function() { abort("'emval_newers' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "craftEmvalAllocator")) Module["craftEmvalAllocator"] = function() { abort("'craftEmvalAllocator' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "emval_get_global")) Module["emval_get_global"] = function() { abort("'emval_get_global' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "emval_methodCallers")) Module["emval_methodCallers"] = function() { abort("'emval_methodCallers' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "InternalError")) Module["InternalError"] = function() { abort("'InternalError' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "BindingError")) Module["BindingError"] = function() { abort("'BindingError' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "UnboundTypeError")) Module["UnboundTypeError"] = function() { abort("'UnboundTypeError' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "PureVirtualError")) Module["PureVirtualError"] = function() { abort("'PureVirtualError' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "init_embind")) Module["init_embind"] = function() { abort("'init_embind' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "throwInternalError")) Module["throwInternalError"] = function() { abort("'throwInternalError' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "throwBindingError")) Module["throwBindingError"] = function() { abort("'throwBindingError' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "throwUnboundTypeError")) Module["throwUnboundTypeError"] = function() { abort("'throwUnboundTypeError' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "ensureOverloadTable")) Module["ensureOverloadTable"] = function() { abort("'ensureOverloadTable' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "exposePublicSymbol")) Module["exposePublicSymbol"] = function() { abort("'exposePublicSymbol' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "replacePublicSymbol")) Module["replacePublicSymbol"] = function() { abort("'replacePublicSymbol' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "extendError")) Module["extendError"] = function() { abort("'extendError' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "createNamedFunction")) Module["createNamedFunction"] = function() { abort("'createNamedFunction' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "registeredInstances")) Module["registeredInstances"] = function() { abort("'registeredInstances' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "getBasestPointer")) Module["getBasestPointer"] = function() { abort("'getBasestPointer' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "registerInheritedInstance")) Module["registerInheritedInstance"] = function() { abort("'registerInheritedInstance' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "unregisterInheritedInstance")) Module["unregisterInheritedInstance"] = function() { abort("'unregisterInheritedInstance' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "getInheritedInstance")) Module["getInheritedInstance"] = function() { abort("'getInheritedInstance' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "getInheritedInstanceCount")) Module["getInheritedInstanceCount"] = function() { abort("'getInheritedInstanceCount' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "getLiveInheritedInstances")) Module["getLiveInheritedInstances"] = function() { abort("'getLiveInheritedInstances' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "registeredTypes")) Module["registeredTypes"] = function() { abort("'registeredTypes' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "awaitingDependencies")) Module["awaitingDependencies"] = function() { abort("'awaitingDependencies' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "typeDependencies")) Module["typeDependencies"] = function() { abort("'typeDependencies' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "registeredPointers")) Module["registeredPointers"] = function() { abort("'registeredPointers' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "registerType")) Module["registerType"] = function() { abort("'registerType' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "whenDependentTypesAreResolved")) Module["whenDependentTypesAreResolved"] = function() { abort("'whenDependentTypesAreResolved' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "embind_charCodes")) Module["embind_charCodes"] = function() { abort("'embind_charCodes' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "embind_init_charCodes")) Module["embind_init_charCodes"] = function() { abort("'embind_init_charCodes' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "readLatin1String")) Module["readLatin1String"] = function() { abort("'readLatin1String' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "getTypeName")) Module["getTypeName"] = function() { abort("'getTypeName' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "heap32VectorToArray")) Module["heap32VectorToArray"] = function() { abort("'heap32VectorToArray' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "requireRegisteredType")) Module["requireRegisteredType"] = function() { abort("'requireRegisteredType' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "getShiftFromSize")) Module["getShiftFromSize"] = function() { abort("'getShiftFromSize' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "integerReadValueFromPointer")) Module["integerReadValueFromPointer"] = function() { abort("'integerReadValueFromPointer' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "enumReadValueFromPointer")) Module["enumReadValueFromPointer"] = function() { abort("'enumReadValueFromPointer' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "floatReadValueFromPointer")) Module["floatReadValueFromPointer"] = function() { abort("'floatReadValueFromPointer' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "simpleReadValueFromPointer")) Module["simpleReadValueFromPointer"] = function() { abort("'simpleReadValueFromPointer' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "runDestructors")) Module["runDestructors"] = function() { abort("'runDestructors' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "new_")) Module["new_"] = function() { abort("'new_' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "craftInvokerFunction")) Module["craftInvokerFunction"] = function() { abort("'craftInvokerFunction' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "embind__requireFunction")) Module["embind__requireFunction"] = function() { abort("'embind__requireFunction' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "tupleRegistrations")) Module["tupleRegistrations"] = function() { abort("'tupleRegistrations' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "structRegistrations")) Module["structRegistrations"] = function() { abort("'structRegistrations' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "genericPointerToWireType")) Module["genericPointerToWireType"] = function() { abort("'genericPointerToWireType' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "constNoSmartPtrRawPointerToWireType")) Module["constNoSmartPtrRawPointerToWireType"] = function() { abort("'constNoSmartPtrRawPointerToWireType' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "nonConstNoSmartPtrRawPointerToWireType")) Module["nonConstNoSmartPtrRawPointerToWireType"] = function() { abort("'nonConstNoSmartPtrRawPointerToWireType' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "init_RegisteredPointer")) Module["init_RegisteredPointer"] = function() { abort("'init_RegisteredPointer' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "RegisteredPointer")) Module["RegisteredPointer"] = function() { abort("'RegisteredPointer' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "RegisteredPointer_getPointee")) Module["RegisteredPointer_getPointee"] = function() { abort("'RegisteredPointer_getPointee' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "RegisteredPointer_destructor")) Module["RegisteredPointer_destructor"] = function() { abort("'RegisteredPointer_destructor' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "RegisteredPointer_deleteObject")) Module["RegisteredPointer_deleteObject"] = function() { abort("'RegisteredPointer_deleteObject' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "RegisteredPointer_fromWireType")) Module["RegisteredPointer_fromWireType"] = function() { abort("'RegisteredPointer_fromWireType' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "runDestructor")) Module["runDestructor"] = function() { abort("'runDestructor' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "releaseClassHandle")) Module["releaseClassHandle"] = function() { abort("'releaseClassHandle' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "finalizationGroup")) Module["finalizationGroup"] = function() { abort("'finalizationGroup' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "detachFinalizer_deps")) Module["detachFinalizer_deps"] = function() { abort("'detachFinalizer_deps' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "detachFinalizer")) Module["detachFinalizer"] = function() { abort("'detachFinalizer' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "attachFinalizer")) Module["attachFinalizer"] = function() { abort("'attachFinalizer' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "makeClassHandle")) Module["makeClassHandle"] = function() { abort("'makeClassHandle' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "init_ClassHandle")) Module["init_ClassHandle"] = function() { abort("'init_ClassHandle' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "ClassHandle")) Module["ClassHandle"] = function() { abort("'ClassHandle' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "ClassHandle_isAliasOf")) Module["ClassHandle_isAliasOf"] = function() { abort("'ClassHandle_isAliasOf' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "throwInstanceAlreadyDeleted")) Module["throwInstanceAlreadyDeleted"] = function() { abort("'throwInstanceAlreadyDeleted' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "ClassHandle_clone")) Module["ClassHandle_clone"] = function() { abort("'ClassHandle_clone' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "ClassHandle_delete")) Module["ClassHandle_delete"] = function() { abort("'ClassHandle_delete' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "deletionQueue")) Module["deletionQueue"] = function() { abort("'deletionQueue' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "ClassHandle_isDeleted")) Module["ClassHandle_isDeleted"] = function() { abort("'ClassHandle_isDeleted' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "ClassHandle_deleteLater")) Module["ClassHandle_deleteLater"] = function() { abort("'ClassHandle_deleteLater' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "flushPendingDeletes")) Module["flushPendingDeletes"] = function() { abort("'flushPendingDeletes' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "delayFunction")) Module["delayFunction"] = function() { abort("'delayFunction' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "setDelayFunction")) Module["setDelayFunction"] = function() { abort("'setDelayFunction' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "RegisteredClass")) Module["RegisteredClass"] = function() { abort("'RegisteredClass' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "shallowCopyInternalPointer")) Module["shallowCopyInternalPointer"] = function() { abort("'shallowCopyInternalPointer' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "downcastPointer")) Module["downcastPointer"] = function() { abort("'downcastPointer' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "upcastPointer")) Module["upcastPointer"] = function() { abort("'upcastPointer' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "validateThis")) Module["validateThis"] = function() { abort("'validateThis' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "char_0")) Module["char_0"] = function() { abort("'char_0' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "char_9")) Module["char_9"] = function() { abort("'char_9' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "makeLegalFunctionName")) Module["makeLegalFunctionName"] = function() { abort("'makeLegalFunctionName' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "warnOnce")) Module["warnOnce"] = function() { abort("'warnOnce' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "stackSave")) Module["stackSave"] = function() { abort("'stackSave' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "stackRestore")) Module["stackRestore"] = function() { abort("'stackRestore' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "stackAlloc")) Module["stackAlloc"] = function() { abort("'stackAlloc' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "AsciiToString")) Module["AsciiToString"] = function() { abort("'AsciiToString' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "stringToAscii")) Module["stringToAscii"] = function() { abort("'stringToAscii' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "UTF16ToString")) Module["UTF16ToString"] = function() { abort("'UTF16ToString' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "stringToUTF16")) Module["stringToUTF16"] = function() { abort("'stringToUTF16' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "lengthBytesUTF16")) Module["lengthBytesUTF16"] = function() { abort("'lengthBytesUTF16' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "UTF32ToString")) Module["UTF32ToString"] = function() { abort("'UTF32ToString' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "stringToUTF32")) Module["stringToUTF32"] = function() { abort("'stringToUTF32' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "lengthBytesUTF32")) Module["lengthBytesUTF32"] = function() { abort("'lengthBytesUTF32' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "allocateUTF8")) Module["allocateUTF8"] = function() { abort("'allocateUTF8' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "allocateUTF8OnStack")) Module["allocateUTF8OnStack"] = function() { abort("'allocateUTF8OnStack' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
Module["writeStackCookie"] = writeStackCookie;
Module["checkStackCookie"] = checkStackCookie;
if (!Object.getOwnPropertyDescriptor(Module, "intArrayFromBase64")) Module["intArrayFromBase64"] = function() { abort("'intArrayFromBase64' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };
if (!Object.getOwnPropertyDescriptor(Module, "tryParseAsDataURI")) Module["tryParseAsDataURI"] = function() { abort("'tryParseAsDataURI' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") };if (!Object.getOwnPropertyDescriptor(Module, "ALLOC_NORMAL")) Object.defineProperty(Module, "ALLOC_NORMAL", { configurable: true, get: function() { abort("'ALLOC_NORMAL' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") } });
if (!Object.getOwnPropertyDescriptor(Module, "ALLOC_STACK")) Object.defineProperty(Module, "ALLOC_STACK", { configurable: true, get: function() { abort("'ALLOC_STACK' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") } });
if (!Object.getOwnPropertyDescriptor(Module, "ALLOC_NONE")) Object.defineProperty(Module, "ALLOC_NONE", { configurable: true, get: function() { abort("'ALLOC_NONE' was not exported. add it to EXTRA_EXPORTED_RUNTIME_METHODS (see the FAQ)") } });


var calledRun;

/**
 * @constructor
 * @this {ExitStatus}
 */
function ExitStatus(status) {
  this.name = "ExitStatus";
  this.message = "Program terminated with exit(" + status + ")";
  this.status = status;
}

var calledMain = false;


dependenciesFulfilled = function runCaller() {
  // If run has never been called, and we should call run (INVOKE_RUN is true, and Module.noInitialRun is not false)
  if (!calledRun) run();
  if (!calledRun) dependenciesFulfilled = runCaller; // try this again later, after new deps are fulfilled
};





/** @type {function(Array=)} */
function run(args) {
  args = args || arguments_;

  if (runDependencies > 0) {
    return;
  }

  writeStackCookie();

  preRun();

  if (runDependencies > 0) return; // a preRun added a dependency, run will be called later

  function doRun() {
    // run may have just been called through dependencies being fulfilled just in this very frame,
    // or while the async setStatus time below was happening
    if (calledRun) return;
    calledRun = true;
    Module['calledRun'] = true;

    if (ABORT) return;

    initRuntime();

    preMain();

    if (Module['onRuntimeInitialized']) Module['onRuntimeInitialized']();

    assert(!Module['_main'], 'compiled without a main, but one is present. if you added it from JS, use Module["onRuntimeInitialized"]');

    postRun();
  }

  if (Module['setStatus']) {
    Module['setStatus']('Running...');
    setTimeout(function() {
      setTimeout(function() {
        Module['setStatus']('');
      }, 1);
      doRun();
    }, 1);
  } else
  {
    doRun();
  }
  checkStackCookie();
}
Module['run'] = run;

function checkUnflushedContent() {
  // Compiler settings do not allow exiting the runtime, so flushing
  // the streams is not possible. but in ASSERTIONS mode we check
  // if there was something to flush, and if so tell the user they
  // should request that the runtime be exitable.
  // Normally we would not even include flush() at all, but in ASSERTIONS
  // builds we do so just for this check, and here we see if there is any
  // content to flush, that is, we check if there would have been
  // something a non-ASSERTIONS build would have not seen.
  // How we flush the streams depends on whether we are in SYSCALLS_REQUIRE_FILESYSTEM=0
  // mode (which has its own special function for this; otherwise, all
  // the code is inside libc)
  var print = out;
  var printErr = err;
  var has = false;
  out = err = function(x) {
    has = true;
  }
  try { // it doesn't matter if it fails
    var flush = null;
    if (flush) flush();
  } catch(e) {}
  out = print;
  err = printErr;
  if (has) {
    warnOnce('stdio streams had content in them that was not flushed. you should set EXIT_RUNTIME to 1 (see the FAQ), or make sure to emit a newline when you printf etc.');
    warnOnce('(this may also be due to not including full filesystem support - try building with -s FORCE_FILESYSTEM=1)');
  }
}

/** @param {boolean|number=} implicit */
function exit(status, implicit) {
  checkUnflushedContent();

  // if this is just main exit-ing implicitly, and the status is 0, then we
  // don't need to do anything here and can just leave. if the status is
  // non-zero, though, then we need to report it.
  // (we may have warned about this earlier, if a situation justifies doing so)
  if (implicit && noExitRuntime && status === 0) {
    return;
  }

  if (noExitRuntime) {
    // if exit() was called, we may warn the user if the runtime isn't actually being shut down
    if (!implicit) {
      var msg = 'program exited (with status: ' + status + '), but EXIT_RUNTIME is not set, so halting execution but not exiting the runtime or preventing further async execution (build with EXIT_RUNTIME=1, if you want a true shutdown)';
      err(msg);
    }
  } else {

    EXITSTATUS = status;

    exitRuntime();

    if (Module['onExit']) Module['onExit'](status);

    ABORT = true;
  }

  quit_(status, new ExitStatus(status));
}

if (Module['preInit']) {
  if (typeof Module['preInit'] == 'function') Module['preInit'] = [Module['preInit']];
  while (Module['preInit'].length > 0) {
    Module['preInit'].pop()();
  }
}


  noExitRuntime = true;

run();






// {{MODULE_ADDITIONS}}



