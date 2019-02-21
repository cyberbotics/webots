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

from __future__ import unicode_literals
import codecs
import os
import re

# unit test suites are not good about screening out illegal unicode characters (#603)
# recipe from http://boodebr.org/main/python/all-about-python-and-unicode#UNI_XML
# code copied from rosunit/src/junitxml.py
try:
    char = unichr
except NameError:
    char = chr
RE_XML_ILLEGAL = ('([%s-%s%s-%s%s-%s%s-%s])' + \
    '|' + \
    '([%s-%s][^%s-%s])|([^%s-%s][%s-%s])|([%s-%s]$)|(^[%s-%s])') % \
    (char(0x0000), char(0x0008), char(0x000b), char(0x000c),
     char(0x000e), char(0x001f), char(0xfffe), char(0xffff),
     char(0xd800), char(0xdbff), char(0xdc00), char(0xdfff),
     char(0xd800), char(0xdbff), char(0xdc00), char(0xdfff),
     char(0xd800), char(0xdbff), char(0xdc00), char(0xdfff))
_SAFE_XML_REGEX = re.compile(RE_XML_ILLEGAL)


def tidy_xml(filename):
    '''
    read in file, screen out unsafe unicode characters, write back file in utf-8

    :param filename: str
    :returns: False if unable to read from file
    '''
    if not os.path.isfile(filename):
        raise ValueError('file does not exist')

    # try first utf-8 then iso. This is ugly, but the files in
    # question that are problematic do not declare unicode type
    data = None
    for ftype in ['utf-8', 'iso8859-1']:
        fhand = None
        try:
            fhand = codecs.open(filename, 'r', ftype)
            data = fhand.read()
            break
        except ValueError:
            continue
        finally:
            if fhand is not None:
                fhand.close()

    if data is None:
        return False

    for match in _SAFE_XML_REGEX.finditer(data):
        data = data[:match.start()] + '?' + data[match.end():]

    with open(filename, 'w') as fhand:
        fhand.write(data)
    return True
