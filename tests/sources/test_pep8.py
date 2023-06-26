#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Test module of the PEP8 format in the tests."""
import unittest

import _ast
import fnmatch
import pycodestyle
import os
import sys
from io import open  # needed for compatibility with Python 2.7 for open(file, encoding='utf-8')

from pyflakes import checker
from pyflakes.reporter import Reporter

WEBOTS_HOME = os.path.normpath(os.environ['WEBOTS_HOME'])

skippedDirectories = [
    '.git',
    'dependencies',
    'projects/robots/mobsya/thymio/controllers/thymio2_aseba/aseba',
    'projects/robots/robotis/darwin-op/libraries/python',
    'resources/webots_ros/'
]
skippedDirectoriesFull = [os.path.join(WEBOTS_HOME, os.path.normpath(path)) for path in skippedDirectories]


class FlakesReporter(Reporter):
    """Flakes reporter."""

    def __init__(self):
        self.error = ''

    def unexpectedError(self, filename, msg):
        """Handle unexpected errors."""
        self.error += '%s: %s\n' % (filename, msg)

    def syntaxError(self, filename, msg, lineno, offset, text):
        """Handle syntax errors."""
        line = text.splitlines()[-1]
        if offset is not None:
            offset = offset - (len(text) - len(line))
        self.error += '%s:%d: %s\n' % (filename, lineno, msg)
        self.error += line + '\n'
        if offset is not None:
            self.error += ' ' * (offset + 1) + '^\n'

    def flake(self, message):
        """Add message to error string."""
        self.error += str(message) + '\n'


def checkFlakes(codeString, filename, reporter):
    """Check the Python source given by codeString} for flakes."""
    # First, compile into an AST and handle syntax errors.
    try:
        tree = compile(codeString, filename, "exec", _ast.PyCF_ONLY_AST)
    except SyntaxError:
        value = sys.exc_info()[1]
        msg = value.args[0]
        (lineno, offset, text) = value.lineno, value.offset, value.text
        # If there's an encoding problem with the file, the text is None.
        if text is None:
            # Avoid using msg, since for the only known case, it contains a
            # bogus message that claims the encoding the file declared was
            # unknown.
            reporter.unexpectedError(filename, 'problem decoding source')
        else:
            reporter.syntaxError(filename, msg, lineno, offset, text)
        return
    except Exception:
        reporter.unexpectedError(filename, 'problem decoding source')
        return
    # Okay, it's syntactically valid. Now check it.
    w = checker.Checker(tree, filename)
    w.messages.sort(key=lambda m: m.lineno)
    for warning in w.messages:
        reporter.flake(warning)


def checkFlakesPath(filename, reporter):
    """Check the given path, printing out any warnings detected."""
    try:
        with open(filename, encoding='utf-8') as f:
            codestr = f.read() + '\n'
    except UnicodeError:
        reporter.unexpectedError(filename, 'problem decoding source')
        return
    except IOError:
        msg = sys.exc_info()[1]
        reporter.unexpectedError(filename, msg.args[1])
        return
    checkFlakes(codestr.encode('utf-8'), filename, reporter)


class CustomReport(pycodestyle.StandardReport):
    """Collect report, and overload the string operator."""

    results = []

    def get_file_results(self):
        """Overload this function to collect the report."""
        if self._deferred_print:
            self._deferred_print.sort()
            for line_number, offset, code, text, _ in self._deferred_print:
                self.results.append({
                    'path': self.filename,
                    'row': self.line_offset + line_number,
                    'col': offset + 1,
                    'code': code,
                    'text': text,
                })
        return self.file_errors

    def __str__(self):
        """Overload the string operator."""
        output = 'Report:\n'
        for result in self.results:
            output += '%s:%d:%d: %s: %s\n' % (
                result['path'],
                result['row'],
                result['col'],
                result['code'],
                result['text']
            )
        return output


class TestCodeFormat(unittest.TestCase):
    """Unit test of the PEP8 format in the tests."""

    def setUp(self):
        """Get all the python files."""
        self.files = []
        for rootPath, dirNames, fileNames in os.walk(WEBOTS_HOME):
            for fileName in fnmatch.filter(fileNames, '*.py'):
                shouldContinue = False
                for skippedDirectory in skippedDirectoriesFull:
                    if rootPath.startswith(skippedDirectory):
                        shouldContinue = True
                        break
                if shouldContinue:
                    continue
                filePath = os.path.join(rootPath, fileName)
                if sys.version_info[0] < 3:
                    with open(filePath) as file:
                        if file.readline().startswith('#!/usr/bin/env python3'):
                            continue
                self.files.append(filePath)

    def test_pep8_conformance(self):
        """Test that the tests are PEP8 compliant."""
        # Use pep8 module to detect 'W' and 'E' errors
        checker = pycodestyle.StyleGuide(
            quiet=True,
            paths=self.files,
            reporter=CustomReport,
        )
        checker.options.max_line_length = 128
        report = checker.check_files()
        self.assertEqual(
            report.total_errors, 0,
            msg='PEP8 errors:\n%s' % (report)
        )
        # Use flakes module to detect 'F' errors
        reporter = FlakesReporter()
        for fileName in self.files:
            checkFlakesPath(fileName, reporter)
        self.assertTrue(
            not reporter.error,
            msg='PEP8 errors:\n%s' % (reporter.error)
        )


if __name__ == '__main__':
    unittest.main()
