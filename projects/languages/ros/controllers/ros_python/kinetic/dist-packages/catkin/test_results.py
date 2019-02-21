# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
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

from __future__ import print_function
import errno
import os
import sys
from xml.etree.ElementTree import ElementTree, ParseError

from catkin.tidy_xml import tidy_xml


def remove_junit_result(filename):
    # if result file exists remove it before test execution
    if os.path.exists(filename):
        os.remove(filename)
    # if placeholder (indicating previous failure) exists remove it before test execution
    missing_filename = _get_missing_junit_result_filename(filename)
    if os.path.exists(missing_filename):
        os.remove(missing_filename)


def ensure_junit_result_exist(filename):
    if os.path.exists(filename):
        # if result file exists ensure that it contains valid xml
        tree = None
        try:
            tree = ElementTree(None, filename)
        except ParseError:
            # print('Invalid XML in result file "%s"' % filename)
            tidy_xml(filename)
            try:
                tree = ElementTree(None, filename)
            except ParseError as e:
                print("Invalid XML in result file '%s' (even after trying to tidy it): %s " % (filename, str(e)), file=sys.stderr)
                return False
        if tree:
            _, num_errors, num_failures = read_junit(filename)
            if num_errors or num_failures:
                return False
    else:
        # if result file does not exist create placeholder which indicates failure
        missing_filename = _get_missing_junit_result_filename(filename)
        print("Cannot find results, writing failure results to '%s'" % missing_filename, file=sys.stderr)
        # create folder if necessary
        if not os.path.exists(os.path.dirname(filename)):
            try:
                os.makedirs(os.path.dirname(filename))
            except OSError as e:
                # catch case where folder has been created in the mean time
                if e.errno != errno.EEXIST:
                    raise
        with open(missing_filename, 'w') as f:
            data = {'test': os.path.basename(filename), 'test_file': filename}
            f.write('''<?xml version="1.0" encoding="UTF-8"?>
<testsuite tests="1" failures="1" time="1" errors="0" name="%(test)s">
  <testcase name="test_ran" status="run" time="1" classname="Results">
    <failure message="Unable to find test results for %(test)s, test did not run.\nExpected results in %(test_file)s" type=""/>
  </testcase>
</testsuite>''' % data)
        return False
    return True


def _get_missing_junit_result_filename(filename):
    return os.path.join(os.path.dirname(filename), 'MISSING-%s' % os.path.basename(filename))


def read_junit(filename):
    """Same as `read_junit2` except it doesn't return num_skipped."""
    num_tests, num_errors, num_failures, _ = read_junit2(filename)
    return (num_tests, num_errors, num_failures)


def read_junit2(filename):
    """
    parses xml file expected to follow junit/gtest conventions
    see http://code.google.com/p/googletest/wiki/AdvancedGuide#Generating_an_XML_Report

    :param filename: str junit xml file name
    :returns: num_tests, num_errors, num_failures, num_skipped
    :raises ParseError: if xml is not well-formed
    :raises IOError: if filename does not exist
    """
    tree = ElementTree()
    root = tree.parse(filename)
    num_tests = int(root.attrib['tests'])
    num_errors = int(root.attrib['errors'])
    num_failures = int(root.attrib['failures'])
    num_skipped = int(root.get('skip', '0')) + int(root.get('disabled', '0'))
    return (num_tests, num_errors, num_failures, num_skipped)


def test_results(test_results_dir, show_verbose=False, show_all=False):
    """Same as `test_results2` except the returned values don't include num_skipped."""
    results = {}
    results2 = test_results2(
        test_results_dir, show_verbose=show_verbose, show_all=show_all)
    for name, values in results2.items():
        num_tests, num_errors, num_failures, _ = values
        results[name] = (num_tests, num_errors, num_failures)
    return results


def test_results2(test_results_dir, show_verbose=False, show_all=False):
    '''
    Collects test results by parsing all xml files in given path,
    attempting to interpret them as junit results.

    :param test_results_dir: str foldername
    :param show_verbose: bool show output for tests which had errors or failed
    :returns: dict {rel_path, (num_tests, num_errors, num_failures, num_skipped)}
    '''
    results = {}
    for dirpath, dirnames, filenames in os.walk(test_results_dir):
        # do not recurse into folders starting with a dot
        dirnames[:] = [d for d in dirnames if not d.startswith('.')]
        for filename in [f for f in filenames if f.endswith('.xml')]:
            filename_abs = os.path.join(dirpath, filename)
            name = filename_abs[len(test_results_dir) + 1:]
            try:
                num_tests, num_errors, num_failures, num_skipped = read_junit2(filename_abs)
            except Exception as e:
                if show_all:
                    print('Skipping "%s": %s' % (name, str(e)))
                continue
            results[name] = (num_tests, num_errors, num_failures, num_skipped)
            if show_verbose and (num_errors + num_failures > 0):
                print("Full test results for '%s'" % (name))
                print('-------------------------------------------------')
                with open(filename_abs, 'r') as f:
                    print(f.read())
                print('-------------------------------------------------')
    return results


def aggregate_results(results, callback_per_result=None):
    """Same as `aggregate_results2` except it doesn't return num_skipped."""
    callback = None
    if callback_per_result is not None:
        def callback(name, num_tests, num_errors, num_failures, num_skipped):
            callback_per_result(name, num_tests, num_errors, num_failures)
    sum_tests, sum_errors, sum_failures, _ = aggregate_results2(
        results, callback_per_result=callback)
    return (sum_tests, sum_errors, sum_failures)


def aggregate_results2(results, callback_per_result=None):
    """
    Aggregate results

    :param results: dict as from test_results()
    :returns: tuple (num_tests, num_errors, num_failures, num_skipped)
    """
    sum_tests = sum_errors = sum_failures = sum_skipped = 0
    for name in sorted(results.keys()):
        (num_tests, num_errors, num_failures, num_skipped) = results[name]
        sum_tests += num_tests
        sum_errors += num_errors
        sum_failures += num_failures
        sum_skipped += num_skipped
        if callback_per_result:
            callback_per_result(
                name, num_tests, num_errors, num_failures, num_skipped)
    return sum_tests, sum_errors, sum_failures, sum_skipped


def print_summary(results, show_stable=False, show_unstable=True):
    """Same as `print_summary2` except it doesn't print skipped tests."""
    print_summary2(
        results, show_stable=show_stable, show_unstable=show_unstable,
        print_skipped=False)


def print_summary2(results, show_stable=False, show_unstable=True, print_skipped=True):
    """
    print summary to stdout

    :param results: dict as from test_results()
    :param show_stable: print tests without failures extra
    :param show_stable: print tests with failures extra
    :param print_skipped: include skipped tests in output
    """
    def callback(name, num_tests, num_errors, num_failures, num_skipped):
        if show_stable and not num_errors and not num_failures and not num_skipped:
            print('%s: %d tests' % (name, num_tests))
        if show_unstable and (num_errors or num_failures or num_skipped):
            msg = '{}: {} tests, {} errors, {} failures'
            msg_args = [name, num_tests, num_errors, num_failures]
            if print_skipped:
                msg += ', {} skipped'
                msg_args.append(num_skipped)
            print(msg.format(*msg_args))
    sum_tests, sum_errors, sum_failures, sum_skipped = aggregate_results2(results, callback)

    msg = 'Summary: {} tests, {} errors, {} failures'
    msg_args = [sum_tests, sum_errors, sum_failures]
    if print_skipped:
        msg += ', {} skipped'
        msg_args.append(sum_skipped)

    print(msg.format(*msg_args))
