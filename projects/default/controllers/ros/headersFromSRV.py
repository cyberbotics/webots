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

import os
import sys
import getopt

from headersGenerator import HeadersGenerator


def main(argv):
    headersGenerator = HeadersGenerator()
    try:
        opts, args = getopt.getopt(argv, "h", ["help"])
    except getopt.GetoptError:
        print('usage: headersFromSRV.py <servicefile.srv> [templatesFolder]')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('usage: headersFromSRV.py <servicefile.srv> [templatesFolder]')
            sys.exit(2)
        elif opt in ("-h", "--help"):
            print('usage: headersFromSRV.py <servicefile.srv> [templatesFolder]')
            sys.exit(2)
    SRVFile = open(args[0], 'r')
    filename = args[0].split('/')[-1]
    filename = filename.split('.')[0]
    templateFolder = 'include'
    if len(args) > 1:
        templateFolder = args[1]
    headerType = 'request'
    requestType = []
    requestName = []
    responseType = []
    responseName = []
    for line in SRVFile:
        if len(line.split()) == 1:
            headerType = 'response'
        else:
            message = line.split()
            if headerType == 'request':
                requestType.append(message[0])
                requestName.append(message[1])
            else:
                responseType.append(message[0])
                responseName.append(message[1])
    SRVFile.close()
    requestName.reverse()
    requestType.reverse()
    responseName.reverse()
    responseType.reverse()

    if not os.path.exists('include/webots_ros'):
        try:
            os.makedirs('include/webots_ros')
        except OSError:
            pass

    headersGenerator.replace_template_tags(templateFolder + '/templateHeader.h', 'include/webots_ros/' + filename + '.h',
                                           filename, args[0])

    headersGenerator.replace_template_tags(templateFolder + '/templateRequest.h', filename + 'tempFile.txt', filename, args[0])
    headersGenerator.replace_message_tags(filename + 'tempFile.txt', 'include/webots_ros/' + filename + 'Request.h',
                                          requestName, requestType)

    try:
        os.remove(filename + 'tempFile.txt')
    except OSError:
        print('failed to remove tempFile for ' + filename)

    headersGenerator.replace_template_tags(templateFolder + '/templateResponse.h', filename + 'tempFile.txt', filename, args[0])
    headersGenerator.replace_message_tags(filename + 'tempFile.txt', 'include/webots_ros/' + filename + 'Response.h',
                                          responseName, responseType)

    try:
        os.remove(filename + 'tempFile.txt')
    except OSError:
        print('failed to remove tempFile for ' + filename)


if __name__ == "__main__":
    main(sys.argv[1:])
