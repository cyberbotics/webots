#!/usr/bin/env python3

# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""PROTO tag replacer"""

import os
import glob
import argparse

VALID_TAGS = ['static', 'nonDeterministic', 'hidden', 'deprecated']


def parse_tag_line(line):
    # faster to just check if it contains the tags than dealing with the formatting
    existing_tags = []
    for tag in VALID_TAGS:
        if tag in line:
            existing_tags.append(tag)
    return existing_tags


def generate_tag_line(tags):
    if tags:
        line = '# tags: '
        for tag in tags:
            line = line + tag + ', '
        line = line[:-2] + '\n'
    else:
        line = ''
    return line


def has_tag(proto_path, tag):
    has_tag = False
    has_tag_line = False
    with open(proto_path, 'r') as file:
        while(True):
            line = file.readline()
            if line.startswith('#'):
                if line.startswith('#tags:') or line.startswith('# tags:'):
                    has_tag_line = True
                    if tag in line:
                        has_tag = True
                    break
            else:
                break
    return has_tag_line, has_tag


def get_proto_files(target_directory, no_recursion):
    if target_directory:
        search_path = target_directory
        if target_directory.endswith('/'):
            search_path = search_path + '**/*.proto'
        else:
            search_path = search_path + '/**/*.proto'

        proto_files = glob.glob(search_path, recursive=~no_recursion)
    else:
        WEBOTS_HOME = os.environ['WEBOTS_HOME']
        proto_files = glob.glob(WEBOTS_HOME + '/**/*.proto', recursive=~no_recursion)

    if proto_files:
        print('> found %d PROTO files' % len(proto_files))
    else:
        print('> no PROTO files found in: ' + target_directory + ', aborting.')
        exit()

    return proto_files


def filter_by_keywords(proto_files, keywords):
    filtered_list = []
    if keywords:
        for key in keywords:
            for proto in proto_files:
                with open(proto, 'r') as file:
                    if key in file.read() and proto not in filtered_list:
                        filtered_list.append(proto)
    else:
        print('> no keyword filter used, targetting all PROTO files.')
        return proto_files

    if filtered_list:
        print('> %d PROTO files remain after filtering by keywords.' % len(filtered_list))
    else:
        print('> no PROTO files satisfy the keywords filtering condition, aborting.')
        exit()

    return filtered_list


def filter_by_tag(proto_files, tag):
    filtered_list = []

    if tag:
        for proto in proto_files:
            tag_line_present, tag_present = has_tag(proto, tag)
            if tag != 'no_tag' and tag_present:
                filtered_list.append(proto)
            if tag == 'no_tag' and not tag_line_present:
                filtered_list.append(proto)
    else:
        print('> no tag filters used, targetting all PROTO files.')
        return proto_files

    if filtered_list:
        print('> %d PROTO files remain after filtering by tag.' % len(filtered_list))
    else:
        print('> no PROTO files satisfy the tag filtering condition, aborting.')
        exit()

    return filtered_list


def add_tag_to_proto(proto_files, tag_to_add):
    for proto in proto_files:
        with open(proto, 'r+') as file:
            file_lines = file.readlines()
            # check if any tag is present, otherwise add new line below the first
            for i, line in enumerate(file_lines):
                if line.startswith('#'):
                    if line.startswith('#tags:') or line.startswith('# tags:'):
                        current_tags = parse_tag_line(line)
                        if tag_to_add in current_tags:
                            # already present, but ensure consistent formatting
                            if line.startswith('#tags:'):
                                file_lines[i] = line[:1] + ' ' + line[1:]
                        else:
                            current_tags.append(tag_to_add)
                            file_lines[i] = generate_tag_line(current_tags)

                        file.seek(0)
                        file.writelines(file_lines)
                        file.truncate()
                        break
                else:
                    # no existing tag line, create a new one
                    file.seek(0)
                    file_lines.insert(1, '# tags: ' + tag_to_add + '\n')
                    file.writelines(file_lines)
                    file.truncate()
                    break


def remove_tag_from_proto(proto_files, tag_to_remove):
    for proto in proto_files:
        with open(proto, 'r+') as file:
            file_lines = file.readlines()
            # search for tag line, if any
            for i, line in enumerate(file_lines):
                if line.startswith('#'):
                    if line.startswith('#tags:') or line.startswith('# tags:'):
                        current_tags = parse_tag_line(line)
                        if tag_to_remove in current_tags:  # tag is present, remove it
                            current_tags.remove(tag_to_remove)
                            new_line = generate_tag_line(current_tags)
                            if new_line == '':
                                del file_lines[i]  # remove tag line entirely if no tags remain after removal
                            else:
                                file_lines[i] = new_line
                            file.seek(0)
                            file.writelines(file_lines)
                            file.truncate()
                            break
                else:
                    break


def replace_tag_in_proto(proto_files, tag_to_replace):
    for proto in proto_files:
        with open(proto, 'r+') as file:
            file_lines = file.readlines()
            # search for tag line, if any
            for i, line in enumerate(file_lines):
                if line.startswith('#'):
                    if line.startswith('#tags:') or line.startswith('# tags:'):
                        current_tags = parse_tag_line(line)
                        if tag_to_replace[0] in current_tags:
                            current_tags.remove(tag_to_replace[0])
                            if tag_to_replace[1] not in current_tags:  # only add second tag if isn't there already
                                current_tags.append(tag_to_replace[1])
                            file_lines[i] = generate_tag_line(current_tags)
                            file.seek(0)
                            file.writelines(file_lines)
                            file.truncate()
                            break
                        else:  # do nothing if the first is missing
                            break
                else:
                    break


def main():
    parser = argparse.ArgumentParser(description='Manipulate tags in PROTO files')
    filter_group = parser.add_argument_group('filter', 'allows to filter the list of PROTO files, by keyword or by tag')
    filter_group.add_argument('--dir', type=str, metavar=('PATH'), dest='target_directory',
                              help='specifies the directory where to search for PROTO files. \
                              If unspecified, WEBOTS_HOME is used.')
    filter_group.add_argument('--no-recursion', metavar='', default=False, help='disables recursive search')
    filter_group.add_argument('--filter-tag', type=str, metavar=('TAG'), choices=VALID_TAGS.append('no_tag'), dest='filter_tag',
                              help='filters the list of PROTO files and keeps only those that have the provided tag. \
                              Use \'no_tag\' to select PROTO files that have no tags.')
    filter_group.add_argument('--filter-keywords', nargs='+', type=str, metavar=('KEYWORDS'), dest='filter_keywords',
                              help='filters the list of PROTO files and keeps only those that contain any of the provided \
                              keywords in the file.')
    modify_group = parser.add_mutually_exclusive_group(required=True)
    modify_group.add_argument('--remove', type=str, metavar=('TAG'), choices=VALID_TAGS, dest='tag_to_remove',
                              help='removes the provided tag from the list of PROTO files')
    modify_group.add_argument('--add', type=str, metavar=('TAG'), choices=VALID_TAGS, dest='tag_to_add',
                              help='adds the provided tag to the list of PROTO files')
    modify_group.add_argument('--replace', nargs=2, metavar=('TAG', 'WITH_TAG'), choices=VALID_TAGS, dest='tag_to_replace',
                              help='replaces any occurrence of the first tag with the second')
    modify_group.add_argument('--list', action='store_true', help='lists PROTO paths that satisfy the filter')

    args = parser.parse_args()
    proto_files = get_proto_files(args.target_directory, args.no_recursion)
    proto_files = filter_by_keywords(proto_files, args.filter_keywords)
    proto_files = filter_by_tag(proto_files, args.filter_tag)

    if args.list:
        print('> listing\n')
        for proto in proto_files:
            print(proto)
    elif args.tag_to_add:
        add_tag_to_proto(proto_files, args.tag_to_add)
    elif args.tag_to_remove:
        remove_tag_from_proto(proto_files, args.tag_to_remove)
    elif args.tags_to_replace:
        replace_tag_in_proto(proto_files, args.tag_to_replace)


if __name__ == '__main__':
    main()
