#!/usr/bin/env bash

# This script prints the parent branch name of the current branch
# It is used by the generate_diff.py script

git show-branch | grep '\*' | grep -v "$(git rev-parse --abbrev-ref HEAD)" | head -n1 | sed 's/.*\[\(.*\)\].*/\1/' | sed 's/[\^~].*//'
