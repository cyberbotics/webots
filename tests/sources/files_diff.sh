#!/usr/bin/env bash

# this bash script returns the list of modified files in a git child branch
parent_branch=`git show-branch | grep -F '*' | grep -v "$(git rev-parse --abbrev-ref HEAD)" | head -n1 | \
sed 's/.*\[\(.*\)\].*/\1/' | sed 's/[\^~].*//'`
git diff --name-only origin/$parent_branch
