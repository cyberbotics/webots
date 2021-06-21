#!/bin/bash

# Get the name of the working branch and write it to a file if correct
branch=$(git branch 2>&1)

if [[ $branch != fatal* ]] && [[ $branch != '* (HEAD'* ]]
then
	echo "$branch" | $(sed -n -e 's/^\* \(.*\)/\1/p' > $WEBOTS_HOME/resources/branch.txt)
fi

# Get the name of the github repository and write it to a file if correct
repo=$(git config --get remote.origin.url 2>&1)

if [[ $branch != "" ]]
then
	echo "$repo" | $(cut -c 20- | rev | cut -c 5- | rev > $WEBOTS_HOME/resources/repo.txt)
fi
