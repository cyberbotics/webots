#!/bin/bash

# Get the name of the working branch and write it to a file if correct
branch=$(git branch 2>&1)

if [[ $branch == '* (HEAD detached at origin/'* ]]
then
	echo "$branch" | $(sed -n -e 's/^\* \(.*\)/\1/p' | cut -c 26- | rev | cut -c 2- | rev > $WEBOTS_HOME/resources/branch.txt)
elif [[ $branch == '* (HEAD detached at '* ]]
then
	echo "$branch" | $(sed -n -e 's/^\* \(.*\)/\1/p' | cut -c 19- | rev | cut -c 2- | rev > $WEBOTS_HOME/resources/branch.txt)
elif [[ $branch != fatal* ]]
then
	echo "$branch" | $(sed -n -e 's/^\* \(.*\)/\1/p' > $WEBOTS_HOME/resources/branch.txt)
else
	echo "master" > $WEBOTS_HOME/resources/branch.txt
fi

# Get the name of the github repository and write it to a file if correct
repo=$(git config --get remote.origin.url 2>&1)

if [[ $repo == git@github.com* ]]
then
	echo "$repo" | $(cut -c 16- | rev | cut -c 5- | rev > $WEBOTS_HOME/resources/repo.txt)
elif [[ $repo == https://github.com* ]]
then
	echo "$repo" | $(cut -c 20- | rev | cut -c 5- | rev > $WEBOTS_HOME/resources/repo.txt)
else
	echo "cyberbotics/webots" > $WEBOTS_HOME/resources/repo.txt
fi

# Get the name of the github commit and write it to a file if correct
commit=$(git rev-parse --short HEAD 2> /dev/null | sed "s/\(.*\)/\1/")

if [[ $commit != "" ]]
then
	echo "$commit" > $WEBOTS_HOME/resources/commit.txt
fi
