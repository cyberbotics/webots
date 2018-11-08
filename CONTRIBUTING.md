# Contributing

We love pull requests from everyone.

## Install the development environment

* For [Windows](/omichel/webots/wiki/Windows-installation/)
* For [Linux](/omichel/webots/wiki/Linux-installation/)
* For [macOS](/omichel/webots/wiki/macOS-installation/)

## Create a Pull Request

1. Fork the repository: https://help.github.com/articles/fork-a-repo
2. Create a branch in your fork: https://help.github.com/articles/creating-and-deleting-branches-within-your-repository
3. Pull the branch as a pull request targeting `omichel:webots@master`: https://help.github.com/articles/creating-a-pull-request-from-a-fork
4. Wait for our awesome review :-)

Our git workflow is explained in detail [here](/omichel/webots-dev/wiki/Git-workflow/).

## Development Guideline

* Follow our [Coding Style](/omichel/webots/wiki/Coding-Style/).
* Avoid comitting files that exist elsewhere. Instead we should link to the source of these files.
* Avoid comitting files that can be re-created from other files using a Makefile, a script or a compiler.

## Untit test

* Before merging any pull request the unit test should pass. 
