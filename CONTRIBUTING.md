# Contributing

We love pull requests from everyone.

## Install the development environment

* Follow the installation instructions in our [wiki](https://github.com/cyberbotics/webots/wiki/).

## Implement your contribution

Unless you have a good idea to implement, you can browse the [beginner issues](https://github.com/cyberbotics/webots/issues?utf8=%E2%9C%93&q=is%3Aissue+is%3Aopen+label%3A%22beginner+issue%22) and select a feature or bug fix that should be fairly easy to implement.

## Create a Pull Request

1. Fork the repository: https://help.github.com/articles/fork-a-repo
2. Create a branch in your fork: https://help.github.com/articles/creating-and-deleting-branches-within-your-repository
3. Pull the branch as a pull request targeting `cyberbotics:webots@master`: https://help.github.com/articles/creating-a-pull-request-from-a-fork
4. Wait for our unit tests and review of your pull request.

Our git workflow is explained in detail [here](https://github.com/cyberbotics/webots/wiki/Git-workflow/).

## Development Guideline

* Follow our [Coding Style](https://github.com/cyberbotics/webots/wiki/Coding-Style/).
* Avoid comitting files that exist elsewhere. Instead we should link to the source of these files.
* Avoid comitting files that can be re-created from other files using a Makefile, a script or a compiler.
