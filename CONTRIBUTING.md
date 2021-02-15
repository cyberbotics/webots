# Contributing

Thank you for your interest in contributing to Webots!
The following is a set of guidelines for helping you to contribute to Webots.

## Required skills

You don't need to be an expert in robotics or software development to become a contributor.
Depending on your skills, your contribution may address different parts of the Webots software:

- Bug reporting: [A precise description](https://github.com/cyberbotics/webots/issues/new?template=bug_report.md) of a reproducible bug is very helpful to us.
- Technical English writing: [documentation pages](https://github.com/cyberbotics/webots/tree/released/docs).
- Foreign language: [translate the user interface](https://github.com/cyberbotics/webots/tree/master/resources/translations) in your own language.
- Python programming: [sample simulations](https://github.com/cyberbotics/webots/tree/master/projects/languages/python/controllers), libraries, tools such as [urdf2webots](https://github.com/cyberbotics/urdf2webots), etc.
- C/C++ programming: [webots source code](https://github.com/cyberbotics/webots/tree/master/src), sample simulations, libraries, tools, etc.
- 3D modeling: contribute models of [objects](https://github.com/cyberbotics/webots/tree/master/projects/objects), [devices](https://github.com/cyberbotics/webots/tree/master/projects/devices) or [robots](https://github.com/cyberbotics/webots/tree/master/projects/robots).

In any case, you should have a minimal knowledge of GitHub to fork our repository and create a Pull Request that we will review and hopefully accept.

For C/C++ contributions is strongly recommended that you install the Webots development environment as explained below.

If your contribution is external to Webots, e.g., a sample simulation, a PROTO model, a controller library, etc., you may be interested to contribute it to the [community projects](https://github.com/cyberbotics/community-projects) repository.

## Need ideas to contribute?

Unless you have a good idea to implement, you can browse the [beginner issues](https://github.com/cyberbotics/webots/issues?utf8=%E2%9C%93&q=is%3Aissue+is%3Aopen+label%3A%22good+first+issue%22) and select a feature or bug fix that should be fairly easy to implement.
Once you will become an experienced Webots developer, you will be able to address more complex issues.

## Install the development environment

* Follow the installation instructions in our [wiki](https://github.com/cyberbotics/webots/wiki/).

## Create a Pull Request

1. Fork the repository: https://help.github.com/articles/fork-a-repo
2. Create a branch in your fork: https://help.github.com/articles/creating-and-deleting-branches-within-your-repository
3. Pull the branch as a pull request targeting `cyberbotics:webots@master`: https://help.github.com/articles/creating-a-pull-request-from-a-fork
4. Wait for our unit tests and review of your pull request.

Our git workflow is explained in detail [here](https://github.com/cyberbotics/webots/wiki/Git-workflow/).

## Development Guideline

* Follow our [Coding Style](https://github.com/cyberbotics/webots/wiki/Coding-Style/).
* Avoid comitting files that exist elsewhere. Instead you should link to the source of these files.
* Avoid comitting files that can be re-created from other files using a Makefile, a script or a compiler.

### Step-by-Step Guides

Several step-by-step guides are available on the wiki:

* [Adding a new Robot](https://github.com/cyberbotics/webots/wiki/Adding-a-New-Robot)
* [Adding a new API function](https://github.com/cyberbotics/webots/wiki/Adding-New-Node-and-API-Function#adding-a-new-api-function)
* [Adding a new Webots node](https://github.com/cyberbotics/webots/wiki/Adding-New-Node-and-API-Function#adding-a-new-node)
