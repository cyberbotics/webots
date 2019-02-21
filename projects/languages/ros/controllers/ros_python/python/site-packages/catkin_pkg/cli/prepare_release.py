from __future__ import print_function

import argparse
import os
import re
import subprocess
import sys

from catkin_pkg import metapackage
from catkin_pkg.changelog import CHANGELOG_FILENAME, get_changelog_from_path
from catkin_pkg.package import InvalidPackage, PACKAGE_MANIFEST_FILENAME
from catkin_pkg.package_version import bump_version
from catkin_pkg.package_version import get_forthcoming_label, update_changelog_sections, update_versions
from catkin_pkg.packages import find_packages, verify_equal_package_versions
from catkin_pkg.terminal_color import disable_ANSI_colors, fmt
from catkin_pkg.workspace_vcs import get_repository_type, vcs_remotes

try:
    raw_input
except NameError:
    raw_input = input  # noqa: A001


def has_changes(base_path, path, vcs_type):
    cmd = [_find_executable(vcs_type), 'diff', path]
    try:
        output = subprocess.check_output(cmd, cwd=base_path)
    except subprocess.CalledProcessError as e:
        raise RuntimeError(fmt("@{rf}Failed to check if '@{boldon}%s@{boldoff}' has modifications: %s" % (path, str(e))))
    return output.decode('utf-8').rstrip() != ''


def prompt_continue(msg, default):
    """Prompt the user for continuation."""
    if default:
        msg += fmt(' @{yf}[Y/n]@{reset}?')
    else:
        msg += fmt(' @{yf}[y/N]@{reset}?')

    while True:
        _flush_stdin()
        try:
            response = raw_input(msg)
        except EOFError:
            response = ''
        if not response:
            response = 'y' if default else 'n'
        else:
            response = response.lower()

        if response in ['y', 'n']:
            return response == 'y'

        print(
            fmt(
                "@{yf}Response '@{boldon}%s@{boldoff}' was not recognized, please use one of the following options: %s" %
                (response, ', '.join([('@{boldon}%s@{boldoff}' % x) for x in ['y', 'Y', 'n', 'N']]))
            ), file=sys.stderr)


def _flush_stdin():
    try:
        from termios import tcflush, TCIFLUSH
        tcflush(sys.stdin, TCIFLUSH)
    except ImportError:
        # fallback if not supported on some platforms
        pass


def get_git_branch(base_path):
    cmd_branch = [_find_executable('git'), 'rev-parse', '--abbrev-ref', 'HEAD']
    try:
        branch = subprocess.check_output(cmd_branch, cwd=base_path)
    except subprocess.CalledProcessError as e:
        raise RuntimeError(fmt('@{rf}Could not determine git branch: %s' % str(e)))
    return branch.decode('utf-8').rstrip()


def get_git_remote(base_path):
    branch = get_git_branch(base_path)

    cmd_remote = [_find_executable('git'), 'config', '--get', 'branch.%s.remote' % branch]
    try:
        remote = subprocess.check_output(cmd_remote, cwd=base_path)
    except subprocess.CalledProcessError as e:
        msg = 'Could not determine git remote: %s' % str(e)
        msg += "\n\nMay be the branch '%s' is not tracking a remote branch?" % branch
        raise RuntimeError(fmt('@{rf}%s' % msg))
    return remote.decode('utf-8').rstrip()


def try_repo_push(base_path, vcs_type):
    if vcs_type in ['git']:
        print('Trying to push to remote repository (dry run)...')
        cmd = [_find_executable(vcs_type), 'push']
        if vcs_type == 'git':
            cmd.extend(['-n'] + [get_git_remote(base_path), get_git_branch(base_path)])
        try:
            subprocess.check_call(cmd, cwd=base_path)
        except (subprocess.CalledProcessError, RuntimeError) as e:
            raise RuntimeError(fmt('@{rf}Failed to dry push to repository: %s' % str(e)))


def check_clean_working_copy(base_path, vcs_type):
    if vcs_type in ['bzr', 'hg', 'svn']:
        cmd = [_find_executable(vcs_type), 'status']
    elif vcs_type in ['git']:
        cmd = [_find_executable(vcs_type), 'status', '-s', '-u']
    else:
        assert False, 'Unknown vcs type: %s' % vcs_type
    try:
        output = subprocess.check_output(cmd, cwd=base_path)
    except subprocess.CalledProcessError as e:
        raise RuntimeError(fmt('@{rf}Failed to check working copy state: %s' % str(e)))
    output = output.decode('utf-8').rstrip()
    if output != '':
        print(output)
        return False
    return True


def commit_files(base_path, vcs_type, packages, packages_with_changelogs, message, dry_run=False):
    cmd = [_find_executable(vcs_type), 'commit', '-m', message]
    cmd += [os.path.join(p, PACKAGE_MANIFEST_FILENAME) for p in packages.keys()]
    cmd += [path for path, _, _ in packages_with_changelogs.values()]
    if not dry_run:
        try:
            subprocess.check_call(cmd, cwd=base_path)
        except subprocess.CalledProcessError as e:
            raise RuntimeError(fmt('@{rf}Failed to commit package.xml files: %s' % str(e)))
    return cmd


def tag_repository(base_path, vcs_type, tag_name, has_tag_prefix, dry_run=False):
    if vcs_type in ['bzr', 'git', 'hg']:
        cmd = [_find_executable(vcs_type), 'tag', tag_name]
    elif vcs_type == 'svn':
        svn_url = vcs_remotes(base_path, 'svn')[5:]
        if os.path.basename(svn_url) == 'trunk':
            # tag "trunk"
            base_url = os.path.dirname(svn_url)
        elif os.path.basename(os.path.dirname(svn_url)) == 'branches':
            # tag a direct subfolder of "branches"
            base_url = os.path.dirname(os.path.dirname(svn_url))
        elif svn_url.rfind('/trunk/') != -1:
            # tag any subfolder of trunk but require a tag prefix
            if not has_tag_prefix:
                raise RuntimeError(fmt('@{rf}When tagging a subfolder you must use --tag-prefix to make your tag name unique'))
            base_url = svn_url[:svn_url.rfind('/trunk/')]
        elif svn_url.rfind('/branches/') != -1:
            # tag any subfolder of trunk but require a tag prefix
            if not has_tag_prefix:
                raise RuntimeError(fmt('@{rf}When tagging a subfolder you must use --tag-prefix to make your tag name unique'))
            base_url = svn_url[:svn_url.rfind('/branches/')]
        else:
            raise RuntimeError(fmt("@{rf}Could not determine base URL of SVN repository '%s'" % svn_url))
        tag_url = '%s/tags/%s' % (base_url, tag_name)
        cmd = ['svn', 'cp', '-m', '"tagging %s"' % tag_name, svn_url, tag_url]
    else:
        assert False, 'Unknown vcs type: %s' % vcs_type
    if not dry_run:
        try:
            subprocess.check_call(cmd, cwd=base_path)
        except subprocess.CalledProcessError as e:
            raise RuntimeError(fmt('@{rf}Failed to tag repository: %s' % str(e)))
    return cmd


def push_changes(base_path, vcs_type, tag_name, dry_run=False):
    commands = []

    # push changes to the repository
    cmd = [_find_executable(vcs_type), 'push']
    if vcs_type == 'git':
        cmd.extend([get_git_remote(base_path), get_git_branch(base_path)])
    commands.append(cmd)
    if not dry_run:
        try:
            subprocess.check_call(cmd, cwd=base_path)
        except subprocess.CalledProcessError as e:
            raise RuntimeError(fmt('@{rf}Failed to push changes to the repository: %s\n\nYou need to manually push the changes/tag to the repository.' % str(e)))

    # push tags to the repository
    if vcs_type in ['git']:
        cmd = [_find_executable(vcs_type), 'push', get_git_remote(base_path), tag_name]
        commands.append(cmd)
        if not dry_run:
            try:
                subprocess.check_call(cmd, cwd=base_path)
            except subprocess.CalledProcessError as e:
                raise RuntimeError(fmt('@{rf}Failed to push tag to the repository: %s\n\nYou need to manually push the new tag to the repository.' % str(e)))

    return commands


def _find_executable(vcs_type):
    for path in os.getenv('PATH').split(os.path.pathsep):
        file_path = os.path.join(path, vcs_type)
        if os.path.isfile(file_path):
            return file_path
    raise RuntimeError(fmt('@{rf}Could not find vcs binary: %s' % vcs_type))


def main():
    try:
        _main()
    except RuntimeError as e:
        print(e, file=sys.stderr)
        sys.exit(1)


def _main():
    parser = argparse.ArgumentParser(
        description='Runs the commands to bump the version number, commit the modified %s files and create a tag in the repository.' % PACKAGE_MANIFEST_FILENAME)
    parser.add_argument('--bump', choices=('major', 'minor', 'patch'), default='patch', help='Which part of the version number to bump? (default: %(default)s)')
    parser.add_argument('--version', help='Specify a specific version to use')
    parser.add_argument('--no-color', action='store_true', default=False, help='Disables colored output')
    parser.add_argument('--no-push', action='store_true', default=False, help='Disables pushing to remote repository')
    parser.add_argument('-t', '--tag-prefix', default='', help='Add this prefix to the created release tag')
    parser.add_argument('-y', '--non-interactive', action='store_true', default=False, help="Run without user interaction, confirming all questions with 'yes'")
    args = parser.parse_args()

    if args.version and not re.match('^(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)$', args.version):
        parser.error('The passed version must follow the conventions (positive integers x.y.z with no leading zeros)')

    if args.tag_prefix and ' ' in args.tag_prefix:
        parser.error('The tag prefix must not contain spaces')

    # force --no-color if stdout is non-interactive
    if not sys.stdout.isatty():
        args.no_color = True
    # disable colors if asked
    if args.no_color:
        disable_ANSI_colors()

    base_path = '.'

    print(fmt('@{gf}Prepare the source repository for a release.'))

    # determine repository type
    vcs_type = get_repository_type(base_path)
    if vcs_type is None:
        raise RuntimeError(fmt("@{rf}Could not determine repository type of @{boldon}'%s'@{boldoff}" % base_path))
    print(fmt('Repository type: @{boldon}%s@{boldoff}' % vcs_type))

    # find packages
    try:
        packages = find_packages(base_path)
    except InvalidPackage as e:
        raise RuntimeError(fmt("@{rf}Invalid package at path @{boldon}'%s'@{boldoff}:\n  %s" % (os.path.abspath(base_path), str(e))))
    if not packages:
        raise RuntimeError(fmt('@{rf}No packages found'))
    print('Found packages: %s' % ', '.join([fmt('@{bf}@{boldon}%s@{boldoff}@{reset}' % p.name) for p in packages.values()]))

    # complain about packages with non-catkin build_type as they might require additional steps before being released
    # complain about packages with upper case character since they won't be releasable with bloom
    non_catkin_pkg_names = []
    invalid_pkg_names = []
    for package in packages.values():
        build_types = [export.content for export in package.exports if export.tagname == 'build_type']
        build_type = build_types[0] if build_types else 'catkin'
        if build_type != 'catkin':
            non_catkin_pkg_names.append(package.name)
        if package.name != package.name.lower():
            invalid_pkg_names.append(package.name)
    if non_catkin_pkg_names:
        print(
            fmt(
                "@{yf}Warning: the following package are not of build_type catkin and may require manual steps to release': %s" %
                ', '.join([('@{boldon}%s@{boldoff}' % p) for p in sorted(non_catkin_pkg_names)])
            ), file=sys.stderr)
        if not args.non_interactive and not prompt_continue('Continue anyway', default=False):
            raise RuntimeError(fmt('@{rf}Aborted release, verify that non-catkin packages are ready to be released or release manually.'))
    if invalid_pkg_names:
        print(
            fmt(
                "@{yf}Warning: the following package names contain upper case characters which violate both ROS and Debian naming conventions': %s" %
                ', '.join([('@{boldon}%s@{boldoff}' % p) for p in sorted(invalid_pkg_names)])
            ), file=sys.stderr)
        if not args.non_interactive and not prompt_continue('Continue anyway', default=False):
            raise RuntimeError(fmt('@{rf}Aborted release, fix the names of the packages.'))

    local_modifications = []
    for pkg_path, package in packages.items():
        # verify that the package.xml files don't have modifications pending
        package_xml_path = os.path.join(pkg_path, PACKAGE_MANIFEST_FILENAME)
        if has_changes(base_path, package_xml_path, vcs_type):
            local_modifications.append(package_xml_path)
        # verify that metapackages are valid
        if package.is_metapackage():
            try:
                metapackage.validate_metapackage(pkg_path, package)
            except metapackage.InvalidMetapackage as e:
                raise RuntimeError(fmt(
                    "@{rf}Invalid metapackage at path '@{boldon}%s@{boldoff}':\n  %s\n\nSee requirements for metapackages: %s" %
                    (os.path.abspath(pkg_path), str(e), metapackage.DEFINITION_URL)))

    # fetch current version and verify that all packages have same version number
    old_version = verify_equal_package_versions(packages.values())
    if args.version:
        new_version = args.version
    else:
        new_version = bump_version(old_version, args.bump)
    tag_name = args.tag_prefix + new_version

    if (
        not args.non_interactive and
        not prompt_continue(
            fmt(
                "Prepare release of version '@{bf}@{boldon}%s@{boldoff}@{reset}'%s" %
                (new_version, " (tagged as '@{bf}@{boldon}%s@{boldoff}@{reset}')" % tag_name if args.tag_prefix else '')
            ), default=True)
    ):
        raise RuntimeError(fmt("@{rf}Aborted release, use option '--bump' to release a different version and/or '--tag-prefix' to add a prefix to the tag name."))

    # check for changelog entries
    missing_changelogs = []
    missing_changelogs_but_forthcoming = {}
    for pkg_path, package in packages.items():
        changelog_path = os.path.join(pkg_path, CHANGELOG_FILENAME)
        if not os.path.exists(changelog_path):
            missing_changelogs.append(package.name)
            continue
        # verify that the changelog files don't have modifications pending
        if has_changes(base_path, changelog_path, vcs_type):
            local_modifications.append(changelog_path)
        changelog = get_changelog_from_path(changelog_path, package.name)
        try:
            changelog.get_content_of_version(new_version)
        except KeyError:
            # check that forthcoming section exists
            forthcoming_label = get_forthcoming_label(changelog.rst)
            if forthcoming_label:
                missing_changelogs_but_forthcoming[package.name] = (changelog_path, changelog, forthcoming_label)
            else:
                missing_changelogs.append(package.name)

    if local_modifications:
        raise RuntimeError(fmt('@{rf}The following files have modifications, please commit/revert them before:' + ''.join([('\n- @{boldon}%s@{boldoff}' % path) for path in local_modifications])))

    if missing_changelogs:
        print(
            fmt(
                "@{yf}Warning: the following packages do not have a changelog file or entry for version '@{boldon}%s@{boldoff}': %s" %
                (new_version, ', '.join([('@{boldon}%s@{boldoff}' % p) for p in sorted(missing_changelogs)]))
            ), file=sys.stderr)
        if not args.non_interactive and not prompt_continue('Continue without changelogs', default=False):
            raise RuntimeError(fmt("@{rf}Aborted release, populate the changelog with '@{boldon}catkin_generate_changelog@{boldoff}' and review / clean up the content."))

    # verify that repository is pushable (if the vcs supports dry run of push)
    if not args.no_push:
        try_repo_push(base_path, vcs_type)

    # check for staged changes and modified and untracked files
    print(fmt('@{gf}Checking if working copy is clean (no staged changes, no modified files, no untracked files)...'))
    is_clean = check_clean_working_copy(base_path, vcs_type)
    if not is_clean:
        print(fmt('@{yf}Warning: the working copy contains other changes. Consider reverting/committing/stashing them before preparing a release.'), file=sys.stderr)
        if not args.non_interactive and not prompt_continue('Continue anyway', default=False):
            raise RuntimeError(fmt('@{rf}Aborted release, clean the working copy before trying again.'))

    # for svn verify that we know how to tag that repository
    if vcs_type in ['svn']:
        tag_svn_cmd = tag_repository(base_path, vcs_type, tag_name, args.tag_prefix != '', dry_run=True)

    # tag forthcoming changelog sections
    update_changelog_sections(missing_changelogs_but_forthcoming, new_version)
    print(fmt(
        "@{gf}Rename the forthcoming section@{reset} of the following packages to version '@{bf}@{boldon}%s@{boldoff}@{reset}': %s" %
        (new_version, ', '.join([('@{boldon}%s@{boldoff}' % p) for p in sorted(missing_changelogs_but_forthcoming.keys())]))))

    # bump version number
    update_versions(packages.keys(), new_version)
    print(fmt("@{gf}Bump version@{reset} of all packages from '@{bf}%s@{reset}' to '@{bf}@{boldon}%s@{boldoff}@{reset}'" % (old_version, new_version)))

    pushed = None
    if vcs_type in ['svn']:
        # for svn everything affects the remote repository immediately
        commands = []
        commands.append(commit_files(base_path, vcs_type, packages, missing_changelogs_but_forthcoming, tag_name, dry_run=True))
        commands.append(tag_svn_cmd)
        if not args.no_push:
            print(fmt('@{gf}The following commands will be executed to commit the changes and tag the new version:'))
        else:
            print(fmt('@{gf}You can use the following commands to manually commit the changes and tag the new version:'))
        for cmd in commands:
            print(fmt('  @{bf}@{boldon}%s@{boldoff}' % ' '.join(cmd)))

        if not args.no_push:
            if not args.non_interactive:
                # confirm before modifying repository
                if not prompt_continue('Execute commands which will modify the repository', default=True):
                    pushed = False
            if pushed is None:
                commit_files(base_path, vcs_type, packages, missing_changelogs_but_forthcoming, tag_name)
                tag_repository(base_path, vcs_type, tag_name, args.tag_prefix != '')
                pushed = True

    else:
        # for other vcs types the changes are first done locally
        print(fmt('@{gf}Committing the package.xml files...'))
        commit_files(base_path, vcs_type, packages, missing_changelogs_but_forthcoming, tag_name)

        print(fmt("@{gf}Creating tag '@{boldon}%s@{boldoff}'..." % (tag_name)))
        tag_repository(base_path, vcs_type, tag_name, args.tag_prefix != '')

        try:
            commands = push_changes(base_path, vcs_type, tag_name, dry_run=True)
        except RuntimeError:
            print(fmt('@{yf}Warning: could not determine commands to push the changes and tag to the remote repository. Do you have a remote configured for the current branch?'))
        else:
            if not args.no_push:
                print(fmt('@{gf}The following commands will be executed to push the changes and tag to the remote repository:'))
            else:
                print(fmt('@{gf}You can use the following commands to manually push the changes to the remote repository:'))
            for cmd in commands:
                print(fmt('  @{bf}@{boldon}%s@{boldoff}' % ' '.join(cmd)))

            if not args.no_push:
                if not args.non_interactive:
                    # confirm commands to push to remote repository
                    if not prompt_continue('Execute commands to push the local commits and tags to the remote repository', default=True):
                        pushed = False
                if pushed is None:
                    push_changes(base_path, vcs_type, tag_name)
                    pushed = True

    if pushed:
        print(fmt("@{gf}The source repository has been released successfully. The next step will be '@{boldon}bloom-release@{boldoff}'."))
    else:
        msg = 'The release of the source repository has been prepared successfully but the changes have not been pushed yet. ' \
            "After pushing the changes manually the next step will be '@{boldon}bloom-release@{boldoff}'."
        if args.no_push or pushed is False:
            print(fmt('@{yf}%s' % msg))
        else:
            raise RuntimeError(fmt('@{rf}%s' % msg))
