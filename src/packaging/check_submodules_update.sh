#!/bin/bash -

# Description:
# - Check that the submodules are up-to-date (=last commit of their master branch)
#   and display a warning if it's not the case.

# Arguments:
# - "--verbose": display on stdout more information
verbose=false
while test $# -gt 0
do
  case "$1" in
    --verbose) verbose=true
      ;;
  esac
  shift
done

# ANSI escape characters
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get paths
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
webots_dir=$script_dir/../..

# Make sure that the submodules are initialized
cd $webots_dir
git submodule init
git submodule update

# foreach submodule
git submodule | awk '{print $2}' | while read module ; do
  if $verbose ; then echo === $module === ; fi
  # Enter in the module
  cd $module
  # Fetch in order to be sure to have the latest information
  # about the remote repository
  git fetch origin --quiet
  # Get the current and origin/master latest commits
  current_commit=$(git show | head -1 | awk '{print $2}')
  if $verbose ; then echo Current commit: $current_commit ; fi
  master_commit=$(git show origin/master | head -1 | awk '{print $2}')
  if $verbose ; then echo Master commit: $master_commit ; fi
  # Compare them
  if [ "$current_commit" != "$master_commit" ]; then
    >&2 echo -e "# ${YELLOW}Submodule '$module':${NC}"
    >&2 echo -e "# ${YELLOW}- 'Current' and 'origin/master' commits IDs mismatch.${NC}"
    >&2 echo -e "# ${YELLOW}  - 'Current' commit id: $current_commit${NC}"
    >&2 echo -e "# ${YELLOW}  - 'origin/master' commit id: $master_commit${NC}"
  fi
  # Restore the initial directory
  cd $webots_dir
done
