#!/bin/bash
# usage ./new_version.sh R2018a 0 R2018a 1 master
# moves version from R2018a R2018a revision 1 on the master branch

if [ "$(uname)" == "Darwin" ]; then
  echo "This script does not work on macOS."
  exit 2
fi

if [[ -z "${WEBOTS_HOME}" ]]; then
  echo "WEBOTS_HOME is not defined."
  exit 1
fi

if [ "$#" -ne 5 ]; then
  echo "Usage: $0 <old_version> <old_revision> <new_version> <new_revision> <master/develop>" >&2
  echo "Example: $0 R2018a 0 R2018a 1 master" >&2
  exit 1
fi

if [ "$2" -eq 0 ]; then
  old_version=$1
  old_package=$1
else
  old_version=$1"\srevision\s"$2
  old_package=$1"-rev"$2
fi

if [ "$4" -eq 0 ]; then
  new_version=$3
  new_package=$3
else
  new_version=$3"\srevision\s"$4
  new_package=$3"-rev"$4
fi

branch=$5
if [ "$branch" -eq "master" ]; then
  branch_search="https://cdn.jsdelivr.net/gh/cyberbotics/webots@develop/"
  branch_replace="https://cdn.jsdelivr.net/gh/cyberbotics/webots@master/"
elif
  branch_search="https://cdn.jsdelivr.net/gh/cyberbotics/webots@master/"
  branch_replace="https://cdn.jsdelivr.net/gh/cyberbotics/webots@develop/"
fi

old_version_without_revision=$1
old_version_year=${old_version:1:4}
old_version_letter=${old_version:5:1}

new_version_without_revision=$3
new_version_year=${new_version:1:4}
new_version_letter=${new_version:5:1}

CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Update application and documentation version..."
$CURRENT_DIR/new_version_file.sh $old_version $new_version $WEBOTS_HOME/src/webots/core/WbApplicationInfo.cpp
$CURRENT_DIR/new_version_file.sh $old_version $new_version $WEBOTS_HOME/resources/version.txt
$CURRENT_DIR/new_version_file.sh $old_version $new_version $WEBOTS_HOME/scripts/packaging/webots_version.txt
$CURRENT_DIR/new_version_file.sh $old_version $new_version $WEBOTS_HOME/Contents/Info.plist
if [ $new_version_year -ne $old_version_year ]; then
  $CURRENT_DIR/new_version_file.sh "Copyright 1998-[0-9]\+" "Copyright 1998-"$new_version_year $WEBOTS_HOME/Contents/Info.plist
fi
# documentation
if [ $new_version_without_revision != $old_version_without_revision ]; then
  $CURRENT_DIR/new_version_file.sh "major:\\s'.*'" "major: '"$new_version_without_revision"'" $WEBOTS_HOME/docs/js/showdown-extensions.js
fi
$CURRENT_DIR/new_version_file.sh "full:\\s'.*'" "full: '"$new_version"'" $WEBOTS_HOME/docs/js/showdown-extensions.js
$CURRENT_DIR/new_version_file.sh "package:\\s'.*'" "package: '"$new_package"'" $WEBOTS_HOME/docs/js/showdown-extensions.js
if [ $new_version_year -ne $old_version_year ]; then
  $CURRENT_DIR/new_version_file.sh "year:\\s[0-9]\+" "year: "$new_version_year $WEBOTS_HOME/docs/js/showdown-extensions.js
fi

if [ $new_version_without_revision != $old_version_without_revision ];
then
  echo "Update file headers.."
  $CURRENT_DIR/new_version_file_headers.sh $old_version_without_revision $new_version_without_revision
  $CURRENT_DIR/new_version_file.sh "#VRML_SIM\\s"$old_version_without_revision "#VRML_SIM "$new_version_without_revision $WEBOTS_HOME/docs/reference/proto-example.md
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/guide/object-shapes.md
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/guide/object-living-room-furniture.md
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/guide/object-school-furniture.md
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/guide/object-television.md
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/guide/object-balls.md
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/guide/object-plants.md
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/guide/object-bedroom.md
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/guide/object-traffic.md
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/guide/object-drinks.md
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/guide/object-toys.md
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/guide/object-advertising-board.md
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/guide/object-solids.md
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/guide/object-kitchen.md
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/guide/object-rocks.md
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/guide/object-trees.md
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/guide/object-paintings.md
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/guide/object-road.md
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/guide/object-lights.md
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/guide/object-street-furniture.md
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/reference/javascript-procedural-proto.md

  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/resources/web/streaming_viewer/index.html
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/resources/web/streaming_viewer/setup_viewer.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/resources/web/templates/x3d_playback.html
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/resources/web/wwi/AnimationSlider.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/resources/web/wwi/WebotsView.js
  $CURRENT_DIR/new_version_file.sh "#VRML_SIM\\s$old_version_without_revision" "#VRML_SIM $new_version_without_revision" $WEBOTS_HOME/resources/osm_importer/utils/misc_utils.py
  $CURRENT_DIR/new_version_file.sh $branch_search $branch_replace $WEBOTS_HOME/resources/osm_importer/utils/misc_utils.py
  $CURRENT_DIR/new_version_file.sh $branch_search $branch_replace $WEBOTS_HOME/resources/osm_importer/config.ini

  $CURRENT_DIR/new_version_file.sh "wwi\/$old_version_without_revision\/" "wwi\/$new_version_without_revision\/" $WEBOTS_HOME/docs/dependencies.txt
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/js/webots_documentation_loader.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/css/webots-doc.css
  $CURRENT_DIR/new_version_file.sh $branch_search $branch_replace $WEBOTS_HOME/resources/osm_importer/webots_objects/barrier.py
  $CURRENT_DIR/new_version_file.sh $branch_search $branch_replace $WEBOTS_HOME/resources/osm_importer/webots_objects/road.py
  $CURRENT_DIR/new_version_file.sh $branch_search $branch_replace $WEBOTS_HOME/resources/osm_importer/elevation.py
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/resources/web/server/simulation_server.py
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/resources/web/server/session_server.py

  $CURRENT_DIR/new_version_file.sh $branch_search $branch_replace $WEBOTS_HOME/projects/humans/c3d/plugins/robot_windows/c3d_viewer_window/c3d_viewer_window.html
  $CURRENT_DIR/new_version_file.sh $branch_search $branch_replace $WEBOTS_HOME/projects/vehicles/plugins/robot_windows/automobile/automobile.html
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/projects/robots/gctronic/e-puck/plugins/robot_windows/e-puck/e-puck.html
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/tests/api/controllers/supervisor_animation/animation.html

  echo "wwi resources on the cyberbotics FTP should be updated."
fi
