#!/bin/bash
# usage ./new_version.sh R2018a 0 R2018a 1
# moves version from R2018a R2018a revision 1

if [ "$(uname)" == "Darwin" ]; then
  echo "This script does not work on macOS."
  exit 2
fi

if [[ -z "${WEBOTS_HOME}" ]]; then
  echo "WEBOTS_HOME is not defined."
  exit 1
fi

if [ "$#" -ne 4 ]; then
  echo "Usage: $0 <old_version> <old_revision> <new_version> <new_revision>" >&2
  echo "Example: $0 R2018a 0 R2018a 1" >&2
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

old_version_without_revision=$1
old_version_year=${old_version:1:4}
old_version_letter=${old_version:5:1}

new_version_without_revision=$3
new_version_year=${new_version:1:4}
new_version_letter=${new_version:5:1}

CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Update application and documentation version..."
$CURRENT_DIR/new_version_file.sh $old_version $new_version $WEBOTS_HOME/src/webots/core/WbApplicationInfo.cpp
$CURRENT_DIR/new_version_file.sh $old_version $new_version $WEBOTS_HOME/src/webots/gui/WbUpdatedDialog.cpp
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

  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/resources/web/streaming_viewer/index.html
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/resources/web/streaming_viewer/setup_viewer.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/resources/web/templates/x3d_playback.html
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/resources/web/wwi/AnimationSlider.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/resources/web/wwi/NodeSelectorWindow.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/resources/web/wwi/WebotsView.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/resources/web/wwi/Progress.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/resources/web/wwi/proto_viewer.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/resources/web/wwi/viewer.js

  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/projects/vehicles/plugins/robot_windows/automobile/automobile.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/projects/samples/robotbenchmark/pick_and_place/plugins/robot_windows/pick_and_place/pick_and_place.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/projects/samples/robotbenchmark/humanoid_sprint/plugins/robot_windows/humanoid_sprint/humanoid_sprint.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/projects/samples/robotbenchmark/humanoid_marathon/plugins/robot_windows/humanoid_marathon/humanoid_marathon.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/projects/samples/robotbenchmark/wall_following/plugins/robot_windows/wall_following/wall_following.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/projects/samples/robotbenchmark/highway_driving/plugins/robot_windows/highway_driving/highway_driving.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/projects/samples/robotbenchmark/pit_escape/plugins/robot_windows/pit_escape/pit_escape.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/projects/samples/robotbenchmark/obstacle_avoidance/plugins/robot_windows/obstacle_avoidance/obstacle_avoidance.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/projects/samples/robotbenchmark/inverted_pendulum/plugins/robot_windows/inverted_pendulum/inverted_pendulum.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/projects/samples/robotbenchmark/square_path/plugins/robot_windows/square_path/square_path.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/projects/samples/robotbenchmark/robot_programming/plugins/robot_windows/robot_programming/robot_programming.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/projects/samples/robotbenchmark/maze_runner/plugins/robot_windows/maze_runner/maze_runner.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/projects/samples/robotbenchmark/visual_tracking/plugins/robot_windows/pan_tilt_camera_view/pan_tilt_camera_view.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/projects/samples/robotbenchmark/visual_tracking/plugins/robot_windows/visual_tracking/visual_tracking.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/projects/samples/howto/custom_robot_window/plugins/robot_windows/custom_robot_window/custom_robot_window.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/projects/samples/howto/custom_robot_window_simple/plugins/robot_windows/custom_robot_window_simple/custom_robot_window_simple.html
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/projects/humans/c3d/plugins/robot_windows/c3d_viewer_window/c3d_viewer_window.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/projects/robots/mobsya/thymio/plugins/robot_windows/thymio2/thymio2.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/projects/robots/gctronic/e-puck/plugins/robot_windows/e-puck/e-puck.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/tests/api/plugins/robot_windows/info/info.js
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/resources/projects/plugins/robot_windows/generic/generic.js

  # $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/projects/samples/howto/docker/README.md
  # $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/projects/samples/howto/docker/launcher.py

  # $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/resources/osm_importer/utils/misc_utils.py
  # $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/resources/osm_importer/config.ini
  # $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/resources/osm_importer/webots_objects/barrier.py
  # $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/resources/osm_importer/webots_objects/road.py
  # $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/resources/osm_importer/elevation.py

  $CURRENT_DIR/new_version_file.sh "wwi\/$old_version_without_revision\/" "wwi\/$new_version_without_revision\/" $WEBOTS_HOME/docs/dependencies.txt
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/index.template.html
  $CURRENT_DIR/new_version_file.sh $old_version_without_revision $new_version_without_revision $WEBOTS_HOME/docs/css/webots-doc.css

  echo "wwi resources on the cyberbotics FTP should be updated."

  echo "webots_ros assets and tests/cache/worlds/backwards_compatibility.wbt should not be modified by this script"
fi
