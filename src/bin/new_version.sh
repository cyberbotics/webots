#!/bin/bash
# usage ./new_version.sh R2018a 0 R2018a 1
# moves version from R2018a R2018a revision 1

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
new_version_without_revision=$3
year=${new_version:1:4}

echo "Update application and documentation version.."
./new_version_file.sh $old_version $new_version ../webots/core/WbApplicationInfo.cpp
./new_version_file.sh $old_version $new_version ../../resources/version.txt
./new_version_file.sh $old_version $new_version ../packaging/webots_version.txt
./new_version_file.sh $old_version $new_version ../../Contents/Info.plist
./new_version_file.sh "Copyright 1998-[0-9]\+" "Copyright 1998-"$year ../../Contents/Info.plist

# documentation
./new_version_file.sh "major:\\s'"$old_version_without_revision"'" "major: '"$new_version_without_revision"'" ../../docs/js/showdown-extensions.js
./new_version_file.sh "full:\\s'"$old_version"'" "full: '"$new_version"'" ../../docs/js/showdown-extensions.js
./new_version_file.sh "package:\\s'"$old_package"'" "package: '"$new_package"'" ../../docs/js/showdown-extensions.js
./new_version_file.sh "year:\\s[0-9]\+" "year: "$year ../../docs/js/showdown-extensions.js


if [ $new_version_without_revision != $old_version_without_revision ];
then
  echo "Update file headers.."
  ./new_version_file_headers.sh $old_version_without_revision $new_version_without_revision
  ./new_version_file.sh "#VRML_SIM\\s"$old_version_without_revision "#VRML_SIM "$new_version_without_revision ../../docs/reference/proto-example.md

  ./new_version_file.sh $old_version_without_revision $new_version_without_revision ../../resources/projects/worlds/empty.wbt
  ./new_version_file.sh $old_version_without_revision $new_version_without_revision ../../resources/web/README.md
  ./new_version_file.sh $old_version_without_revision $new_version_without_revision ../../resources/web/streaming_viewer/index.html
  ./new_version_file.sh $old_version_without_revision $new_version_without_revision ../../resources/web/streaming_viewer/setup_viewer.js
  ./new_version_file.sh $old_version_without_revision $new_version_without_revision ../../resources/web/templates/x3d_playback.html
  ./new_version_file.sh $old_version_without_revision $new_version_without_revision ../../resources/osm_importer/utils/misc_utils.py

  ./new_version_file.sh "wwi\/$old_version_without_revision\/" "wwi\/$new_version_without_revision\/" ../../docs/dependencies.txt
  ./upload_wwi_files_to_ftp.sh $new_version_without_revision
fi
