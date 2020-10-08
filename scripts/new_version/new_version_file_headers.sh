#!/bin/bash
# update header versions of WBT, PROTO, and WBO files

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
  echo "Usage: $0 [<old_version>] <new_version>" >&2
  echo "Example: $0 R2018a R2018b" >&2
  echo "Example: $0 R2018b" >&2
  exit 1
fi

if [[ -z "${WEBOTS_HOME}" ]]; then
  echo "WEBOTS_HOME is not defined."
  exit 1
fi

CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

if [ "$#" -eq 2 ]; then
  old_sim_header="#VRML_SIM\\s"$1
  old_obj_header="#VRML_OBJ\\s"$1
  new_version=$2
else
  # match any version like V8.5 or V8.3.1
  old_sim_header="#VRML_SIM\\sV[0-9]\+\.[0-9]\+\(\.[0-9]\)\?"
  old_obj_header="#VRML_OBJ\\sV[0-9]\+\.[0-9]\+\(\.[0-9]\)\?"
  new_version=$1
fi

for f in $(find $WEBOTS_HOME -name "*.wbt" -o -name "*.proto")
do
  $CURRENT_DIR/new_version_file.sh $old_sim_header "#VRML_SIM "$new_version $f
done

for f in $(find $WEBOTS_HOME -name "*.wbo")
do
  $CURRENT_DIR/new_version_file.sh $old_obj_header "#VRML_OBJ "$new_version $f
done

old_wbproj_header="Webots\\sProject\\sFile\\sversion\\s"$1
new_version=$2
for f in $(find $WEBOTS_HOME -name "*.wbproj")
do
  $CURRENT_DIR/new_version_file.sh $old_wbproj_header "Webots Project File version "$new_version $f
done
