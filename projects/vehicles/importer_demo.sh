#!/bin/bash

usage() {
  echo
  echo "Usage: ./importer_demo.sh file_base_name"
  echo
  echo "Import an OSM file to Webots:"
  echo "- Generate the ./worlds/$(ARG1).wbt file."
  echo "- Generate the ./worlds/$(ARG1)_net/sumo.net.xml file."
  echo "- Generate random traffic."
  echo
  echo "Notes:"
  echo "- This script works on macOS and linux only."
  echo "- The aim of this script is to show the importation workflow."
  echo "  At each stage, supplementary arguments may be passed or manual edition may be required."
  echo "- This srcipt can override files."
  echo
  echo "Assumptions:"
  echo "- The OSM file is located in the resources directory. e.g. ./resources/osm_files/$(ARG1).osm"
  echo "- Webots and Python 3 are installed as described in the Webots documentation."
  echo "- The WEBOTS_HOME environment variable is defined. e.g. 'export WEBOTS_HOME=/Applications/Webots.app'"
  echo
  echo "Example:"
  echo "$ ./importer_demo.sh village"
}

# variables
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
osm_file_path=$script_dir/resources/osm_files/$1.osm
kernel=$(uname -s)

# arguments check
if [ $# -ne 1 ]; then
  >&2 echo "Error: Wrong arguments size."
	usage
	exit 1
fi
if [ ! -f $osm_file_path ]; then
  >&2 echo "Error: osm file '$osm_file_path' not found."
  usage
	exit 1
fi
if [ -z "$WEBOTS_HOME" ]; then
  echo "WEBOTS_HOME needs to be set."
  usage
  exit 1
fi
if [ -z "$SUMO_HOME" ]; then
  if [ "${kernel:0:6}" == "Darwin" ]; then
    export SUMO_HOME=$WEBOTS_HOME/Contents/projects/default/resources/sumo
  else
    export SUMO_HOME=$WEBOTS_HOME/projects/default/resources/sumo
  fi
  if [ "${kernel:0:5}" == "Linux" ]; then
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$WEBOTS_HOME/lib
  fi
fi
if [ "${kernel:0:5}" == "Linux" ]; then
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$SUMO_HOME/bin
fi

mkdir -p $script_dir/worlds/$1_net

echo
echo "# OSM to Webots..."
echo
set -o xtrace
if [ "$(uname)" == "Darwin" ]; then
  cd $WEBOTS_HOME/Contents/Resources/osm_importer
  rm -f $script_dir/worlds/$1.wbt
  python3 importer.py --input=$osm_file_path --config-file=$WEBOTS_HOME/Contents/Resources/osm_importer/config.ini --output=$script_dir/worlds/$1.wbt
else
  cd $WEBOTS_HOME/resources/osm_importer
  rm -f $script_dir/worlds/$1.wbt
  python3 importer.py --input=$osm_file_path --config-file=$WEBOTS_HOME/resources/osm_importer/config.ini --output=$script_dir/worlds/$1.wbt
fi
set +o xtrace

echo
echo "# Webots to SUMO network..."
echo
set -o xtrace
if [ "$(uname)" == "Darwin" ]; then
  cd $WEBOTS_HOME/Contents/Resources/sumo_exporter
else
  cd $WEBOTS_HOME/resources/sumo_exporter
fi
rm -f $script_dir/worlds/$1_net/sumo.nod.xml $script_dir/worlds/$1_net/sumo.edg.xml $script_dir/worlds/$1_net/sumo.sumocfg $script_dir/worlds/$1_net/sumo.net.xml
python3 exporter.py --input $script_dir/worlds/$1.wbt --output $script_dir/worlds/$1_net
$SUMO_HOME/bin/netconvert --node-files=$script_dir/worlds/$1_net/sumo.nod.xml --edge-files=$script_dir/worlds/$1_net/sumo.edg.xml --output-file=$script_dir/worlds/$1_net/sumo.net.xml
set +o xtrace

echo
echo "# Generate SUMO random traffic..."
echo
set -o xtrace
python3 $SUMO_HOME/tools/randomTrips.py -n $script_dir/worlds/$1_net/sumo.net.xml -o $script_dir/worlds/$1_net/sumo.trip.xml -e 3600 -p 2
$SUMO_HOME/bin/duarouter --trip-files $script_dir/worlds/$1_net/sumo.trip.xml --net-file $script_dir/worlds/$1_net/sumo.net.xml --output-file $script_dir/worlds/$1_net/sumo.rou.xml --ignore-errors true --departlane="random"
set +o xtrace
