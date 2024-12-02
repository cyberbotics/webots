#!/bin/sh

#
# Description: Script for superimposing a label ("Webots TM") at the bottom left of a PNG image
# Author: fabien.rohrer@cyberbotics.com
# Required: ImageMagick (http://www.imagemagick.org) AND ttf-freefont
# Usage: ./superimpose_webots_logo.sh yourImage.png
#

if [ $# -ne 1 ]; then
  echo 1>&2 Usage: $0 image_filename
  exit 127
fi

width=`identify -format %w $1`
height=`identify -format %h $1`
env LC_CTYPE=en_US.utf8 printf "Webots\u2122" | \
  convert -font /usr/share/fonts/truetype/freefont/FreeSansBold.ttf -background none -gravity South -fill '#00000060' \
  -strokewidth 2 -stroke '#ffffff60' -size ${width}x${height} -resize 25x25% \
  label:@- +size $1 +swap -gravity SouthWest -geometry +5+5 -composite $1
