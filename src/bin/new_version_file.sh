#!/bin/bash

# replace \s by a space
escaped_string=$2
replace_string="${escaped_string//\\s/ }"
echo "s/"$1"/"$replace_string"/g" > sed.cmd
sed -e 's/@/ /g' sed.cmd > sed2.cmd
sed -f sed2.cmd  $3 > tmp.bak
mv tmp.bak $3
rm sed.cmd
rm sed2.cmd
