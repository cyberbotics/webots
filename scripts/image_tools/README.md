# Image tools

## `equirectangular_to_cubemap`

Convert an image having an equirectangular projection to 6 cubemap images.

Typical usage:

```
cd $WEBOTS_HOME

python scripts/image_tools/equirectangular_to_cubemap.py --help

python scripts/image_tools/equirectangular_to_cubemap.py --input projects/default/worlds/textures/cubic/entrance_hall.hdr --width 1042 --height 1042
python scripts/image_tools/equirectangular_to_cubemap.py --input /home/user/Desktop/entrance_hall.png
```
