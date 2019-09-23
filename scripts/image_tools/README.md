# Image tools

## Dependencies

Install `Pillow` from pip, typically:

```
pip install Pillow
```

## `clamp_hdr`

Clamp the HDR data to a specific threshold.

```
cd $WEBOTS_HOME

python scripts/image_tools/clamp_hdr.py --help

python scripts/image_tools/clamp_hdr.py --input projects/default/worlds/textures/cubic/entrance_hall.hdr --clamp 30
```

## `convert_hdr_format`

Convert an HDR image to a PNG image.

```
cd $WEBOTS_HOME

python scripts/image_tools/convert_hdr_format.py --help

python scripts/image_tools/convert_hdr_format.py --input projects/default/worlds/textures/cubic/entrance_hall.hdr
```

## `downscale_hdr`

Downscale an HDR image to a specified size.

```
cd $WEBOTS_HOME

python scripts/image_tools/downscale_hdr.py --help

python scripts/image_tools/downscale_hdr.py --input projects/default/worlds/textures/cubic/entrance_hall.hdr --clamp 30
```

## `equirectangular_to_cubemap`

Convert an image having an equirectangular projection to 6 cubemap images.

Typical usage:

```
cd $WEBOTS_HOME

python scripts/image_tools/equirectangular_to_cubemap.py --help

python scripts/image_tools/equirectangular_to_cubemap.py --input projects/default/worlds/textures/cubic/entrance_hall.hdr --width 1042 --height 1042
python scripts/image_tools/equirectangular_to_cubemap.py --input /home/user/Desktop/entrance_hall.png
```

## `hdr_to_irradiance_maps`

Convert an HDR image to irradiance maps.

```
cd $WEBOTS_HOME

python scripts/image_tools/hdr_to_irradiance_maps.py --help

python scripts/image_tools/hdr_to_irradiance_maps.py --input projects/default/worlds/textures/cubic/entrance_hall.hdr
```
