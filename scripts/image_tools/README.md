# Image tools

## Dependencies

Python 3 or more recent is required to run the image tool scripts.

Install `Pillow` from pip, typically:

```shell
pip install --user --requirement requirements.txt
```

## Scripts

### `clamp_hdr`

Clamp the HDR data to a specific threshold.

> Note: It may be interesting to clamp the highest values of an HDR image when obtaining saturated results in the Webots rendering.
A typical value for the threshold may be 30.

```shell
cd $WEBOTS_HOME

./clamp_hdr.py --help

./clamp_hdr.py --input /Users/$USER/Desktop/entrance_hall.hdr --clamp 30
```

### `convert_hdr_format`

Convert an HDR image to a PNG image.

```shell
cd $WEBOTS_HOME

./convert_hdr_format.py --help

./convert_hdr_format.py --input /Users/$USER/Desktop/entrance_hall.hdr
```

### `downscale_hdr`

Downscale an HDR image to a specified size.

> Note: A smaller image takes less time to be loaded.
A typical irradiance map could be 256x256.

```shell
cd $WEBOTS_HOME

./downscale_hdr.py --help

./downscale_hdr.py --input /Users/$USER/Desktop/entrance_hall.hdr --width 128 --height 128
```

### `equirectangular_to_cubemap`

Convert an image having an equirectangular projection to 6 cubemap images.
The conversion preserves the input image format.

Typical usage:

```shell
cd $WEBOTS_HOME

./equirectangular_to_cubemap.py --help

./equirectangular_to_cubemap.py --input /Users/$USER/Desktop/entrance_hall.png
./equirectangular_to_cubemap.py --input /Users/$USER/Desktop/entrance_hall.hdr --width 1024 --height 1024
```

## Typical Usage

### Convert an HDR Equirectangular Map to a Webots-Compliant Background

```shell
name=background
suffixes=( "back" "bottom" "front" "left" "right" "top" )

./equirectangular_to_cubemap.py --input $name.hdr
for suffix in "${suffixes[@]}"
do
  echo $name\_$suffix.hdr
  ./convert_hdr_format.py --input $name\_$suffix.hdr
  ./downscale_hdr.py --input $name\_$suffix.hdr
done
```
