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

python clamp_hdr.py --help

python clamp_hdr.py --input /Users/$USER/Desktop/entrance_hall.hdr --clamp 30
```

### `convert_hdr_format`

Convert an HDR image to a PNG image.

```shell
cd $WEBOTS_HOME

python convert_hdr_format.py --help

python convert_hdr_format.py --input /Users/$USER/Desktop/entrance_hall.hdr
```

### `downscale_hdr`

Downscale an HDR image to a specified size.

> Note: A smaller image takes less time to be loaded.
A typical irradiance map could be 256x256.

```shell
cd $WEBOTS_HOME

python downscale_hdr.py --help

python downscale_hdr.py --input /Users/$USER/Desktop/entrance_hall.hdr --width 128 --height 128
```

### `equirectangular_to_cubemap`

Convert an image having an equirectangular projection to 6 cubemap images.
The conversion preserves the input image format.

Typical usage:

```shell
cd $WEBOTS_HOME

python equirectangular_to_cubemap.py --help

python equirectangular_to_cubemap.py --input /Users/$USER/Desktop/entrance_hall.png
python equirectangular_to_cubemap.py --input /Users/$USER/Desktop/entrance_hall.hdr --width 1042 --height 1042
```

## Typical Usage

### Convert an HDR Equirectangular Map to a Webots-Compliant Background

```shell
name=background
suffixes=( "back" "bottom" "front" "left" "right" "top" )

python equirectangular_to_cubemap.py --input $name.hdr
for suffix in "${suffixes[@]}"
do
  echo $name\_$suffix.hdr
  python convert_hdr_format.py --input $name\_$suffix.hdr
  python downscale_hdr.py --input $name\_$suffix.hdr
done
```
