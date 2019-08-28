#!/usr/bin/python

"""Convert a equirectanglar HDR image to cubemap HDR images."""

import optparse
from hdr import HDR

optParser = optparse.OptionParser(usage="usage: %prog --input=image.hdr [options]")
optParser.add_option("--input", dest="input", default="image.hdr", help="specifies the input equirectangle image path")
options, args = optParser.parse_args()

hdr = HDR.load_from_file(options.input)
hdr.save(options.input.replace('.hdr', '.test.hdr'))

# Verification script
# from PIL import Image
# im = hdr.to_pil()
# im.show()
