from pyclibrary import CParser

parser = CParser(['../include/wren/camera.h'])

print(CParser.parse_defs(parser))
