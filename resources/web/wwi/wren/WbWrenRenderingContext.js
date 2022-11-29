export default class WbWrenRenderingContext {}

WbWrenRenderingContext.VF_INVISIBLE_FROM_CAMERA = 0x00000002; // flag for selected outlines
WbWrenRenderingContext.VF_CAMERA_FRUSTUMS = 0x00000008; // flag for camera frustum
WbWrenRenderingContext.VF_RANGE_FINDER_FRUSTUMS = 0x00004000; // flag for range-finder frustum
WbWrenRenderingContext.VF_NORMALS = 0x00040000; // Display mesh normals
// no special renderings, i.e. no outlines and no optional renderings from menu selection
WbWrenRenderingContext.VM_REGULAR = 0xFFF00000;
WbWrenRenderingContext.VM_MAIN = 0xFFFFFFFE;

WbWrenRenderingContext.PP_GTAO = 0;
WbWrenRenderingContext.PP_BLOOM = 1;
WbWrenRenderingContext.PP_HDR = 2;
WbWrenRenderingContext.PP_SMAA = 3;
