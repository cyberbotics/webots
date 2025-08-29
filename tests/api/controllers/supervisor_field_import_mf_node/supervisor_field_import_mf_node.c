#include <webots/supervisor.h>
#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

const char *solid_text = "DEF FLOOR Solid {\n"
                         "  translation -0.5 0 0.5\n"
                         "  rotation 1 0 0 -1.57079632679\n"
                         "  children [\n"
                         "    Shape {\n"
                         "      geometry ElevationGrid {\n"
                         "        color Color {\n"
                         "          color [\n"
                         "            0.803922 0.498039 0.298039\n"
                         "            1 1 0\n"
                         "          ]\n"
                         "        }\n"
                         "        colorPerVertex FALSE\n"
                         "        xDimension 11\n"
                         "        xSpacing 0.1\n"
                         "        yDimension 11\n"
                         "        ySpacing 0.1\n"
                         "      }\n"
                         "    }\n"
                         "  ]\n"
                         "  boundingObject Pose {\n"
                         "    translation 0.5 0.5 0\n"
                         "    children [\n"
                         "      Plane {\n"
                         "      }\n"
                         "    ]\n"
                         "  }\n"
                         "}";

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbNodeRef root = wb_supervisor_node_get_root();
  ts_assert_pointer_not_null(root, "wb_supervisor_node_get_root() failed to return the root node.");

  WbFieldRef field = wb_supervisor_node_get_field(root, "children");
  ts_assert_pointer_not_null(field, "wb_supervisor_node_get_field() failed to find the 'children' field.");

  wb_supervisor_field_import_mf_node_from_string(field, 0, solid_text);

  WbNodeRef floor = wb_supervisor_node_get_from_def("FLOOR");
  ts_assert_pointer_not_null(floor, "wb_supervisor_node_get_from_def() failed to find the 'DEF FLOOR' node.");

  wb_supervisor_field_import_mf_node_from_string(field, 0, "DEF CHAIR Chair {}");
  WbNodeRef chair = wb_supervisor_node_get_from_def("CHAIR");
  ts_assert_pointer_not_null(chair, "wb_supervisor_node_get_from_def() failed to find the 'DEF CHAIR' node.");

  WbFieldRef color = wb_supervisor_node_get_field(chair, "color");
  ts_assert_pointer_not_null(color, "wb_supervisor_node_get_field() failed to find the 'color' proto parameter.");

  ts_send_success();
  return EXIT_SUCCESS;
}
