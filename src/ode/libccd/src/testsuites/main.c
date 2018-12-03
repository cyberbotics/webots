#include "vec3.h"
#include "polytope.h"
#include "boxbox.h"
#include "spheresphere.h"
#include "cylcyl.h"
#include "boxcyl.h"

#include "mpr_boxbox.h"
#include "mpr_cylcyl.h"
#include "mpr_boxcyl.h"

TEST_SUITES {
    TEST_SUITE_ADD(TSVec3),
    TEST_SUITE_ADD(TSPt),
    TEST_SUITE_ADD(TSBoxBox),
    TEST_SUITE_ADD(TSSphereSphere),
    TEST_SUITE_ADD(TSCylCyl),
    TEST_SUITE_ADD(TSBoxCyl),

    TEST_SUITE_ADD(TSMPRBoxBox),
    TEST_SUITE_ADD(TSMPRCylCyl),
    TEST_SUITE_ADD(TSMPRBoxCyl),

    TEST_SUITES_CLOSURE
};
int main(int argc, char *argv[])
{
    CU_SET_OUT_PREFIX("regressions/");
    CU_RUN(argc, argv);

    return 0;
}
