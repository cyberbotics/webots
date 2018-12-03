#ifndef CYL_CYL
#define CYL_CYL

#include <cu/cu.h>

TEST(cylcylSetUp);
TEST(cylcylTearDown);

TEST(cylcylAlignedX);
TEST(cylcylAlignedY);
TEST(cylcylAlignedZ);

TEST(cylcylPenetrationEPA);

TEST_SUITE(TSCylCyl) {
    TEST_ADD(cylcylSetUp),

    TEST_ADD(cylcylAlignedX),
    TEST_ADD(cylcylAlignedY),
    TEST_ADD(cylcylAlignedZ),

    TEST_ADD(cylcylPenetrationEPA),

    TEST_ADD(cylcylTearDown),
    TEST_SUITE_CLOSURE
};

#endif
