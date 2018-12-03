#ifndef BOX_BOX
#define BOX_BOX

#include <cu/cu.h>

TEST(boxboxSetUp);
TEST(boxboxTearDown);

TEST(boxboxAlignedX);
TEST(boxboxAlignedY);
TEST(boxboxAlignedZ);

TEST(boxboxRot);

TEST(boxboxSeparate);
TEST(boxboxPenetration);

TEST_SUITE(TSBoxBox) {
    TEST_ADD(boxboxSetUp),

    TEST_ADD(boxboxAlignedX),
    TEST_ADD(boxboxAlignedY),
    TEST_ADD(boxboxAlignedZ),
    TEST_ADD(boxboxRot),
    TEST_ADD(boxboxSeparate),
    TEST_ADD(boxboxPenetration),

    TEST_ADD(boxboxTearDown),
    TEST_SUITE_CLOSURE
};

#endif
