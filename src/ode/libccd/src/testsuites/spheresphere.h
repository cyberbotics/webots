#ifndef SPHERE_SPHERE
#define SPHERE_SPHERE

#include <cu/cu.h>

TEST(spheresphereSetUp);
TEST(spheresphereTearDown);

TEST(spheresphereAlignedX);
TEST(spheresphereAlignedY);
TEST(spheresphereAlignedZ);

TEST_SUITE(TSSphereSphere) {
    TEST_ADD(spheresphereSetUp),

    TEST_ADD(spheresphereAlignedX),
    TEST_ADD(spheresphereAlignedY),
    TEST_ADD(spheresphereAlignedZ),

    TEST_ADD(spheresphereTearDown),
    TEST_SUITE_CLOSURE
};

#endif
