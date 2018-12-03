#ifndef TEST_VEC3_H
#define TEST_VEC3_H

#include <cu/cu.h>

TEST(vec3SetUp);
TEST(vec3TearDown);
TEST(vec3PointSegmentDist);
TEST(vec3PointTriDist);

TEST_SUITE(TSVec3) {
    TEST_ADD(vec3SetUp),

    TEST_ADD(vec3PointSegmentDist),
    TEST_ADD(vec3PointTriDist),

    TEST_ADD(vec3TearDown),
    TEST_SUITE_CLOSURE
};
#endif
