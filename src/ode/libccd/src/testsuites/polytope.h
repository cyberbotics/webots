#ifndef TEST_POLYTOPE_H
#define TEST_POLYTOPE_H

#include <cu/cu.h>

TEST(ptSetUp);
TEST(ptTearDown);

TEST(ptCreate1);
TEST(ptCreate2);
TEST(ptNearest);

TEST_SUITE(TSPt) {
    TEST_ADD(ptSetUp),

    TEST_ADD(ptCreate1),
    TEST_ADD(ptCreate2),
    TEST_ADD(ptNearest),

    TEST_ADD(ptTearDown),
    TEST_SUITE_CLOSURE
};

#endif
