#ifndef TEST_BOXCYL_H
#define TEST_BOXCYL_H

#include <cu/cu.h>

TEST(boxcylIntersect);
TEST(boxcylPenEPA);

TEST_SUITE(TSBoxCyl){
    TEST_ADD(boxcylIntersect),
    TEST_ADD(boxcylPenEPA),

    TEST_SUITE_CLOSURE
};

#endif
