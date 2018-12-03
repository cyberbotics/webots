#include <stdio.h>
#include <cu/cu.h>
#include <ccd/vec3.h>

TEST(vec3SetUp)
{
}

TEST(vec3TearDown)
{
}

TEST(vec3PointSegmentDist)
{
    ccd_vec3_t P, a, b, w, ew;
    ccd_real_t dist;

    ccdVec3Set(&a, 0., 0., 0.);
    ccdVec3Set(&b, 1., 0., 0.);

    // extereme w == a
    ccdVec3Set(&P, -1., 0., 0.);
    dist = ccdVec3PointSegmentDist2(&P, &a, &b, &w);
    assertTrue(ccdEq(dist, 1.));
    assertTrue(ccdVec3Eq(&w, &a));

    ccdVec3Set(&P, -0.5, 0., 0.);
    dist = ccdVec3PointSegmentDist2(&P, &a, &b, &w);
    assertTrue(ccdEq(dist, 0.5 * 0.5));
    assertTrue(ccdVec3Eq(&w, &a));

    ccdVec3Set(&P, -0.1, 0., 0.);
    dist = ccdVec3PointSegmentDist2(&P, &a, &b, &w);
    assertTrue(ccdEq(dist, .1 * .1));
    assertTrue(ccdVec3Eq(&w, &a));

    ccdVec3Set(&P, 0., 0., 0.);
    dist = ccdVec3PointSegmentDist2(&P, &a, &b, &w);
    assertTrue(ccdEq(dist, 0.));
    assertTrue(ccdVec3Eq(&w, &a));

    ccdVec3Set(&P, -1., 1., 0.);
    dist = ccdVec3PointSegmentDist2(&P, &a, &b, &w);
    assertTrue(ccdEq(dist, 2.));
    assertTrue(ccdVec3Eq(&w, &a));

    ccdVec3Set(&P, -0.5, 0.5, 0.);
    dist = ccdVec3PointSegmentDist2(&P, &a, &b, &w);
    assertTrue(ccdEq(dist, 0.5));
    assertTrue(ccdVec3Eq(&w, &a));

    ccdVec3Set(&P, -0.1, -1., 2.);
    dist = ccdVec3PointSegmentDist2(&P, &a, &b, &w);
    assertTrue(ccdEq(dist, 5.01));
    assertTrue(ccdVec3Eq(&w, &a));

    // extereme w == b
    ccdVec3Set(&P, 2., 0., 0.);
    dist = ccdVec3PointSegmentDist2(&P, &a, &b, &w);
    assertTrue(ccdEq(dist, 1.));
    assertTrue(ccdVec3Eq(&w, &b));

    ccdVec3Set(&P, 1.5, 0., 0.);
    dist = ccdVec3PointSegmentDist2(&P, &a, &b, &w);
    assertTrue(ccdEq(dist, 0.5 * 0.5));
    assertTrue(ccdVec3Eq(&w, &b));

    ccdVec3Set(&P, 1.1, 0., 0.);
    dist = ccdVec3PointSegmentDist2(&P, &a, &b, &w);
    assertTrue(ccdEq(dist, .1 * .1));
    assertTrue(ccdVec3Eq(&w, &b));

    ccdVec3Set(&P, 1., 0., 0.);
    dist = ccdVec3PointSegmentDist2(&P, &a, &b, &w);
    assertTrue(ccdEq(dist, 0.));
    assertTrue(ccdVec3Eq(&w, &b));

    ccdVec3Set(&P, 2., 1., 0.);
    dist = ccdVec3PointSegmentDist2(&P, &a, &b, &w);
    assertTrue(ccdEq(dist, 2.));
    assertTrue(ccdVec3Eq(&w, &b));

    ccdVec3Set(&P, 1.5, 0.5, 0.);
    dist = ccdVec3PointSegmentDist2(&P, &a, &b, &w);
    assertTrue(ccdEq(dist, 0.5));
    assertTrue(ccdVec3Eq(&w, &b));

    ccdVec3Set(&P, 1.1, -1., 2.);
    dist = ccdVec3PointSegmentDist2(&P, &a, &b, &w);
    assertTrue(ccdEq(dist, 5.01));
    assertTrue(ccdVec3Eq(&w, &b));

    // inside segment
    ccdVec3Set(&P, .5, 0., 0.);
    dist = ccdVec3PointSegmentDist2(&P, &a, &b, &w);
    assertTrue(ccdEq(dist, 0.));
    assertTrue(ccdVec3Eq(&w, &P));

    ccdVec3Set(&P, .9, 0., 0.);
    dist = ccdVec3PointSegmentDist2(&P, &a, &b, &w);
    assertTrue(ccdEq(dist, 0.));
    assertTrue(ccdVec3Eq(&w, &P));

    ccdVec3Set(&P, .5, 1., 0.);
    dist = ccdVec3PointSegmentDist2(&P, &a, &b, &w);
    assertTrue(ccdEq(dist, 1.));
    ccdVec3Set(&ew, 0.5, 0., 0.);
    assertTrue(ccdVec3Eq(&w, &ew));

    ccdVec3Set(&P, .5, 1., 1.);
    dist = ccdVec3PointSegmentDist2(&P, &a, &b, &w);
    assertTrue(ccdEq(dist, 2.));
    ccdVec3Set(&ew, 0.5, 0., 0.);
    assertTrue(ccdVec3Eq(&w, &ew));

    ccdVec3Set(&a, -.5, 2., 1.);
    ccdVec3Set(&b, 1., 1.5, 0.5);

    // extereme w == a
    ccdVec3Set(&P, -10., 0., 0.);
    dist = ccdVec3PointSegmentDist2(&P, &a, &b, &w);
    assertTrue(ccdEq(dist, 9.5 * 9.5 + 2. * 2. + 1.));
    assertTrue(ccdVec3Eq(&w, &a));

    ccdVec3Set(&P, -10., 9.2, 3.4);
    dist = ccdVec3PointSegmentDist2(&P, &a, &b, &w);
    assertTrue(ccdEq(dist, 9.5 * 9.5 + 7.2 * 7.2 + 2.4 * 2.4));
    assertTrue(ccdVec3Eq(&w, &a));

    // extereme w == b
    ccdVec3Set(&P, 10., 0., 0.);
    dist = ccdVec3PointSegmentDist2(&P, &a, &b, &w);
    assertTrue(ccdEq(dist, 9. * 9. + 1.5 * 1.5 + 0.5 * 0.5));
    assertTrue(ccdVec3Eq(&w, &b));

    ccdVec3Set(&P, 10., 9.2, 3.4);
    dist = ccdVec3PointSegmentDist2(&P, &a, &b, &w);
    assertTrue(ccdEq(dist, 9. * 9. + 7.7 * 7.7 + 2.9 * 2.9));
    assertTrue(ccdVec3Eq(&w, &b));

    // inside ab
    ccdVec3Set(&a, -.1, 1., 1.);
    ccdVec3Set(&b, 1., 1., 1.);
    ccdVec3Set(&P, 0., 0., 0.);
    dist = ccdVec3PointSegmentDist2(&P, &a, &b, &w);
    assertTrue(ccdEq(dist, 2.));
    ccdVec3Set(&ew, 0., 1., 1.);
    assertTrue(ccdVec3Eq(&w, &ew));
}

TEST(vec3PointTriDist)
{
    ccd_vec3_t P, a, b, c, w, P0;
    ccd_real_t dist;

    ccdVec3Set(&a, -1., 0., 0.);
    ccdVec3Set(&b, 0., 1., 1.);
    ccdVec3Set(&c, -1., 0., 1.);

    ccdVec3Set(&P, -1., 0., 0.);
    dist = ccdVec3PointTriDist2(&P, &a, &b, &c, &w);
    assertTrue(ccdEq(dist, 0.));
    assertTrue(ccdVec3Eq(&w, &a));

    ccdVec3Set(&P, 0., 1., 1.);
    dist = ccdVec3PointTriDist2(&P, &a, &b, &c, &w);
    assertTrue(ccdEq(dist, 0.));
    assertTrue(ccdVec3Eq(&w, &b));

    ccdVec3Set(&P, -1., 0., 1.);
    dist = ccdVec3PointTriDist2(&P, &a, &b, &c, &w);
    assertTrue(ccdEq(dist, 0.));
    assertTrue(ccdVec3Eq(&w, &c));

    ccdVec3Set(&P, 0., 0., 0.);
    dist = ccdVec3PointTriDist2(&P, &a, &b, &c, NULL);
    assertTrue(ccdEq(dist, 2./3.));

    // region 4
    ccdVec3Set(&P, -2., 0., 0.);
    dist = ccdVec3PointTriDist2(&P, &a, &b, &c, &w);
    assertTrue(ccdEq(dist, ccdVec3Dist2(&P, &a)));
    assertTrue(ccdVec3Eq(&w, &a));
    ccdVec3Set(&P, -2., 0.2, -1.);
    dist = ccdVec3PointTriDist2(&P, &a, &b, &c, &w);
    assertTrue(ccdEq(dist, ccdVec3Dist2(&P, &a)));
    assertTrue(ccdVec3Eq(&w, &a));

    // region 2
    ccdVec3Set(&P, -1.3, 0., 1.2);
    dist = ccdVec3PointTriDist2(&P, &a, &b, &c, &w);
    assertTrue(ccdEq(dist, ccdVec3Dist2(&P, &c)));
    assertTrue(ccdVec3Eq(&w, &c));
    ccdVec3Set(&P, -1.2, 0.2, 1.1);
    dist = ccdVec3PointTriDist2(&P, &a, &b, &c, &w);
    assertTrue(ccdEq(dist, ccdVec3Dist2(&P, &c)));
    assertTrue(ccdVec3Eq(&w, &c));

    // region 6
    ccdVec3Set(&P, 0.3, 1., 1.);
    dist = ccdVec3PointTriDist2(&P, &a, &b, &c, &w);
    assertTrue(ccdEq(dist, ccdVec3Dist2(&P, &b)));
    assertTrue(ccdVec3Eq(&w, &b));
    ccdVec3Set(&P, .1, 1., 1.);
    dist = ccdVec3PointTriDist2(&P, &a, &b, &c, &w);
    assertTrue(ccdEq(dist, ccdVec3Dist2(&P, &b)));
    assertTrue(ccdVec3Eq(&w, &b));

    // region 1
    ccdVec3Set(&P, 0., 1., 2.);
    dist = ccdVec3PointTriDist2(&P, &a, &b, &c, &w);
    assertTrue(ccdEq(dist, 1.));
    assertTrue(ccdVec3Eq(&w, &b));
    ccdVec3Set(&P, -1., 0., 2.);
    dist = ccdVec3PointTriDist2(&P, &a, &b, &c, &w);
    assertTrue(ccdEq(dist, 1.));
    assertTrue(ccdVec3Eq(&w, &c));
    ccdVec3Set(&P, -0.5, 0.5, 2.);
    dist = ccdVec3PointTriDist2(&P, &a, &b, &c, &w);
    assertTrue(ccdEq(dist, 1.));
    ccdVec3Set(&P0, -0.5, 0.5, 1.);
    assertTrue(ccdVec3Eq(&w, &P0));

    // region 3
    ccdVec3Set(&P, -2., -1., 0.7);
    dist = ccdVec3PointTriDist2(&P, &a, &b, &c, &w);
    assertTrue(ccdEq(dist, 2.));
    ccdVec3Set(&P0, -1., 0., 0.7);
    assertTrue(ccdVec3Eq(&w, &P0));

    // region 5
    ccdVec3Set(&P, 0., 0., 0.);
    dist = ccdVec3PointTriDist2(&P, &a, &b, &c, &w);
    assertTrue(ccdEq(dist, 2./3.));
    ccdVec3Set(&P0, -2./3., 1./3., 1./3.);
    assertTrue(ccdVec3Eq(&w, &P0));

    // region 0
    ccdVec3Set(&P, -0.5, 0.5, 0.5);
    dist = ccdVec3PointTriDist2(&P, &a, &b, &c, &w);
    assertTrue(ccdEq(dist, 0.));
    assertTrue(ccdVec3Eq(&w, &P));
    ccdVec3Set(&P, -0.5, 0.5, 0.7);
    dist = ccdVec3PointTriDist2(&P, &a, &b, &c, &w);
    assertTrue(ccdEq(dist, 0.));
    assertTrue(ccdVec3Eq(&w, &P));
    ccdVec3Set(&P, -0.5, 0.5, 0.9);
    dist = ccdVec3PointTriDist2(&P, &a, &b, &c, &w);
    assertTrue(ccdEq(dist, 0.));
    assertTrue(ccdVec3Eq(&w, &P));

    ccdVec3Set(&P, 0., 0., 0.5);
    dist = ccdVec3PointTriDist2(&P, &a, &b, &c, &w);
    assertTrue(ccdEq(dist, 0.5));
    ccdVec3Set(&P0, -.5, .5, .5);
    assertTrue(ccdVec3Eq(&w, &P0));

    ccdVec3Set(&a, -1., 0., 0.);
    ccdVec3Set(&b, 0., 1., -1.);
    ccdVec3Set(&c, 0., 1., 1.);
    ccdVec3Set(&P, 0., 0., 0.);
    dist = ccdVec3PointTriDist2(&P, &a, &b, &c, &w);
    assertTrue(ccdEq(dist, 0.5));
    ccdVec3Set(&P0, -.5, .5, 0.);
    assertTrue(ccdVec3Eq(&w, &P0));
    //fprintf(stderr, "dist: %lf\n", dist);
}
