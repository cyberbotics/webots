#include <stdio.h>
#include <cu/cu.h>
#include "support.h"
#include <ccd/ccd.h>

TEST(spheresphereSetUp)
{
}

TEST(spheresphereTearDown)
{
}

TEST(spheresphereAlignedX)
{
    ccd_t ccd;
    CCD_SPHERE(s1);
    CCD_SPHERE(s2);
    size_t i;
    int res;

    CCD_INIT(&ccd);
    ccd.support1 = ccdSupport;
    ccd.support2 = ccdSupport;

    s1.radius = 0.35;
    s2.radius = .5;

    ccdVec3Set(&s1.pos, -5., 0., 0.);
    for (i = 0; i < 100; i++){
        res = ccdGJKIntersect(&s1, &s2, &ccd);

        if (i < 42 || i > 58){
            assertFalse(res);
        }else{
            assertTrue(res);
        }

        s1.pos.v[0] += 0.1;
    }
}

TEST(spheresphereAlignedY)
{
    ccd_t ccd;
    CCD_SPHERE(s1);
    CCD_SPHERE(s2);
    size_t i;
    int res;

    CCD_INIT(&ccd);
    ccd.support1 = ccdSupport;
    ccd.support2 = ccdSupport;

    s1.radius = 0.35;
    s2.radius = .5;

    ccdVec3Set(&s1.pos, 0., -5., 0.);
    for (i = 0; i < 100; i++){
        res = ccdGJKIntersect(&s1, &s2, &ccd);

        if (i < 42 || i > 58){
            assertFalse(res);
        }else{
            assertTrue(res);
        }

        s1.pos.v[1] += 0.1;
    }
}

TEST(spheresphereAlignedZ)
{
    ccd_t ccd;
    CCD_SPHERE(s1);
    CCD_SPHERE(s2);
    size_t i;
    int res;

    CCD_INIT(&ccd);
    ccd.support1 = ccdSupport;
    ccd.support2 = ccdSupport;

    s1.radius = 0.35;
    s2.radius = .5;

    ccdVec3Set(&s1.pos, 0., 0., -5.);
    for (i = 0; i < 100; i++){
        res = ccdGJKIntersect(&s1, &s2, &ccd);

        if (i < 42 || i > 58){
            assertFalse(res);
        }else{
            assertTrue(res);
        }

        s1.pos.v[2] += 0.1;
    }
}
