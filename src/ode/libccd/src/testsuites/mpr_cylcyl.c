#include <stdio.h>
#include <cu/cu.h>
#include <ccd/ccd.h>
#include "support.h"
#include "common.h"

TEST(mprCylcylAlignedX)
{
    ccd_t ccd;
    CCD_CYL(c1);
    CCD_CYL(c2);
    size_t i;
    int res;

    CCD_INIT(&ccd);
    ccd.support1 = ccdSupport;
    ccd.support2 = ccdSupport;
    ccd.center1  = ccdObjCenter;
    ccd.center2  = ccdObjCenter;

    c1.radius = 0.35;
    c1.height = 0.5;
    c2.radius = 0.5;
    c2.height = 1.;

    ccdVec3Set(&c1.pos, -5., 0., 0.);
    for (i = 0; i < 100; i++){
        res = ccdMPRIntersect(&c1, &c2, &ccd);

        if (i < 42 || i > 58){
            assertFalse(res);
        }else{
            assertTrue(res);
        }

        c1.pos.v[0] += 0.1;
    }
}

TEST(mprCylcylAlignedY)
{
    ccd_t ccd;
    CCD_CYL(c1);
    CCD_CYL(c2);
    size_t i;
    int res;

    CCD_INIT(&ccd);
    ccd.support1 = ccdSupport;
    ccd.support2 = ccdSupport;
    ccd.center1  = ccdObjCenter;
    ccd.center2  = ccdObjCenter;

    c1.radius = 0.35;
    c1.height = 0.5;
    c2.radius = 0.5;
    c2.height = 1.;

    ccdVec3Set(&c1.pos, 0., -5., 0.);
    for (i = 0; i < 100; i++){
        res = ccdMPRIntersect(&c1, &c2, &ccd);

        if (i < 42 || i > 58){
            assertFalse(res);
        }else{
            assertTrue(res);
        }

        c1.pos.v[1] += 0.1;
    }
}

TEST(mprCylcylAlignedZ)
{
    ccd_t ccd;
    CCD_CYL(c1);
    CCD_CYL(c2);
    size_t i;
    int res;

    CCD_INIT(&ccd);
    ccd.support1 = ccdSupport;
    ccd.support2 = ccdSupport;
    ccd.center1  = ccdObjCenter;
    ccd.center2  = ccdObjCenter;

    c1.radius = 0.35;
    c1.height = 0.5;
    c2.radius = 0.5;
    c2.height = 1.;

    ccdVec3Set(&c1.pos, 0., 0., -5.);
    for (i = 0; i < 100; i++){
        res = ccdMPRIntersect(&c1, &c2, &ccd);

        if (i < 43 || i > 57){
            assertFalse(res);
        }else{
            assertTrue(res);
        }

        c1.pos.v[2] += 0.1;
    }
}

#define TOSVT() \
    svtObjPen(&cyl1, &cyl2, stdout, "Pen 1", depth, &dir, &pos); \
    ccdVec3Scale(&dir, depth); \
    ccdVec3Add(&cyl2.pos, &dir); \
    svtObjPen(&cyl1, &cyl2, stdout, "Pen 1", depth, &dir, &pos)

TEST(mprCylcylPenetration)
{
    ccd_t ccd;
    CCD_CYL(cyl1);
    CCD_CYL(cyl2);
    int res;
    ccd_vec3_t axis;
    ccd_real_t depth;
    ccd_vec3_t dir, pos;

    fprintf(stderr, "\n\n\n---- mprCylcylPenetration ----\n\n\n");

    cyl1.radius = 0.35;
    cyl1.height = 0.5;
    cyl2.radius = 0.5;
    cyl2.height = 1.;

    CCD_INIT(&ccd);
    ccd.support1 = ccdSupport;
    ccd.support2 = ccdSupport;
    ccd.center1  = ccdObjCenter;
    ccd.center2  = ccdObjCenter;

    ccdVec3Set(&cyl2.pos, 0., 0., 0.3);
    res = ccdMPRPenetration(&cyl1, &cyl2, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 1");
    //TOSVT();

    ccdVec3Set(&cyl1.pos, 0.3, 0.1, 0.1);
    res = ccdMPRPenetration(&cyl1, &cyl2, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 2");
    //TOSVT();

    ccdVec3Set(&axis, 0., 1., 1.);
    ccdQuatSetAngleAxis(&cyl2.quat, M_PI / 4., &axis);
    ccdVec3Set(&cyl2.pos, 0., 0., 0.);
    res = ccdMPRPenetration(&cyl1, &cyl2, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 3");
    //TOSVT();

    ccdVec3Set(&axis, 0., 1., 1.);
    ccdQuatSetAngleAxis(&cyl2.quat, M_PI / 4., &axis);
    ccdVec3Set(&cyl2.pos, -0.2, 0.7, 0.2);
    res = ccdMPRPenetration(&cyl1, &cyl2, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 4");
    //TOSVT();

    ccdVec3Set(&axis, 0.567, 1.2, 1.);
    ccdQuatSetAngleAxis(&cyl2.quat, M_PI / 4., &axis);
    ccdVec3Set(&cyl2.pos, 0.6, -0.7, 0.2);
    res = ccdMPRPenetration(&cyl1, &cyl2, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 5");
    //TOSVT();

    ccdVec3Set(&axis, -4.567, 1.2, 0.);
    ccdQuatSetAngleAxis(&cyl2.quat, M_PI / 3., &axis);
    ccdVec3Set(&cyl2.pos, 0.6, -0.7, 0.2);
    res = ccdMPRPenetration(&cyl1, &cyl2, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 6");
    //TOSVT();
}
