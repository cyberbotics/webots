#include <cu/cu.h>
#include "common.h"
#include <ccd/ccd.h>
#include "support.h"

#define TOSVT() \
    svtObjPen(&box, &cyl, stdout, "Pen 1", depth, &dir, &pos); \
    ccdVec3Scale(&dir, depth); \
    ccdVec3Add(&cyl.pos, &dir); \
    svtObjPen(&box, &cyl, stdout, "Pen 1", depth, &dir, &pos)

TEST(boxcylIntersect)
{
    ccd_t ccd;
    CCD_BOX(box);
    CCD_CYL(cyl);
    int res;
    ccd_vec3_t axis;

    box.x = 0.5;
    box.y = 1.;
    box.z = 1.5;
    cyl.radius = 0.4;
    cyl.height = 0.7;

    CCD_INIT(&ccd);
    ccd.support1 = ccdSupport;
    ccd.support2 = ccdSupport;

    ccdVec3Set(&cyl.pos, 0.1, 0., 0.);
    res = ccdGJKIntersect(&box, &cyl, &ccd);
    assertTrue(res);

    ccdVec3Set(&cyl.pos, .6, 0., 0.);
    res = ccdGJKIntersect(&box, &cyl, &ccd);
    assertTrue(res);

    ccdVec3Set(&cyl.pos, .6, 0.6, 0.);
    res = ccdGJKIntersect(&box, &cyl, &ccd);
    assertTrue(res);

    ccdVec3Set(&cyl.pos, .6, 0.6, 0.5);
    res = ccdGJKIntersect(&box, &cyl, &ccd);
    assertTrue(res);

    ccdVec3Set(&axis, 0., 1., 0.);
    ccdQuatSetAngleAxis(&cyl.quat, M_PI / 3., &axis);
    ccdVec3Set(&cyl.pos, .6, 0.6, 0.5);
    res = ccdGJKIntersect(&box, &cyl, &ccd);
    assertTrue(res);

    ccdVec3Set(&axis, 0.67, 1.1, 0.12);
    ccdQuatSetAngleAxis(&cyl.quat, M_PI / 4., &axis);
    ccdVec3Set(&cyl.pos, .6, 0., 0.5);
    res = ccdGJKIntersect(&box, &cyl, &ccd);
    assertTrue(res);

    ccdVec3Set(&axis, -0.1, 2.2, -1.);
    ccdQuatSetAngleAxis(&cyl.quat, M_PI / 5., &axis);
    ccdVec3Set(&cyl.pos, .6, 0., 0.5);
    ccdVec3Set(&axis, 1., 1., 0.);
    ccdQuatSetAngleAxis(&box.quat, -M_PI / 4., &axis);
    ccdVec3Set(&box.pos, .6, 0., 0.5);
    res = ccdGJKIntersect(&box, &cyl, &ccd);
    assertTrue(res);

    ccdVec3Set(&axis, -0.1, 2.2, -1.);
    ccdQuatSetAngleAxis(&cyl.quat, M_PI / 5., &axis);
    ccdVec3Set(&cyl.pos, .6, 0., 0.5);
    ccdVec3Set(&axis, 1., 1., 0.);
    ccdQuatSetAngleAxis(&box.quat, -M_PI / 4., &axis);
    ccdVec3Set(&box.pos, .9, 0.8, 0.5);
    res = ccdGJKIntersect(&box, &cyl, &ccd);
    assertTrue(res);
}

TEST(boxcylPenEPA)
{
    ccd_t ccd;
    CCD_BOX(box);
    CCD_CYL(cyl);
    int res;
    ccd_vec3_t axis;
    ccd_real_t depth;
    ccd_vec3_t dir, pos;

    box.x = 0.5;
    box.y = 1.;
    box.z = 1.5;
    cyl.radius = 0.4;
    cyl.height = 0.7;

    CCD_INIT(&ccd);
    ccd.support1 = ccdSupport;
    ccd.support2 = ccdSupport;

    ccdVec3Set(&cyl.pos, 0.1, 0., 0.);
    res = ccdGJKPenetration(&box, &cyl, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 1");
    //TOSVT();

    ccdVec3Set(&cyl.pos, .6, 0., 0.);
    res = ccdGJKPenetration(&box, &cyl, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 2");
    //TOSVT(); <<<

    ccdVec3Set(&cyl.pos, .6, 0.6, 0.);
    res = ccdGJKPenetration(&box, &cyl, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 3");
    //TOSVT();

    ccdVec3Set(&cyl.pos, .6, 0.6, 0.5);
    res = ccdGJKPenetration(&box, &cyl, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 4");
    //TOSVT();

    ccdVec3Set(&axis, 0., 1., 0.);
    ccdQuatSetAngleAxis(&cyl.quat, M_PI / 3., &axis);
    ccdVec3Set(&cyl.pos, .6, 0.6, 0.5);
    res = ccdGJKPenetration(&box, &cyl, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 5");
    //TOSVT();

    ccdVec3Set(&axis, 0.67, 1.1, 0.12);
    ccdQuatSetAngleAxis(&cyl.quat, M_PI / 4., &axis);
    ccdVec3Set(&cyl.pos, .6, 0., 0.5);
    res = ccdGJKPenetration(&box, &cyl, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 6");
    //TOSVT();

    ccdVec3Set(&axis, -0.1, 2.2, -1.);
    ccdQuatSetAngleAxis(&cyl.quat, M_PI / 5., &axis);
    ccdVec3Set(&cyl.pos, .6, 0., 0.5);
    ccdVec3Set(&axis, 1., 1., 0.);
    ccdQuatSetAngleAxis(&box.quat, -M_PI / 4., &axis);
    ccdVec3Set(&box.pos, .6, 0., 0.5);
    res = ccdGJKPenetration(&box, &cyl, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 7");
    //TOSVT();

    ccdVec3Set(&axis, -0.1, 2.2, -1.);
    ccdQuatSetAngleAxis(&cyl.quat, M_PI / 5., &axis);
    ccdVec3Set(&cyl.pos, .6, 0., 0.5);
    ccdVec3Set(&axis, 1., 1., 0.);
    ccdQuatSetAngleAxis(&box.quat, -M_PI / 4., &axis);
    ccdVec3Set(&box.pos, .9, 0.8, 0.5);
    res = ccdGJKPenetration(&box, &cyl, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 8");
    //TOSVT();
}
