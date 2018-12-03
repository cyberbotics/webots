#include <stdio.h>
#include <cu/cu.h>

#include <ccd/ccd.h>
#include "support.h"
#include <ccd/vec3.h>
#include <ccd/dbg.h>
#include "common.h"

TEST(mprBoxboxAlignedX)
{
    size_t i;
    ccd_t ccd;
    CCD_BOX(box1);
    CCD_BOX(box2);
    int res;

    CCD_INIT(&ccd);
    ccd.support1 = ccdSupport;
    ccd.support2 = ccdSupport;
    ccd.center1  = ccdObjCenter;
    ccd.center2  = ccdObjCenter;

    box1.x = 1;
    box1.y = 2;
    box1.z = 1;
    box2.x = 2;
    box2.y = 1;
    box2.z = 2;

    ccdVec3Set(&box1.pos, -5., 0., 0.);
    ccdVec3Set(&box2.pos, 0., 0., 0.);
    ccdQuatSet(&box1.quat, 0., 0., 0., 1.);
    ccdQuatSet(&box2.quat, 0., 0., 0., 1.);
    for (i = 0; i < 100; i++){
        res = ccdMPRIntersect(&box1, &box2, &ccd);
        if (i < 35 || i > 65){
            assertFalse(res);
        }else if (i != 35 && i != 65){
            assertTrue(res);
        }

        box1.pos.v[0] += 0.1;
    }

    box1.x = 0.1;
    box1.y = 0.2;
    box1.z = 0.1;
    box2.x = 0.2;
    box2.y = 0.1;
    box2.z = 0.2;

    ccdVec3Set(&box1.pos, -0.5, 0., 0.);
    ccdVec3Set(&box2.pos, 0., 0., 0.);
    ccdQuatSet(&box1.quat, 0., 0., 0., 1.);
    ccdQuatSet(&box2.quat, 0., 0., 0., 1.);
    for (i = 0; i < 100; i++){
        res = ccdMPRIntersect(&box1, &box2, &ccd);

        if (i < 35 || i > 65){
            assertFalse(res);
        }else if (i != 35 && i != 65){
            assertTrue(res);
        }

        box1.pos.v[0] += 0.01;
    }

    box1.x = 1;
    box1.y = 2;
    box1.z = 1;
    box2.x = 2;
    box2.y = 1;
    box2.z = 2;

    ccdVec3Set(&box1.pos, -5., -0.1, 0.);
    ccdVec3Set(&box2.pos, 0., 0., 0.);
    ccdQuatSet(&box1.quat, 0., 0., 0., 1.);
    ccdQuatSet(&box2.quat, 0., 0., 0., 1.);
    for (i = 0; i < 100; i++){
        res = ccdMPRIntersect(&box1, &box2, &ccd);

        if (i < 35 || i > 65){
            assertFalse(res);
        }else if (i != 35 && i != 65){
            assertTrue(res);
        }

        box1.pos.v[0] += 0.1;
    }
}

TEST(mprBoxboxAlignedY)
{
    size_t i;
    ccd_t ccd;
    CCD_BOX(box1);
    CCD_BOX(box2);
    int res;

    CCD_INIT(&ccd);
    ccd.support1 = ccdSupport;
    ccd.support2 = ccdSupport;
    ccd.center1  = ccdObjCenter;
    ccd.center2  = ccdObjCenter;

    box1.x = 1;
    box1.y = 2;
    box1.z = 1;
    box2.x = 2;
    box2.y = 1;
    box2.z = 2;

    ccdVec3Set(&box1.pos, 0., -5., 0.);
    ccdVec3Set(&box2.pos, 0., 0., 0.);
    ccdQuatSet(&box1.quat, 0., 0., 0., 1.);
    ccdQuatSet(&box2.quat, 0., 0., 0., 1.);
    for (i = 0; i < 100; i++){
        res = ccdMPRIntersect(&box1, &box2, &ccd);

        if (i < 35 || i > 65){
            assertFalse(res);
        }else if (i != 35 && i != 65){
            assertTrue(res);
        }

        box1.pos.v[1] += 0.1;
    }
}

TEST(mprBoxboxAlignedZ)
{
    size_t i;
    ccd_t ccd;
    CCD_BOX(box1);
    CCD_BOX(box2);
    int res;

    CCD_INIT(&ccd);
    ccd.support1 = ccdSupport;
    ccd.support2 = ccdSupport;
    ccd.center1  = ccdObjCenter;
    ccd.center2  = ccdObjCenter;

    box1.x = 1;
    box1.y = 2;
    box1.z = 1;
    box2.x = 2;
    box2.y = 1;
    box2.z = 2;

    ccdVec3Set(&box1.pos, 0., 0., -5.);
    ccdVec3Set(&box2.pos, 0., 0., 0.);
    ccdQuatSet(&box1.quat, 0., 0., 0., 1.);
    ccdQuatSet(&box2.quat, 0., 0., 0., 1.);
    for (i = 0; i < 100; i++){
        res = ccdMPRIntersect(&box1, &box2, &ccd);

        if (i < 35 || i > 65){
            assertFalse(res);
        }else if (i != 35 && i != 65){
            assertTrue(res);
        }

        box1.pos.v[2] += 0.1;
    }
}

TEST(mprBoxboxRot)
{
    size_t i;
    ccd_t ccd;
    CCD_BOX(box1);
    CCD_BOX(box2);
    int res;
    ccd_vec3_t axis;
    ccd_real_t angle;

    CCD_INIT(&ccd);
    ccd.support1 = ccdSupport;
    ccd.support2 = ccdSupport;
    ccd.center1  = ccdObjCenter;
    ccd.center2  = ccdObjCenter;

    box1.x = 1;
    box1.y = 2;
    box1.z = 1;
    box2.x = 2;
    box2.y = 1;
    box2.z = 2;

    ccdVec3Set(&box1.pos, -5., 0.5, 0.);
    ccdVec3Set(&box2.pos, 0., 0., 0.);
    ccdQuatSet(&box2.quat, 0., 0., 0., 1.);
    ccdVec3Set(&axis, 0., 1., 0.);
    ccdQuatSetAngleAxis(&box1.quat, M_PI / 4., &axis);

    for (i = 0; i < 100; i++){
        res = ccdMPRIntersect(&box1, &box2, &ccd);

        if (i < 33 || i > 67){
            assertFalse(res);
        }else if (i != 33 && i != 67){
            assertTrue(res);
        }

        box1.pos.v[0] += 0.1;
    }

    box1.x = 1;
    box1.y = 1;
    box1.z = 1;
    box2.x = 1;
    box2.y = 1;
    box2.z = 1;

    ccdVec3Set(&box1.pos, -1.01, 0., 0.);
    ccdVec3Set(&box2.pos, 0., 0., 0.);
    ccdQuatSet(&box1.quat, 0., 0., 0., 1.);
    ccdQuatSet(&box2.quat, 0., 0., 0., 1.);

    ccdVec3Set(&axis, 0., 1., 0.);
    angle = 0.;
    for (i = 0; i < 30; i++){
        res = ccdMPRIntersect(&box1, &box2, &ccd);

        if (i != 0 && i != 10 && i != 20){
            assertTrue(res);
        }else{
            assertFalse(res);
        }

        angle += M_PI / 20.;
        ccdQuatSetAngleAxis(&box1.quat, angle, &axis);
    }

}

static void pConf(ccd_box_t *box1, ccd_box_t *box2, const ccd_vec3_t *v)
{
    fprintf(stdout, "# box1.pos: [%lf %lf %lf]\n",
            ccdVec3X(&box1->pos), ccdVec3Y(&box1->pos), ccdVec3Z(&box1->pos));
    fprintf(stdout, "# box1->quat: [%lf %lf %lf %lf]\n",
            box1->quat.q[0], box1->quat.q[1], box1->quat.q[2], box1->quat.q[3]);
    fprintf(stdout, "# box2->pos: [%lf %lf %lf]\n",
            ccdVec3X(&box2->pos), ccdVec3Y(&box2->pos), ccdVec3Z(&box2->pos));
    fprintf(stdout, "# box2->quat: [%lf %lf %lf %lf]\n",
            box2->quat.q[0], box2->quat.q[1], box2->quat.q[2], box2->quat.q[3]);
    fprintf(stdout, "# sep: [%lf %lf %lf]\n",
            ccdVec3X(v), ccdVec3Y(v), ccdVec3Z(v));
    fprintf(stdout, "#\n");
}

TEST(mprBoxboxSeparate)
{
    ccd_t ccd;
    CCD_BOX(box1);
    CCD_BOX(box2);
    int res;
    ccd_vec3_t sep, expsep, expsep2, axis;

    fprintf(stderr, "\n\n\n---- boxboxSeparate ----\n\n\n");

    box1.x = box1.y = box1.z = 1.;
    box2.x = 0.5;
    box2.y = 1.;
    box2.z = 1.5;

    CCD_INIT(&ccd);
    ccd.support1 = ccdSupport;
    ccd.support2 = ccdSupport;

    ccdVec3Set(&box1.pos, -0.5, 0.5, 0.2);
    res = ccdMPRIntersect(&box1, &box2, &ccd);
    assertTrue(res);

    res = ccdGJKSeparate(&box1, &box2, &ccd, &sep);
    assertTrue(res == 0);
    ccdVec3Set(&expsep, 0.25, 0., 0.);
    assertTrue(ccdVec3Eq(&sep, &expsep));

    ccdVec3Scale(&sep, -1.);
    ccdVec3Add(&box1.pos, &sep);
    res = ccdGJKSeparate(&box1, &box2, &ccd, &sep);
    assertTrue(res == 0);
    ccdVec3Set(&expsep, 0., 0., 0.);
    assertTrue(ccdVec3Eq(&sep, &expsep));

    ccdVec3Set(&box1.pos, -0.3, 0.5, 1.);
    res = ccdGJKSeparate(&box1, &box2, &ccd, &sep);
    assertTrue(res == 0);
    ccdVec3Set(&expsep, 0., 0., -0.25);
    assertTrue(ccdVec3Eq(&sep, &expsep));

    box1.x = box1.y = box1.z = 1.;
    box2.x = box2.y = box2.z = 1.;
    ccdVec3Set(&axis, 0., 0., 1.);
    ccdQuatSetAngleAxis(&box1.quat, M_PI / 4., &axis);
    ccdVec3Set(&box1.pos, 0., 0., 0.);

    res = ccdGJKSeparate(&box1, &box2, &ccd, &sep);
    assertTrue(res == 0);
    ccdVec3Set(&expsep, 0., 0., 1.);
    ccdVec3Set(&expsep2, 0., 0., -1.);
    assertTrue(ccdVec3Eq(&sep, &expsep) || ccdVec3Eq(&sep, &expsep2));

    box1.x = box1.y = box1.z = 1.;
    ccdVec3Set(&axis, 0., 0., 1.);
    ccdQuatSetAngleAxis(&box1.quat, M_PI / 4., &axis);
    ccdVec3Set(&box1.pos, -0.5, 0., 0.);

    res = ccdGJKSeparate(&box1, &box2, &ccd, &sep);
    assertTrue(res == 0);
    pConf(&box1, &box2, &sep);

    box1.x = box1.y = box1.z = 1.;
    ccdVec3Set(&axis, 0., 1., 1.);
    ccdQuatSetAngleAxis(&box1.quat, M_PI / 4., &axis);
    ccdVec3Set(&box1.pos, -0.5, 0.1, 0.4);

    res = ccdGJKSeparate(&box1, &box2, &ccd, &sep);
    assertTrue(res == 0);
    pConf(&box1, &box2, &sep);
}

#define TOSVT() \
    svtObjPen(&box1, &box2, stdout, "Pen 1", depth, &dir, &pos); \
    ccdVec3Scale(&dir, depth); \
    ccdVec3Add(&box2.pos, &dir); \
    svtObjPen(&box1, &box2, stdout, "Pen 1", depth, &dir, &pos)

TEST(mprBoxboxPenetration)
{
    ccd_t ccd;
    CCD_BOX(box1);
    CCD_BOX(box2);
    int res;
    ccd_vec3_t axis;
    ccd_quat_t rot;
    ccd_real_t depth;
    ccd_vec3_t dir, pos;

    fprintf(stderr, "\n\n\n---- boxboxPenetration ----\n\n\n");

    box1.x = box1.y = box1.z = 1.;
    box2.x = 0.5;
    box2.y = 1.;
    box2.z = 1.5;

    CCD_INIT(&ccd);
    ccd.support1 = ccdSupport;
    ccd.support2 = ccdSupport;
    ccd.center1  = ccdObjCenter;
    ccd.center2  = ccdObjCenter;

    /*
    ccdVec3Set(&box2.pos, 0., 0., 0.);
    res = ccdMPRPenetration(&box1, &box2, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 1");
    TOSVT();
    */

    ccdVec3Set(&box2.pos, 0.1, 0., 0.);
    res = ccdMPRPenetration(&box1, &box2, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 1");
    //TOSVT();

    ccdVec3Set(&box1.pos, -0.3, 0.5, 1.);
    res = ccdMPRPenetration(&box1, &box2, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 2");
    //TOSVT();

    box1.x = box1.y = box1.z = 1.;
    box2.x = box2.y = box2.z = 1.;
    ccdVec3Set(&axis, 0., 0., 1.);
    ccdQuatSetAngleAxis(&box1.quat, M_PI / 4., &axis);
    ccdVec3Set(&box1.pos, 0.1, 0., 0.1);

    res = ccdMPRPenetration(&box1, &box2, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 3");
    //TOSVT();

    box1.x = box1.y = box1.z = 1.;
    box2.x = box2.y = box2.z = 1.;
    ccdVec3Set(&axis, 0., 0., 1.);
    ccdQuatSetAngleAxis(&box1.quat, M_PI / 4., &axis);
    ccdVec3Set(&box1.pos, -0.5, 0., 0.);

    res = ccdMPRPenetration(&box1, &box2, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 4");
    //TOSVT();

    box1.x = box1.y = box1.z = 1.;
    box2.x = box2.y = box2.z = 1.;
    ccdVec3Set(&axis, 0., 0., 1.);
    ccdQuatSetAngleAxis(&box1.quat, M_PI / 4., &axis);
    ccdVec3Set(&box1.pos, -0.5, 0.5, 0.);

    res = ccdMPRPenetration(&box1, &box2, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 5");
    //TOSVT();

    box1.x = box1.y = box1.z = 1.;
    box2.x = box2.y = box2.z = 1.;
    ccdVec3Set(&box2.pos, 0.1, 0., 0.);

    box1.x = box1.y = box1.z = 1.;
    ccdVec3Set(&axis, 0., 1., 1.);
    ccdQuatSetAngleAxis(&box1.quat, M_PI / 4., &axis);
    ccdVec3Set(&box1.pos, -0.5, 0.1, 0.4);

    res = ccdMPRPenetration(&box1, &box2, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 6");
    //TOSVT();

    box1.x = box1.y = box1.z = 1.;
    ccdVec3Set(&axis, 0., 1., 1.);
    ccdQuatSetAngleAxis(&box1.quat, M_PI / 4., &axis);
    ccdVec3Set(&axis, 1., 1., 1.);
    ccdQuatSetAngleAxis(&rot, M_PI / 4., &axis);
    ccdQuatMul(&box1.quat, &rot);
    ccdVec3Set(&box1.pos, -0.5, 0.1, 0.4);

    res = ccdMPRPenetration(&box1, &box2, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 7");
    //TOSVT();

    box1.x = box1.y = box1.z = 1.;
    box2.x = 0.2; box2.y = 0.5; box2.z = 1.;
    box2.x = box2.y = box2.z = 1.;

    ccdVec3Set(&axis, 0., 0., 1.);
    ccdQuatSetAngleAxis(&box1.quat, M_PI / 4., &axis);
    ccdVec3Set(&axis, 1., 0., 0.);
    ccdQuatSetAngleAxis(&rot, M_PI / 4., &axis);
    ccdQuatMul(&box1.quat, &rot);
    ccdVec3Set(&box1.pos, -1.3, 0., 0.);

    ccdVec3Set(&box2.pos, 0., 0., 0.);

    res = ccdMPRPenetration(&box1, &box2, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 8");
    //TOSVT();

    box1.x = box1.y = box1.z = 1.;
    box2.x = 0.5; box2.y = 0.5; box2.z = .5;
    ccdVec3Set(&box1.pos, 0., 0., 0.);
    ccdQuatSet(&box1.quat, 0., 0., 0., 1.);
    ccdVec3Set(&box2.pos, 0., 0.73, 0.);
    ccdQuatSet(&box2.quat, 0., 0., 0., 1.);

    res = ccdMPRPenetration(&box1, &box2, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 9");
    //TOSVT();

    box1.x = box1.y = box1.z = 1.;
    box2.x = 0.5; box2.y = 0.5; box2.z = .5;
    ccdVec3Set(&box1.pos, 0., 0., 0.);
    ccdQuatSet(&box1.quat, 0., 0., 0., 1.);
    ccdVec3Set(&box2.pos, 0.3, 0.738, 0.);
    ccdQuatSet(&box2.quat, 0., 0., 0., 1.);

    res = ccdMPRPenetration(&box1, &box2, &ccd, &depth, &dir, &pos);
    assertTrue(res == 0);
    recPen(depth, &dir, &pos, stdout, "Pen 10");
    //TOSVT();
}
