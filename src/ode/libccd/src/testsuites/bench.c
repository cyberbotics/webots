#define CU_ENABLE_TIMER
#include <cu/cu.h>
#include <stdio.h>
#include <stdlib.h>
#include <ccd/ccd.h>
#include "support.h"

TEST_SUITES {
    TEST_SUITES_CLOSURE
};

static int bench_num = 1;
static size_t cycles = 10000;

static void runBench(const void *o1, const void *o2, const ccd_t *ccd)
{
    ccd_real_t depth;
    ccd_vec3_t dir, pos;
    int res;
    size_t i;
    const struct timespec *timer;

    cuTimerStart();
    for (i = 0; i < cycles; i++){
        res = ccdGJKPenetration(o1, o2, ccd, &depth, &dir, &pos);
    }
    timer = cuTimerStop();
    fprintf(stdout, "%02d: %ld %ld\n", bench_num,
                    (long)timer->tv_sec, (long)timer->tv_nsec);
    fflush(stdout);

    bench_num++;
}

static void boxbox(void)
{
    fprintf(stdout, "%s:\n", __func__);

    ccd_t ccd;
    CCD_BOX(box1);
    CCD_BOX(box2);
    ccd_vec3_t axis;
    ccd_quat_t rot;

    box1.x = box1.y = box1.z = 1.;
    box2.x = 0.5;
    box2.y = 1.;
    box2.z = 1.5;

    bench_num = 1;

    CCD_INIT(&ccd);
    ccd.support1 = ccdSupport;
    ccd.support2 = ccdSupport;

    runBench(&box1, &box2, &ccd);
    runBench(&box2, &box1, &ccd);

    ccdVec3Set(&box1.pos, -0.3, 0.5, 1.);
    runBench(&box1, &box2, &ccd);
    runBench(&box2, &box1, &ccd);

    box1.x = box1.y = box1.z = 1.;
    box2.x = box2.y = box2.z = 1.;
    ccdVec3Set(&axis, 0., 0., 1.);
    ccdQuatSetAngleAxis(&box1.quat, M_PI / 4., &axis);
    ccdVec3Set(&box1.pos, 0., 0., 0.);
    runBench(&box1, &box2, &ccd);
    runBench(&box2, &box1, &ccd);

    box1.x = box1.y = box1.z = 1.;
    box2.x = box2.y = box2.z = 1.;
    ccdVec3Set(&axis, 0., 0., 1.);
    ccdQuatSetAngleAxis(&box1.quat, M_PI / 4., &axis);
    ccdVec3Set(&box1.pos, -0.5, 0., 0.);
    runBench(&box1, &box2, &ccd);
    runBench(&box2, &box1, &ccd);

    box1.x = box1.y = box1.z = 1.;
    box2.x = box2.y = box2.z = 1.;
    ccdVec3Set(&axis, 0., 0., 1.);
    ccdQuatSetAngleAxis(&box1.quat, M_PI / 4., &axis);
    ccdVec3Set(&box1.pos, -0.5, 0.5, 0.);
    runBench(&box1, &box2, &ccd);
    runBench(&box2, &box1, &ccd);

    box1.x = box1.y = box1.z = 1.;
    ccdVec3Set(&axis, 0., 1., 1.);
    ccdQuatSetAngleAxis(&box1.quat, M_PI / 4., &axis);
    ccdVec3Set(&box1.pos, -0.5, 0.1, 0.4);
    runBench(&box1, &box2, &ccd);
    runBench(&box2, &box1, &ccd);

    box1.x = box1.y = box1.z = 1.;
    ccdVec3Set(&axis, 0., 1., 1.);
    ccdQuatSetAngleAxis(&box1.quat, M_PI / 4., &axis);
    ccdVec3Set(&axis, 1., 1., 1.);
    ccdQuatSetAngleAxis(&rot, M_PI / 4., &axis);
    ccdQuatMul(&box1.quat, &rot);
    ccdVec3Set(&box1.pos, -0.5, 0.1, 0.4);
    runBench(&box1, &box2, &ccd);
    runBench(&box2, &box1, &ccd);

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
    runBench(&box1, &box2, &ccd);
    runBench(&box2, &box1, &ccd);

    fprintf(stdout, "\n----\n\n");
}

void cylcyl(void)
{
    fprintf(stdout, "%s:\n", __func__);

    ccd_t ccd;
    CCD_CYL(cyl1);
    CCD_CYL(cyl2);
    ccd_vec3_t axis;

    cyl1.radius = 0.35;
    cyl1.height = 0.5;
    cyl2.radius = 0.5;
    cyl2.height = 1.;

    CCD_INIT(&ccd);
    ccd.support1 = ccdSupport;
    ccd.support2 = ccdSupport;

    runBench(&cyl1, &cyl2, &ccd);
    runBench(&cyl2, &cyl1, &ccd);

    ccdVec3Set(&cyl1.pos, 0.3, 0.1, 0.1);
    runBench(&cyl1, &cyl2, &ccd);
    runBench(&cyl2, &cyl1, &ccd);

    ccdVec3Set(&axis, 0., 1., 1.);
    ccdQuatSetAngleAxis(&cyl2.quat, M_PI / 4., &axis);
    ccdVec3Set(&cyl2.pos, 0., 0., 0.);
    runBench(&cyl1, &cyl2, &ccd);
    runBench(&cyl2, &cyl1, &ccd);

    ccdVec3Set(&axis, 0., 1., 1.);
    ccdQuatSetAngleAxis(&cyl2.quat, M_PI / 4., &axis);
    ccdVec3Set(&cyl2.pos, -0.2, 0.7, 0.2);
    runBench(&cyl1, &cyl2, &ccd);
    runBench(&cyl2, &cyl1, &ccd);

    ccdVec3Set(&axis, 0.567, 1.2, 1.);
    ccdQuatSetAngleAxis(&cyl2.quat, M_PI / 4., &axis);
    ccdVec3Set(&cyl2.pos, 0.6, -0.7, 0.2);
    runBench(&cyl1, &cyl2, &ccd);
    runBench(&cyl2, &cyl1, &ccd);

    ccdVec3Set(&axis, -4.567, 1.2, 0.);
    ccdQuatSetAngleAxis(&cyl2.quat, M_PI / 3., &axis);
    ccdVec3Set(&cyl2.pos, 0.6, -0.7, 0.2);
    runBench(&cyl1, &cyl2, &ccd);
    runBench(&cyl2, &cyl1, &ccd);

    fprintf(stdout, "\n----\n\n");
}

void boxcyl(void)
{
    fprintf(stdout, "%s:\n", __func__);

    ccd_t ccd;
    CCD_BOX(box);
    CCD_CYL(cyl);
    ccd_vec3_t axis;

    box.x = 0.5;
    box.y = 1.;
    box.z = 1.5;
    cyl.radius = 0.4;
    cyl.height = 0.7;

    CCD_INIT(&ccd);
    ccd.support1 = ccdSupport;
    ccd.support2 = ccdSupport;

    runBench(&box, &cyl, &ccd);
    runBench(&cyl, &box, &ccd);

    ccdVec3Set(&cyl.pos, .6, 0., 0.);
    runBench(&box, &cyl, &ccd);
    runBench(&cyl, &box, &ccd);

    ccdVec3Set(&cyl.pos, .6, 0.6, 0.);
    runBench(&box, &cyl, &ccd);
    runBench(&cyl, &box, &ccd);

    ccdVec3Set(&cyl.pos, .6, 0.6, 0.5);
    runBench(&box, &cyl, &ccd);
    runBench(&cyl, &box, &ccd);

    ccdVec3Set(&axis, 0., 1., 0.);
    ccdQuatSetAngleAxis(&cyl.quat, M_PI / 3., &axis);
    ccdVec3Set(&cyl.pos, .6, 0.6, 0.5);
    runBench(&box, &cyl, &ccd);
    runBench(&cyl, &box, &ccd);

    ccdVec3Set(&axis, 0.67, 1.1, 0.12);
    ccdQuatSetAngleAxis(&cyl.quat, M_PI / 4., &axis);
    ccdVec3Set(&cyl.pos, .6, 0., 0.5);
    runBench(&box, &cyl, &ccd);
    runBench(&cyl, &box, &ccd);

    ccdVec3Set(&axis, -0.1, 2.2, -1.);
    ccdQuatSetAngleAxis(&cyl.quat, M_PI / 5., &axis);
    ccdVec3Set(&cyl.pos, .6, 0., 0.5);
    ccdVec3Set(&axis, 1., 1., 0.);
    ccdQuatSetAngleAxis(&box.quat, -M_PI / 4., &axis);
    ccdVec3Set(&box.pos, .6, 0., 0.5);
    runBench(&box, &cyl, &ccd);
    runBench(&cyl, &box, &ccd);

    ccdVec3Set(&axis, -0.1, 2.2, -1.);
    ccdQuatSetAngleAxis(&cyl.quat, M_PI / 5., &axis);
    ccdVec3Set(&cyl.pos, .6, 0., 0.5);
    ccdVec3Set(&axis, 1., 1., 0.);
    ccdQuatSetAngleAxis(&box.quat, -M_PI / 4., &axis);
    ccdVec3Set(&box.pos, .9, 0.8, 0.5);
    runBench(&box, &cyl, &ccd);
    runBench(&cyl, &box, &ccd);

    fprintf(stdout, "\n----\n\n");
}

int main(int argc, char *argv[])
{
    if (argc > 1){
        cycles = atol(argv[1]);
    }

    fprintf(stdout, "Cycles: %u\n", cycles);
    fprintf(stdout, "\n");

    boxbox();
    cylcyl();
    boxcyl();

    return 0;
}
