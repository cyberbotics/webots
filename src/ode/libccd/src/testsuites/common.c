#include "common.h"
#include <ccd/vec3.h>
#include <ccd/quat.h>
#include "support.h"

static void svtCyl(ccd_cyl_t *c, FILE *out, const char *color, const char *name)
{
    ccd_vec3_t v[32];
    ccd_quat_t rot;
    ccd_vec3_t axis, vpos, vpos2;
    ccd_real_t angle, x, y;
    size_t i;

    ccdVec3Set(&axis, 0., 0., 1.);
    ccdVec3Set(&vpos, 0., c->radius, 0.);
    angle = 0.;
    for (i = 0; i < 16; i++){
        angle = (ccd_real_t)i * (2. * M_PI / 16.);

        ccdQuatSetAngleAxis(&rot, angle, &axis);
        ccdVec3Copy(&vpos2, &vpos);
        ccdQuatRotVec(&vpos2, &rot);
        x = ccdVec3X(&vpos2);
        y = ccdVec3Y(&vpos2);

        ccdVec3Set(&v[i], x, y, c->height / 2.);
        ccdVec3Set(&v[i + 16], x, y, -c->height / 2.);
    }

    for (i = 0; i < 32; i++){
        ccdQuatRotVec(&v[i], &c->quat);
        ccdVec3Add(&v[i], &c->pos);
    }

    fprintf(out, "-----\n");
    if (name)
        fprintf(out, "Name: %s\n", name);

    fprintf(out, "Face color: %s\n", color);
    fprintf(out, "Edge color: %s\n", color);
    fprintf(out, "Point color: %s\n", color);
    fprintf(out, "Points:\n");
    for (i = 0; i < 32; i++){
        fprintf(out, "%lf %lf %lf\n", ccdVec3X(&v[i]), ccdVec3Y(&v[i]), ccdVec3Z(&v[i]));
    }

    fprintf(out, "Edges:\n");
    fprintf(out, "0 16\n");
    fprintf(out, "0 31\n");
    for (i = 1; i < 16; i++){
        fprintf(out, "0 %d\n", i);
        fprintf(out, "16 %d\n", i + 16);
        if (i != 0){
            fprintf(out, "%d %d\n", i - 1, i);
            fprintf(out, "%d %d\n", i + 16 - 1, i + 16);
        }

        fprintf(out, "%d %d\n", i, i + 16);
        fprintf(out, "%d %d\n", i, i + 16 - 1);
    }

    fprintf(out, "Faces:\n");
    for (i = 2; i < 16; i++){
        fprintf(out, "0 %d %d\n", i, i -1);
        fprintf(out, "16 %d %d\n", i + 16, i + 16 -1);

    }
    fprintf(out, "0 16 31\n");
    fprintf(out, "0 31 15\n");
    for (i = 1; i < 16; i++){
        fprintf(out, "%d %d %d\n", i, i + 16, i + 16 - 1);
        fprintf(out, "%d %d %d\n", i, i + 16 - 1, i - 1);
    }
    fprintf(out, "-----\n");
}

static void svtBox(ccd_box_t *b, FILE *out, const char *color, const char *name)
{
    ccd_vec3_t v[8];
    size_t i;

    ccdVec3Set(&v[0], b->x * 0.5, b->y * 0.5, b->z * 0.5);
    ccdVec3Set(&v[1], b->x * 0.5, b->y * -0.5, b->z * 0.5);
    ccdVec3Set(&v[2], b->x * 0.5, b->y * 0.5, b->z * -0.5);
    ccdVec3Set(&v[3], b->x * 0.5, b->y * -0.5, b->z * -0.5);
    ccdVec3Set(&v[4], b->x * -0.5, b->y * 0.5, b->z * 0.5);
    ccdVec3Set(&v[5], b->x * -0.5, b->y * -0.5, b->z * 0.5);
    ccdVec3Set(&v[6], b->x * -0.5, b->y * 0.5, b->z * -0.5);
    ccdVec3Set(&v[7], b->x * -0.5, b->y * -0.5, b->z * -0.5);

    for (i = 0; i < 8; i++){
        ccdQuatRotVec(&v[i], &b->quat);
        ccdVec3Add(&v[i], &b->pos);
    }

    fprintf(out, "-----\n");
    if (name)
        fprintf(out, "Name: %s\n", name);
    fprintf(out, "Face color: %s\n", color);
    fprintf(out, "Edge color: %s\n", color);
    fprintf(out, "Point color: %s\n", color);
    fprintf(out, "Points:\n");
    for (i = 0; i < 8; i++){
        fprintf(out, "%lf %lf %lf\n", ccdVec3X(&v[i]), ccdVec3Y(&v[i]), ccdVec3Z(&v[i]));
    }

    fprintf(out, "Edges:\n");
    fprintf(out, "0 1\n 0 2\n2 3\n3 1\n1 2\n6 2\n1 7\n1 5\n");
    fprintf(out, "5 0\n0 4\n4 2\n6 4\n6 5\n5 7\n6 7\n7 2\n7 3\n4 5\n");

    fprintf(out, "Faces:\n");
    fprintf(out, "0 2 1\n1 2 3\n6 2 4\n4 2 0\n4 0 5\n5 0 1\n");
    fprintf(out, "5 1 7\n7 1 3\n6 4 5\n6 5 7\n2 6 7\n2 7 3\n");
    fprintf(out, "-----\n");
}

void svtObj(void *_o, FILE *out, const char *color, const char *name)
{
    ccd_obj_t *o = (ccd_obj_t *)_o;

    if (o->type == CCD_OBJ_CYL){
        svtCyl((ccd_cyl_t *)o, out, color, name);
    }else if (o->type == CCD_OBJ_BOX){
        svtBox((ccd_box_t *)o, out, color, name);
    }
}

void svtObjPen(void *o1, void *o2,
               FILE *out, const char *name,
               ccd_real_t depth, const ccd_vec3_t *dir, const ccd_vec3_t *pos)
{
    ccd_vec3_t sep;
    char oname[500];

    ccdVec3Copy(&sep, dir);
    ccdVec3Scale(&sep, depth);
    ccdVec3Add(&sep, pos);

    fprintf(out, "------\n");
    if (name)
        fprintf(out, "Name: %s\n", name);
    fprintf(out, "Point color: 0.1 0.1 0.9\n");
    fprintf(out, "Points:\n%lf %lf %lf\n", ccdVec3X(pos), ccdVec3Y(pos), ccdVec3Z(pos));
    fprintf(out, "------\n");
    fprintf(out, "Point color: 0.1 0.9 0.9\n");
    fprintf(out, "Edge color: 0.1 0.9 0.9\n");
    fprintf(out, "Points:\n%lf %lf %lf\n", ccdVec3X(pos), ccdVec3Y(pos), ccdVec3Z(pos));
    fprintf(out, "%lf %lf %lf\n", ccdVec3X(&sep), ccdVec3Y(&sep), ccdVec3Z(&sep));
    fprintf(out, "Edges: 0 1\n");

    oname[0] = 0x0;
    if (name)
        sprintf(oname, "%s o1", name);
    svtObj(o1, out, "0.9 0.1 0.1", oname);

    oname[0] = 0x0;
    if (name)
        sprintf(oname, "%s o1", name);
    svtObj(o2, out, "0.1 0.9 0.1", oname);
}

void recPen(ccd_real_t depth, const ccd_vec3_t *dir, const ccd_vec3_t *pos,
            FILE *out, const char *note)
{
    if (!note)
        note = "";

    fprintf(out, "# %s: depth: %lf\n", note, depth);
    fprintf(out, "# %s: dir:   [%lf %lf %lf]\n", note, ccdVec3X(dir), ccdVec3Y(dir), ccdVec3Z(dir));
    fprintf(out, "# %s: pos:   [%lf %lf %lf]\n", note, ccdVec3X(pos), ccdVec3Y(pos), ccdVec3Z(pos));
    fprintf(out, "#\n");
}
