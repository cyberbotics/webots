//#undef NDEBUG
#include <cu/cu.h>
#include <ccd/polytope.h>
#include <ccd/dbg.h>

TEST(ptSetUp)
{
}

TEST(ptTearDown)
{
}

TEST(ptCreate1)
{
    ccd_pt_t pt;
    ccd_pt_vertex_t *v[3];
    ccd_pt_edge_t *e[3];
    ccd_pt_face_t *f;
    ccd_vec3_t u;
    int res;
    size_t i;

    DBG2("------");

    ccdPtInit(&pt);
    ccdPtDestroy(&pt);

    ccdPtInit(&pt);

    ccdVec3Set(&u, -1., -1., 0.);
    v[0] = ccdPtAddVertexCoords(&pt, -1., -1., 0.);
    assertTrue(ccdVec3Eq(&u, &v[0]->v.v));

    ccdVec3Set(&u, 1., 0., 0.);
    v[1] = ccdPtAddVertexCoords(&pt, 1., 0., 0.);
    assertTrue(ccdVec3Eq(&u, &v[1]->v.v));

    ccdVec3Set(&u, 0., 0., 1.);
    v[2] = ccdPtAddVertexCoords(&pt, 0., 0., 1.);
    assertTrue(ccdVec3Eq(&u, &v[2]->v.v));

    for (i = 0; i < 3; i++){
        assertTrue(ccdEq(v[i]->dist, ccdVec3Len2(&v[i]->v.v)));
    }

    e[0] = ccdPtAddEdge(&pt, v[0], v[1]);
    e[1] = ccdPtAddEdge(&pt, v[1], v[2]);
    e[2] = ccdPtAddEdge(&pt, v[2], v[0]);
    for (i = 0; i < 3; i++){
        DBG("e[%d]->dist: %lf", i, e[i]->dist);
        DBG_VEC3(&e[i]->witness, "     ->witness: ");
    }

    f = ccdPtAddFace(&pt, e[0], e[1], e[2]);
    DBG("f->dist: %lf", f->dist);
    DBG_VEC3(&f->witness, "     ->witness: ");

    for (i = 0; i < 3; i++){
        res = ccdPtDelVertex(&pt, v[i]);
        assertFalse(res == 0);
        res = ccdPtDelEdge(&pt, e[i]);
        assertFalse(res == 0);
    }

    ccdPtDelFace(&pt, f);
    for (i = 0; i < 3; i++){
        res = ccdPtDelVertex(&pt, v[i]);
        assertFalse(res == 0);
    }
    for (i = 0; i < 3; i++){
        res = ccdPtDelEdge(&pt, e[i]);
        assertTrue(res == 0);
    }
    for (i = 0; i < 3; i++){
        res = ccdPtDelVertex(&pt, v[i]);
        assertTrue(res == 0);
    }

    v[0] = ccdPtAddVertexCoords(&pt, -1., -1., 0.);
    v[1] = ccdPtAddVertexCoords(&pt, 1., 0., 0.);
    v[2] = ccdPtAddVertexCoords(&pt, 0., 0., 1.);

    e[0] = ccdPtAddEdge(&pt, v[0], v[1]);
    e[1] = ccdPtAddEdge(&pt, v[1], v[2]);
    e[2] = ccdPtAddEdge(&pt, v[2], v[0]);

    f = ccdPtAddFace(&pt, e[0], e[1], e[2]);

    ccdPtDestroy(&pt);
}

TEST(ptCreate2)
{
    ccd_pt_t pt;
    ccd_pt_vertex_t *v[4];
    ccd_pt_edge_t *e[6];
    ccd_pt_face_t *f[4];
    ccd_vec3_t u;
    int res;
    size_t i;

    DBG2("------");

    ccdPtInit(&pt);

    ccdVec3Set(&u, -1., -1., 0.);
    v[0] = ccdPtAddVertexCoords(&pt, -1., -1., 0.);
    assertTrue(ccdVec3Eq(&u, &v[0]->v.v));

    ccdVec3Set(&u, 1., 0., 0.);
    v[1] = ccdPtAddVertexCoords(&pt, 1., 0., 0.);
    assertTrue(ccdVec3Eq(&u, &v[1]->v.v));

    ccdVec3Set(&u, 0., 0., 1.);
    v[2] = ccdPtAddVertexCoords(&pt, 0., 0., 1.);
    assertTrue(ccdVec3Eq(&u, &v[2]->v.v));

    ccdVec3Set(&u, 0., 1., 0.);
    v[3] = ccdPtAddVertexCoords(&pt, 0., 1., 0.);
    assertTrue(ccdVec3Eq(&u, &v[3]->v.v));

    for (i = 0; i < 4; i++){
        assertTrue(ccdEq(v[i]->dist, ccdVec3Len2(&v[i]->v.v)));
    }
    for (i = 0; i < 4; i++){
        DBG("v[%d]->dist: %lf", i, v[i]->dist);
        DBG_VEC3(&v[i]->witness, "     ->witness: ");
    }

    e[0] = ccdPtAddEdge(&pt, v[0], v[1]);
    e[1] = ccdPtAddEdge(&pt, v[1], v[2]);
    e[2] = ccdPtAddEdge(&pt, v[2], v[0]);
    e[3] = ccdPtAddEdge(&pt, v[3], v[0]);
    e[4] = ccdPtAddEdge(&pt, v[3], v[1]);
    e[5] = ccdPtAddEdge(&pt, v[3], v[2]);
    for (i = 0; i < 6; i++){
        DBG("e[%d]->dist: %lf", i, e[i]->dist);
        DBG_VEC3(&e[i]->witness, "     ->witness: ");
    }

    f[0] = ccdPtAddFace(&pt, e[0], e[1], e[2]);
    f[1] = ccdPtAddFace(&pt, e[3], e[4], e[0]);
    f[2] = ccdPtAddFace(&pt, e[4], e[5], e[1]);
    f[3] = ccdPtAddFace(&pt, e[5], e[3], e[2]);
    for (i = 0; i < 4; i++){
        DBG("f[%d]->dist: %lf", i, f[i]->dist);
        DBG_VEC3(&f[i]->witness, "     ->witness: ");
    }

    for (i = 0; i < 4; i++){
        res = ccdPtDelVertex(&pt, v[i]);
        assertFalse(res == 0);
    }
    for (i = 0; i < 6; i++){
        res = ccdPtDelEdge(&pt, e[i]);
        assertFalse(res == 0);
    }

    res = ccdPtDelFace(&pt, f[0]);
    for (i = 0; i < 6; i++){
        res = ccdPtDelEdge(&pt, e[i]);
        assertFalse(res == 0);
    }

    res = ccdPtDelFace(&pt, f[1]);
    assertTrue(ccdPtDelEdge(&pt, e[0]) == 0);
    assertFalse(ccdPtDelEdge(&pt, e[1]) == 0);
    assertFalse(ccdPtDelEdge(&pt, e[2]) == 0);
    assertFalse(ccdPtDelEdge(&pt, e[3]) == 0);
    assertFalse(ccdPtDelEdge(&pt, e[4]) == 0);
    assertFalse(ccdPtDelEdge(&pt, e[5]) == 0);
    for (i = 0; i < 4; i++){
        res = ccdPtDelVertex(&pt, v[i]);
        assertFalse(res == 0);
    }

    res = ccdPtDelFace(&pt, f[2]);
    assertTrue(ccdPtDelEdge(&pt, e[1]) == 0);
    assertTrue(ccdPtDelEdge(&pt, e[4]) == 0);
    assertFalse(ccdPtDelEdge(&pt, e[2]) == 0);
    assertFalse(ccdPtDelEdge(&pt, e[3]) == 0);
    assertFalse(ccdPtDelEdge(&pt, e[5]) == 0);

    assertTrue(ccdPtDelVertex(&pt, v[1]) == 0);
    assertFalse(ccdPtDelVertex(&pt, v[0]) == 0);
    assertFalse(ccdPtDelVertex(&pt, v[2]) == 0);
    assertFalse(ccdPtDelVertex(&pt, v[3]) == 0);

    res = ccdPtDelFace(&pt, f[3]);
    assertTrue(ccdPtDelEdge(&pt, e[2]) == 0);
    assertTrue(ccdPtDelEdge(&pt, e[3]) == 0);
    assertTrue(ccdPtDelEdge(&pt, e[5]) == 0);

    assertTrue(ccdPtDelVertex(&pt, v[0]) == 0);
    assertTrue(ccdPtDelVertex(&pt, v[2]) == 0);
    assertTrue(ccdPtDelVertex(&pt, v[3]) == 0);

    v[0] = ccdPtAddVertexCoords(&pt, -1., -1., 0.);
    v[1] = ccdPtAddVertexCoords(&pt, 1., 0., 0.);
    v[2] = ccdPtAddVertexCoords(&pt, 0., 0., 1.);
    v[3] = ccdPtAddVertexCoords(&pt, 0., 1., 0.);

    e[0] = ccdPtAddEdge(&pt, v[0], v[1]);
    e[1] = ccdPtAddEdge(&pt, v[1], v[2]);
    e[2] = ccdPtAddEdge(&pt, v[2], v[0]);
    e[3] = ccdPtAddEdge(&pt, v[3], v[0]);
    e[4] = ccdPtAddEdge(&pt, v[3], v[1]);
    e[5] = ccdPtAddEdge(&pt, v[3], v[2]);

    f[0] = ccdPtAddFace(&pt, e[0], e[1], e[2]);
    f[1] = ccdPtAddFace(&pt, e[3], e[4], e[0]);
    f[2] = ccdPtAddFace(&pt, e[4], e[5], e[1]);
    f[3] = ccdPtAddFace(&pt, e[5], e[3], e[2]);

    ccdPtDestroy(&pt);
}

TEST(ptNearest)
{
    ccd_pt_t pt;
    ccd_pt_vertex_t *v[4];
    ccd_pt_edge_t *e[6];
    ccd_pt_face_t *f[4];
    ccd_pt_el_t *nearest;

    DBG2("------");

    ccdPtInit(&pt);

    v[0] = ccdPtAddVertexCoords(&pt, -1., -1., 0.);
    v[1] = ccdPtAddVertexCoords(&pt, 1., 0., 0.);
    v[2] = ccdPtAddVertexCoords(&pt, 0., 0., 1.);
    v[3] = ccdPtAddVertexCoords(&pt, 0., 1., 0.);

    e[0] = ccdPtAddEdge(&pt, v[0], v[1]);
    e[1] = ccdPtAddEdge(&pt, v[1], v[2]);
    e[2] = ccdPtAddEdge(&pt, v[2], v[0]);
    e[3] = ccdPtAddEdge(&pt, v[3], v[0]);
    e[4] = ccdPtAddEdge(&pt, v[3], v[1]);
    e[5] = ccdPtAddEdge(&pt, v[3], v[2]);

    f[0] = ccdPtAddFace(&pt, e[0], e[1], e[2]);
    f[1] = ccdPtAddFace(&pt, e[3], e[4], e[0]);
    f[2] = ccdPtAddFace(&pt, e[4], e[5], e[1]);
    f[3] = ccdPtAddFace(&pt, e[5], e[3], e[2]);

    nearest = ccdPtNearest(&pt);
    //DBG("nearest->type: %d", nearest->type);
    //DBG("       ->dist: %lf", nearest->dist);
    //DBG_VEC3(&nearest->witness, "       ->witness: ");
    assertEquals(nearest->type, CCD_PT_FACE);
    assertEquals(nearest, (ccd_pt_el_t *)f[1]);
    assertTrue(ccdPtDelFace(&pt, f[1]) == 0);

    nearest = ccdPtNearest(&pt);
    //DBG("nearest->type: %d", nearest->type);
    //DBG("       ->dist: %lf", nearest->dist);
    //DBG_VEC3(&nearest->witness, "       ->witness: ");
    assertEquals(nearest->type, CCD_PT_FACE);
    assertTrue(nearest == (ccd_pt_el_t *)f[0]
                || nearest == (ccd_pt_el_t *)f[3]);
    assertTrue(ccdPtDelFace(&pt, (ccd_pt_face_t *)nearest) == 0);

    nearest = ccdPtNearest(&pt);
    //DBG("nearest->type: %d", nearest->type);
    //DBG("       ->dist: %lf", nearest->dist);
    //DBG_VEC3(&nearest->witness, "       ->witness: ");
    assertEquals(nearest->type, CCD_PT_FACE);
    assertTrue(nearest == (ccd_pt_el_t *)f[0]
                || nearest == (ccd_pt_el_t *)f[3]);
    assertTrue(ccdPtDelFace(&pt, (ccd_pt_face_t *)nearest) == 0);

    nearest = ccdPtNearest(&pt);
    //DBG("nearest->type: %d", nearest->type);
    //DBG("       ->dist: %lf", nearest->dist);
    //DBG_VEC3(&nearest->witness, "       ->witness: ");
    assertEquals(nearest->type, CCD_PT_EDGE);
    assertTrue(nearest == (ccd_pt_el_t *)e[0]
                || nearest == (ccd_pt_el_t *)e[3]);
    assertTrue(ccdPtDelEdge(&pt, (ccd_pt_edge_t *)nearest) == 0);

    nearest = ccdPtNearest(&pt);
    //DBG("nearest->type: %d", nearest->type);
    //DBG("       ->dist: %lf", nearest->dist);
    //DBG_VEC3(&nearest->witness, "       ->witness: ");
    assertEquals(nearest->type, CCD_PT_EDGE);
    assertTrue(nearest == (ccd_pt_el_t *)e[0]
                || nearest == (ccd_pt_el_t *)e[3]);
    assertTrue(ccdPtDelEdge(&pt, (ccd_pt_edge_t *)nearest) == 0);

    nearest = ccdPtNearest(&pt);
    //DBG("nearest->type: %d", nearest->type);
    //DBG("       ->dist: %lf", nearest->dist);
    //DBG_VEC3(&nearest->witness, "       ->witness: ");
    assertEquals(nearest->type, CCD_PT_FACE);
    assertEquals(nearest, (ccd_pt_el_t *)f[2]);
    assertTrue(ccdPtDelFace(&pt, f[2]) == 0);

    nearest = ccdPtNearest(&pt);
    //DBG("nearest->type: %d", nearest->type);
    //DBG("       ->dist: %lf", nearest->dist);
    //DBG_VEC3(&nearest->witness, "       ->witness: ");
    assertEquals(nearest->type, CCD_PT_EDGE);
    assertTrue(nearest == (ccd_pt_el_t *)e[1]
                || nearest == (ccd_pt_el_t *)e[4]
                || nearest == (ccd_pt_el_t *)e[5]);
    assertTrue(ccdPtDelEdge(&pt, (ccd_pt_edge_t *)nearest) == 0);

    nearest = ccdPtNearest(&pt);
    //DBG("nearest->type: %d", nearest->type);
    //DBG("       ->dist: %lf", nearest->dist);
    //DBG_VEC3(&nearest->witness, "       ->witness: ");
    assertEquals(nearest->type, CCD_PT_EDGE);
    assertTrue(nearest == (ccd_pt_el_t *)e[1]
                || nearest == (ccd_pt_el_t *)e[4]
                || nearest == (ccd_pt_el_t *)e[5]);
    assertTrue(ccdPtDelEdge(&pt, (ccd_pt_edge_t *)nearest) == 0);

    nearest = ccdPtNearest(&pt);
    //DBG("nearest->type: %d", nearest->type);
    //DBG("       ->dist: %lf", nearest->dist);
    //DBG_VEC3(&nearest->witness, "       ->witness: ");
    assertEquals(nearest->type, CCD_PT_EDGE);
    assertTrue(nearest == (ccd_pt_el_t *)e[1]
                || nearest == (ccd_pt_el_t *)e[4]
                || nearest == (ccd_pt_el_t *)e[5]);
    assertTrue(ccdPtDelEdge(&pt, (ccd_pt_edge_t *)nearest) == 0);

    nearest = ccdPtNearest(&pt);
    //DBG("nearest->type: %d", nearest->type);
    //DBG("       ->dist: %lf", nearest->dist);
    //DBG_VEC3(&nearest->witness, "       ->witness: ");
    assertEquals(nearest->type, CCD_PT_EDGE);
    assertTrue(nearest == (ccd_pt_el_t *)e[2]);
    assertTrue(ccdPtDelEdge(&pt, (ccd_pt_edge_t *)nearest) == 0);

    nearest = ccdPtNearest(&pt);
    //DBG("nearest->type: %d", nearest->type);
    //DBG("       ->dist: %lf", nearest->dist);
    //DBG_VEC3(&nearest->witness, "       ->witness: ");
    assertEquals(nearest->type, CCD_PT_VERTEX);
    assertTrue(nearest == (ccd_pt_el_t *)v[1]
                || nearest == (ccd_pt_el_t *)v[2]
                || nearest == (ccd_pt_el_t *)v[3]);
    assertTrue(ccdPtDelVertex(&pt, (ccd_pt_vertex_t *)nearest) == 0);

    nearest = ccdPtNearest(&pt);
    //DBG("nearest->type: %d", nearest->type);
    //DBG("       ->dist: %lf", nearest->dist);
    //DBG_VEC3(&nearest->witness, "       ->witness: ");
    assertEquals(nearest->type, CCD_PT_VERTEX);
    assertTrue(nearest == (ccd_pt_el_t *)v[1]
                || nearest == (ccd_pt_el_t *)v[2]
                || nearest == (ccd_pt_el_t *)v[3]);
    assertTrue(ccdPtDelVertex(&pt, (ccd_pt_vertex_t *)nearest) == 0);

    nearest = ccdPtNearest(&pt);
    //DBG("nearest->type: %d", nearest->type);
    //DBG("       ->dist: %lf", nearest->dist);
    //DBG_VEC3(&nearest->witness, "       ->witness: ");
    assertEquals(nearest->type, CCD_PT_VERTEX);
    assertTrue(nearest == (ccd_pt_el_t *)v[1]
                || nearest == (ccd_pt_el_t *)v[2]
                || nearest == (ccd_pt_el_t *)v[3]);
    assertTrue(ccdPtDelVertex(&pt, (ccd_pt_vertex_t *)nearest) == 0);

    nearest = ccdPtNearest(&pt);
    //DBG("nearest->type: %d", nearest->type);
    //DBG("       ->dist: %lf", nearest->dist);
    //DBG_VEC3(&nearest->witness, "       ->witness: ");
    assertEquals(nearest->type, CCD_PT_VERTEX);
    assertTrue(nearest == (ccd_pt_el_t *)v[0]);
    assertTrue(ccdPtDelVertex(&pt, (ccd_pt_vertex_t *)nearest) == 0);

    nearest = ccdPtNearest(&pt);
    assertTrue(nearest == NULL);

    ccdPtDestroy(&pt);
}
