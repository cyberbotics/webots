#include "util_MT.h"
#include "objects.h"
#include "collision_kernel.h"
#include "joints/joint.h"
#include "clustering/cluster.h"

dWorldCreateFunction *util_MT::worldFunc = NULL;
dSpaceCreateFunction *util_MT::spaceFunc = NULL;
dSpaceDestroyFunction *util_MT::spaceDestroyFunc = &dSpaceDestroy;
dWorldDestroyFunction *util_MT::worldDestroyFunc = &dWorldDestroy;
dAddGeomToSpaceFunction *util_MT::addGeomToSpaceFunc = &dSpaceAdd;

void util_MT::copyWorldParameters(dWorldID _destWorld, dWorldID _srcWorld)
{
    dWorldSetAngularDamping(_destWorld, dWorldGetAngularDamping(_srcWorld));
    dWorldSetAngularDampingThreshold(_destWorld, dWorldGetAngularDampingThreshold(_srcWorld));
    //dWorldSetAutoDisableAngularAverageThreshold(_destWorld, dWorldGetAutoDisableAngularAverageThreshold(_srcWorld));
    dWorldSetAutoDisableAngularThreshold(_destWorld, dWorldGetAutoDisableAngularThreshold(_srcWorld));
    dWorldSetAutoDisableAverageSamplesCount(_destWorld, dWorldGetAutoDisableAverageSamplesCount(_srcWorld));

    dWorldSetAutoDisableFlag(_destWorld, dWorldGetAutoDisableFlag(_srcWorld));
    //dWorldSetAutoDisableLinearAverageThreshold(_destWorld, dWorldGetAutoDisableLinearAverageThreshold(_srcWorld));
    dWorldSetAutoDisableLinearThreshold(_destWorld, dWorldGetAutoDisableLinearThreshold(_srcWorld));
    dWorldSetAutoDisableSteps(_destWorld, dWorldGetAutoDisableSteps(_srcWorld));
    dWorldSetAutoDisableTime(_destWorld, dWorldGetAutoDisableTime(_srcWorld));

    dWorldSetCFM(_destWorld, dWorldGetCFM(_srcWorld));
    dWorldSetContactMaxCorrectingVel(_destWorld, dWorldGetContactMaxCorrectingVel(_srcWorld));
    dWorldSetContactSurfaceLayer(_destWorld, dWorldGetContactSurfaceLayer(_srcWorld));
    dWorldSetERP(_destWorld, dWorldGetERP(_srcWorld));
    dVector3 gravity;
    dWorldGetGravity(_srcWorld, gravity);
    dWorldSetGravity (_destWorld, gravity[0], gravity[1], gravity[2]);

    dWorldSetLinearDamping(_destWorld, dWorldGetLinearDamping(_srcWorld));
    dWorldSetLinearDampingThreshold(_destWorld, dWorldGetLinearDampingThreshold(_srcWorld));
    dWorldSetMaxAngularSpeed(_destWorld, dWorldGetMaxAngularSpeed(_srcWorld));
    dWorldSetQuickStepNumIterations(_destWorld, dWorldGetQuickStepNumIterations(_srcWorld));
    dWorldSetQuickStepW(_destWorld, dWorldGetQuickStepW(_srcWorld));
}

void util_MT::cleanTags(dxWorld* _world, dxClusteredWorldAndSpace* _cwas)
{
    dUASSERT(_world != NULL, "NULL world passed in to cleanTags!\n");

    for (dxBody *b = _world->firstbody; b; b = (dxBody*) b->next)
    {
        b->mtTag = false;
        if (b->geom) b->geom->tag = false;
        if (b->geom && b->geom->type >= dSimpleSpaceClass)
        {
            dxSpace *gSpace = (dxSpace*) b->geom;
            for (dxGeom *g = gSpace->first; g; g = (dxGeom*) g->next)
            {
                g->tag = false;
                if (g->body) {
                    g->body->mtTag = false;
                    for (dxJointNode *j = g->body->firstjoint; j; j = (dxJointNode*) j->next)
                        j->joint->mtTag = false;
                }
            }
        }
        for (dxJointNode *j = b->firstjoint; j; j = (dxJointNode*) j->next)
                j->joint->mtTag = false;
    }
    for (dxJoint *j = _world->firstjoint; j; j = (dxJoint*) j->next)
    {
        j->mtTag = false;
    }

    _cwas->cleanTags();
}

dReal util_MT::odeMax(dReal val1, dReal val2, dReal val3)
{
    dReal temp = val1 > val2 ? val1 : val2;
    return val3 > temp ? val3 : temp;
}

dReal util_MT::odeMin(dReal val1, dReal val2, dReal val3)
{
    dReal temp = val1 < val2 ? val1 : val2;
    return val3 < temp ? val3 : temp;
}

void util_MT::printSpaceBodies(dxSpace *space)
{
    for (dxGeom *g = space->first; g; g = g->next)
    {
        dUASSERT(g->parent_space == space, "Parent space is invalid!\n");
        printf("geom: %p body:%p type: %d\n", static_cast<void *>(g), static_cast<void *>(g->body), g->type);
    }
    printf("\n");
}

int util_MT::findJointCount(dxBody *body)
{
    if (body == 0) return 0;

    int count = 0;
    for (dxJointNode *j = body->firstjoint; j; j = (dxJointNode*) j->next)
    {
        ++count;
    }
    return count;
}

int util_MT::findJointCount(dxWorld *world)
{
    if (world == 0) return 0;

    int count = 0;
    for (dxJoint *j = world->firstjoint; j; j = (dxJoint*) j->next)
    {
        ++count;
    }
    return count;
}

int util_MT::findBodyCount(dxWorld *world)
{
    if (world == 0) return 0;

    int count = 0;
    for (dxBody *b = world->firstbody; b; b = (dxBody*) b->next)
    {
        ++count;
    }
    return count;
}

static dxBody *bodyList[999];
static int totalBCount = 0;

void util_MT::buildBodyIndices(dWorldID _world)
{
    totalBCount = 0;
    for (dxBody *b = _world->firstbody; b; b = (dxBody*) b->next)
        bodyList[totalBCount++] = b;
}

int util_MT::getBodyIndex(dxBody *_b)
{
    for (int i = 0; i < totalBCount; ++i)
        if (bodyList[i] == _b) return i;

    return -1;
}

void util_MT::printBodyInfo(dxBody *_body)
{
    printf("%d", getBodyIndex(_body));
    for (dxJointNode *jn = _body->firstjoint; jn; jn = jn->next)
    {
        printf(" - %d", getBodyIndex(jn->body));
    }
    printf("\n");
}

void util_MT::printJoints(dxBody *body)
{
    ODE_PRINT ("%p\n|\n", body);
    for (dxJointNode *j = body->firstjoint; j; j = (dxJointNode*) j->next)
    {
        ODE_PRINT ("-%p\n|\n", j->body);
    }
}

void util_MT::printWorldJoints(dxWorld *_world)
{
    int prevBodyID = -1;
    int bodyID1, bodyID2;
    int geom1Type, geom2Type;
    for (dxJoint *j = _world->firstjoint; j; j = (dxJoint*) j->next)
    {
        bodyID1 = getBodyIndex(j->node[0].body);
        bodyID2 = getBodyIndex(j->node[1].body);
        geom1Type = j->node[0].body->geom->type;
        geom2Type = j->node[1].body->geom->type;

        if (prevBodyID == bodyID1)
            printf("-%d(%d)", bodyID2, geom2Type);
        else
            printf("\n%d(%d)-%d(%d)", bodyID1, geom1Type, bodyID2, geom2Type);

        prevBodyID = bodyID2;
    }
}

void util_MT::printAABBList(dxClusterAABB* _aabb)
{
    for (dxClusterAABB* aabb=_aabb; aabb; aabb = aabb->next)
    {
        printf("%p(%d) - ", static_cast<void *>(aabb->geom), aabb->geom->type);
    }
    printf("X\n\n");
}

#ifndef dNODEBUG
void util_MT::checkBodyAndGeomConsistency(int clusterCount, ClusterNode** clusterArray)
{
    //ODE_PRINT("Checking consistency...\n");
    for (int k = 0; k < clusterCount; ++k)
    {
        for (ClusterNode *cluster = clusterArray[k]; cluster; cluster = cluster->next)
        {
            for (dxClusterNode *node = cluster->node; node && node != node->nextFreeNode; node = node->next)
            {
                if (node->aabb->geom) node->aabb->geom->tag = false;
                //node->tag = false;
                node->world = NULL;
                node->space = NULL;
            }
        }
    }

    for (int k = 0; k < clusterCount; ++k)
    {
        for (ClusterNode *cluster = clusterArray[k]; cluster; cluster = cluster->next)
        {
            for (dxClusterNode *node = cluster->node; node && node != node->nextFreeNode; node = node->next)
            {
                if (!node->aabb->geom || node->aabb->geom->tag == false)
                {
                    if (node->aabb->geom)
                      node->aabb->geom->tag = true;
                    if (node->world == NULL)
                    {
                        if (node->aabb->body)
                          node->world = node->aabb->body->world;
                    }
                    if (node->space == NULL && node->aabb->geom)
                    {
                        node->space = node->aabb->geom->parent_space;
                    }
                }
                if (node->aabb->body) dUASSERT(node->world == node->aabb->body->world, "cheeky business 1");
                if (node->aabb->geom) dUASSERT(node->space == node->aabb->geom->parent_space, "cheeky business 2");

            }
        }
    }
}

void util_MT::checkClusterNodes(int clusterCount, dxClusterNode** clusterAABBs)
{
    for (int k = 0; k < clusterCount; ++k)
    {
        dxClusterNode *prevNode = 0x0;
        for (dxClusterNode *node = clusterAABBs[k]; node; node = node->next)
        {
            dUASSERT(node->aabb->clusterID == k, "ClusterID mismatch");
            dUASSERT(node->aabb->node == prevNode, "Parent node of aabb is wrong");
            prevNode = node;
        }
    }
}
#endif
