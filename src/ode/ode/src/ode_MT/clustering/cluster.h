#ifndef _ODE_MT_CLUSTER_H_
#define _ODE_MT_CLUSTER_H_

#include <ode/ode_MT.h>
#include "ode/collision_trimesh.h"
#include "objects.h"
#include "array.h"
#include <map>
#include "ode_MT/util_MT.h"

typedef void clusterChangeCallbackFunc();

template <class T> class dxMaintainedArray : public dArray<T> {
public:
  dxMaintainedArray() { dArray<T>(); }
  void setSize (int newsize)
  {
      int sizeofT = sizeof(T);
      int oldanum = dArray<T>::_anum;
      dArray<T>::_setSize(newsize, sizeofT);
      int newanum = dArray<T>::_anum;
      memset((char*)dArray<T>::_data + oldanum*sizeofT, 0x0, (newanum-oldanum)*sizeofT);
  }
  T & operator[] (int i)
  {
      if (i >= dArray<T>::_size)
        setSize((i+1)*2);

      return ((T*)dArray<T>::_data)[i];
  }
};

class dxClusteredWorldAndSpace
{
public:
    dxClusterNode **table;
    float xmin, xmax, ymin, ymax, zmin, zmax;
    float xsizemin, xsizemax, ysizemin, ysizemax, zsizemin, zsizemax;
    float xsizeavg, ysizeavg, zsizeavg;

    float hsAxis[3];
    int sz, algoN, entityCount;
    float gridstep;
    float gridsize;
    int xsteps, ysteps, zsteps;

    unsigned char *tested;
    int tested_rowsize;

    int tracker;
    int oldN, oldHashSpaceSize;

    typedef std::map<dxGeom*, dxGeom*> geomMap;
    std::map<int, geomMap> geomDuplicationMap;
    std::map<dxGeom*, int> staticGeomClusterIDMap;

    dxMaintainedArray<ClusterNode*> clusterArray;
    dxMaintainedArray<int> activeClusters;
    dxMaintainedArray<int> deactivatedClusters;

    dxMaintainedArray<dxClusterNode*> clusterAABBs;
    dxMaintainedArray<dxClusterNode**> clusterTables;

    bool bCalculateNewSZ;

    clusterChangeCallbackFunc* clusterChangeCallback;

    dxMaintainedArray<bool> clusterStaticTypes;
    typedef std::map<int, bool> intBoolMap;
    typedef dxMaintainedArray<bool> boolArray;
    dxMaintainedArray<boolArray> staticClusterLinks;

public:
    dSpaceID originalSpace;
    dxMaintainedArray<dSpaceID> spaces;

    dWorldID originalWorld;
    dxMaintainedArray<dWorldID> worlds;

    dxClusterAABB *first_aabb;	// list of AABBs in hash table
    dxClusterAABB *big_boxes;	// list of AABBs too big for hash table
    int bigboxCount;

    int activeClusterCount;
    int deactivatedClusterCount;
    int clusterCount;

private:
    void createNewWorldAndSpace(dWorldID&, dSpaceID&);
    dxGeom *retrieveBodySpaceAndFlagInnerGeoms(dxBody *body);
    void createAABB(dxBody *body, dxGeom *geom, dxClusterAABB *&_firstaabb, dxClusterAABB *&_bigboxes, bool isbigbox);
    void makeClusters();
    void attachJoint(int &jointCount, dxJoint *j, dxWorld *oldWorld, dxWorld *newWorld);
    void reattachJoints(int &bodyCount, int &geomCount, int &jointCount, dxBody *body, dxGeom *geom, dxWorld *oldWorld, dxSpace *oldSpace, dxWorld *newWorld, dxSpace *newSpace, bool bTreatSpaces = false);
    void duplicateBodiesAndGeoms(int &bodyCount, int &geomCount, int &jointCount, dxBody* body, dxGeom* geom, dxWorld *oldWorld, dxSpace *oldSpace, dxWorld *newWorld, dxSpace *newSpace, int kid);
    dxGeom* duplicateBodyAndGeom(int &bodyCount, int &geomCount, dxBody* body, dxGeom* geom, dxWorld *oldWorld, dxSpace *oldSpace, dxWorld *newWorld, dxSpace *newSpace, int kid);
    void duplicateJoint(int &jointCount, dxJoint *j, dxWorld *oldWorld, dxWorld *newWorld);
    void initAABBs(dxWorld *_world, dxSpace *_space, dxClusterAABB *&_dynamicList, dxClusterAABB *&_staticList);
    void crossOver(int smallerCluster, int biggerCluster, int cellID);
    void crossOverClusters(int kid1, int kid2, int cellId);
    void checkDynamicClusterConsistencies();
    void addTrackerToNode(int &bCount, int &gCount, int &jCount, int kid, int xi, int yi, int zi, int tracker);
    void transferNodeBetweenClusters(dxClusterNode *node, int oldClusterID, int newClusterID, bool bRefreshActiveClusters = true);
    void attachBodyAndGeom(int &bodyCount, int &geomCount, dxBody *body, dxGeom *geom, dxWorld *oldWorld, dxSpace *oldSpace, dxWorld *newWorld, dxSpace *newSpace);
    void checkHashSpaceConsistency();
    void assignClustersToWorlds();
    int getSpaceClusterID(dSpaceID);
    int getWorldClusterID(dxWorld *_world);
    void cleanupTable(dxClusterNode **&_table);
    unsigned long getVirtualAddress (dVector3 _pos);
    void setMinMax(dReal *bounds);
    void findNeighboringCluster(dxClusterNode *_node, ClusterNode *&_cluster, ClusterNode *&_tail);
    void cleanupMemory();
    void updateAABBs(dxClusterAABB *_dynamicList, dxClusterAABB *_staticList);
    void calculateGridStep();
    void recalculateActiveClusters();
    void addClusterNodeToClusterTable(dxClusterNode *node, ClusterNode *&_tail, bool _isStatic);
    void addClusterNodeToCluster(dxClusterNode *_node, ClusterNode *&_cluster, ClusterNode *&_tail, bool _bGetAllNodesInCell);
    bool isGeomBig(dxGeom* geom);
    bool isGeomStatic(dxGeom* geom);
    bool isGeomBodyless(dxGeom* geom);
    void flagDynamicGeoms(dSpaceID _space);

public:
    dxClusteredWorldAndSpace(clusterChangeCallbackFunc*);
    ~dxClusteredWorldAndSpace();

    void checkClusterConsistency(int kid);
    void update(bool);
    bool updateClusterAABBsAndTable(int kid);
    void recombineClusters();

    void cleanTags();

    int getGeomCount() { return entityCount; }
    int getActiveClusterCount() { return activeClusterCount; }
    int getClusterCount() { return clusterCount; }
    ClusterNode* getClusterNodeList(int clusterId) { return clusterArray[clusterId]; }
    float getGridSize() { return gridsize; }
    void  setGridSize(float _val) { gridsize = _val; bCalculateNewSZ = true; }
    std::map<int, geomMap>& getGeomDuplicationMap() { return geomDuplicationMap; }

    void propagateGeomPosition(dxGeom *g, dReal x, dReal y, dReal z, dGeomSetPositionFunction* _func);
    void propagateGeomRotation(dxGeom *g, const dMatrix3 R, dGeomSetRotationFunction* _func);
    void propagateGeomQuaternion(dxGeom *g, const dQuaternion quat, dGeomSetQuaternionFunction* _func);
    void propagateGeomTransformUpdate(dxGeom *g);
    void propagateGeomBoxSetLengths(dxGeom *g, dReal lx, dReal ly, dReal lz, dGeomBoxSetLengthsFunction* _func);
    void propagateGeomCapsuleSetParams(dxGeom *g, dReal radius, dReal length, dGeomCapsuleSetParamsFunction* _func);
    void propagateGeomCylinderSetParams(dxGeom *g, dReal radius, dReal length, dGeomCylinderSetParamsFunction* _func);
    void propagateGeomTriMeshSetData(dxGeom *g, dTriMeshDataID data, dGeomTriMeshSetDataFunction* _func);
    void propagateGeomSphereSetRadius(dxGeom *g, dReal radius, dGeomSphereSetRadiusFunction* _func);
    void propagatePlaneParams(dxGeom *g, dReal a, dReal b, dReal c, dReal d, dGeomPlaneSetParamsFunction* _func);
    void propagateOffsetChange(dxGeom *g);
};

#endif
