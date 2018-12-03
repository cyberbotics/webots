#ifndef _ODE_ODE_MT_H_
#define _ODE_ODE_MT_H_

#include <ode/common.h>

#ifdef __cplusplus
extern "C" {
#endif

// an axis aligned bounding box in the hash table
struct dxClusterAABB {
  dxClusterAABB *next; // next in the list of all AABBs (list not related to clusters)
  int level;		// the level this is stored in (cell size = 2^level)
  int dbounds[6];	// AABB bounds, discretized to cell size
  dGeomID geom;		// corresponding geometry object (AABB stored here)
  dBodyID body;     // body to which this geom is attached
  int index;		// index of this AABB, starting from 0
  float cellsize;
  bool tag;

  // tracker is used to split nodes in different clusters (declustering).
  // While traversing the hashspace cells looking for crossover conditions,
  // we mark the cells with a tracker in a fashion similar to the original cluster creation
  // algorithm, i.e. neighboring hashspace cells containing geoms are marked with the same id.
  // If we detect that the tracker id needs to be incremented within the same cluster, this is a
  // sufficient condition for declustering.
  int tracker;

  struct ClusterNode *cluster;
  int clusterID;

  // Pointer to cluster AABB which contains this dxClusterAABB
  // i.e., clusterNode->next->aabb->node == clusterNode
  struct dxClusterNode *node;
};

struct dxClusterNode {
  dxClusterAABB *aabb;		// axis aligned bounding box that intersects this cell
  int x,y,z;		// cell position in space, discretized to cell size
  int count;
  struct ClusterNode *cluster;
  bool tagged;
  dxClusterNode *next;		// next node in hash table collision list, NULL if none
  dxClusterNode *nextFreeNode;
  bool isStatic;

  // For debugging purposes
  dWorldID world;
  dSpaceID space;
};

struct ClusterNode {
  dxClusterNode *node;
  dVector3 color;
  bool tagged;
  int count;
  ClusterNode *next;
};

ODE_API int dClusterGetCount(dWorldID _world, dSpaceID _space);
ODE_API float dClusterGetGridStep(dWorldID _world, dSpaceID _space);
ODE_API void dClusterGetCenter(dWorldID _world, dSpaceID _space, float &_x, float &_y, float &_z);
ODE_API dxClusterNode** dClusterGetClusterAABBs(dWorldID _world, dSpaceID _space);
ODE_API int dThreadGetSpacesCount(int threadID);
ODE_API dSpaceID dThreadGetSpaceID(int threadID, int index);

#ifdef __cplusplus
}
#endif

#endif
