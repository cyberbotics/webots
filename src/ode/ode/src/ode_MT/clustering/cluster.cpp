#include "cluster.h"
#include "collision_kernel.h"
#include "joints/joints.h"
#include "ode/objects.h"
#include "util.h"
#include "ode/odemath.h"
#include "collision_std.h"
#include "heightfield.h"
#include "collision_trimesh_internal.h"

// Externally defined functions
extern void addObjectToList( dObject *obj, dObject **first );
extern void removeObjectFromList(dObject *obj);
extern void checkWorld(dxWorld *_world);

#undef ALLOCA
#undef DEALLOCA
#define ALLOCA(x) malloc(x) //dALLOCA16(x)
#define DEALLOCA(x) free(x)

#include <pthread.h>
static pthread_mutex_t geomDuplicationMapMutex; // used to lock geomDuplicationMap assignment

void dxClusteredWorldAndSpace::cleanupTable(dxClusterNode **&_table)
{
    if (_table == NULL) return;

    for (int i = 0; i < sz; ++i)
    {
        dxClusterNode *node = _table[i];
        dxClusterNode *temp;
        while (node)
        {
            temp = node->next;
            DEALLOCA(node);
            node = temp;
        }
    }

    DEALLOCA(_table);
    _table = NULL;
}

// Note: this function is possibly called in a multithread way
unsigned long dxClusteredWorldAndSpace::getVirtualAddress (dVector3 _pos)
{
  const float x = xsteps / 2.0f + _pos[0];
  const float y = ysteps / 2.0f + _pos[1];
  const float z = zsteps / 2.0f + _pos[2];

  unsigned long retVal = (x * ysteps * zsteps) + (y * zsteps) + (z);

  if (retVal >= static_cast<unsigned long>(sz))
  {
      ODE_IMPORTANT("OUT OF BOUNDS! Retval: %ld, sz: %d, pos: (%f, %f, %f)\n", retVal, sz, _pos[0], _pos[1], _pos[2]);
      bCalculateNewSZ = true;
      return sz - 1;
  }

  return retVal;
}

void dxClusteredWorldAndSpace::setMinMax(dReal *bounds)
{
    if (bounds[0] < xmin) xmin = bounds[0];
    if (bounds[1] > xmax) xmax = bounds[1];
    if (bounds[2] < ymin) ymin = bounds[2];
    if (bounds[3] > ymax) ymax = bounds[3];
    if (bounds[4] < zmin) zmin = bounds[4];
    if (bounds[5] > zmax) zmax = bounds[5];

    const dReal xsize = (bounds[1] - bounds[0]);
    const dReal ysize = (bounds[3] - bounds[2]);
    const dReal zsize = (bounds[5] - bounds[4]);

    if (xsize < xsizemin) xsizemin = xsize;
    if (xsize > xsizemax) xsizemax = xsize;
    if (ysize < ysizemin) ysizemin = ysize;
    if (ysize > ysizemax) ysizemax = ysize;
    if (zsize < zsizemin) zsizemin = zsize;
    if (zsize > zsizemax) zsizemax = zsize;

    xsizeavg = (xsizeavg * algoN + xsize) / (float) (algoN + 1);
    ysizeavg = (ysizeavg * algoN + ysize) / (float) (algoN + 1);
    zsizeavg = (zsizeavg * algoN + zsize) / (float) (algoN + 1);
}

void dxClusteredWorldAndSpace::createNewWorldAndSpace(dWorldID &_newWorld, dSpaceID &_newSpace)
{
    _newWorld = util_MT::worldFunc();
    _newSpace = util_MT::spaceFunc(0);

    dSpaceSetCleanup(_newSpace, false);

    util_MT::copyWorldParameters(_newWorld, originalWorld);

}

void dxClusteredWorldAndSpace::addClusterNodeToClusterTable(dxClusterNode *node, ClusterNode *&_tail, bool _isStatic)
{
  if (node->aabb->cluster == NULL)
  {
      node->aabb->cluster = _tail;
      node->aabb->clusterID = clusterCount;

      // assign body/geom to appropriate cluster AABB
      dxClusterNode *newNode = (dxClusterNode*) ALLOCA (sizeof(dxClusterNode));
      newNode->aabb = node->aabb;
      newNode->aabb->node = NULL;
      newNode->x = newNode->y = newNode->z = -1;
      newNode->nextFreeNode = NULL;
      newNode->cluster = NULL;
      newNode->world = NULL;
      newNode->space = NULL;
      newNode->tagged = false;
      newNode->isStatic = _isStatic;
      // update parent node for AABB. used for crossover clustering
      if (clusterAABBs[clusterCount]) clusterAABBs[clusterCount]->aabb->node = newNode;

      if (clusterAABBs[clusterCount]) newNode->count = clusterAABBs[clusterCount]->count + 1;
      else newNode->count = 1;

      newNode->next = clusterAABBs[clusterCount];
      clusterAABBs[clusterCount] = newNode;
  }
}

void dxClusteredWorldAndSpace::addClusterNodeToCluster(dxClusterNode *_node, ClusterNode *&_cluster, ClusterNode *&_tail, bool _bGetAllNodesInCell)
{
    dUASSERT(_node != NULL, "null node passed into findNeighboringCluster!");

    _node->tagged = true;

    // Create new cluster node and add to head of cluster linked list
    ClusterNode *newCluster = (ClusterNode*) ALLOCA (sizeof (ClusterNode));
    newCluster->next = NULL;
    newCluster->node = NULL;
    if (_cluster) newCluster->count = _cluster->count + 1;
    else newCluster->count = 1;
    newCluster->node = _node;

    // attach new cluster to head of clusterchain
    newCluster->next = _cluster;
    _cluster = newCluster;

    if (_tail == NULL)
    {
        _tail = newCluster;
    }

    if (!_bGetAllNodesInCell)
        addClusterNodeToClusterTable(_node, _tail, true);
    else
    {
        // assign all geoms in current clustercell to the same cluster
        for (dxClusterNode *node=_node; node && node != _node->nextFreeNode; node = node->next)
            addClusterNodeToClusterTable(node, _tail, false);
    }
}

void dxClusteredWorldAndSpace::findNeighboringCluster(dxClusterNode *_node, ClusterNode *&_cluster, ClusterNode *&_tail)
{
    addClusterNodeToCluster(_node, _cluster, _tail, true);

    // go through neighboring cells and add to clusters too
    unsigned long hi;
    dVector3 newpos1 = { (dReal)_node->x + 1, (dReal)_node->y, (dReal)_node->z};
    hi = getVirtualAddress (newpos1) % sz;
    if (table[hi] && table[hi]->tagged == false && table[hi]->nextFreeNode != table[hi]) findNeighboringCluster(table[hi], _cluster, _tail);

    dVector3 newpos2 = { (dReal)_node->x - 1, (dReal)_node->y, (dReal)_node->z};
    hi = getVirtualAddress (newpos2) % sz;
    if (table[hi] && table[hi]->tagged == false && table[hi]->nextFreeNode != table[hi]) findNeighboringCluster(table[hi], _cluster, _tail);

    dVector3 newpos3 = { (dReal)_node->x, (dReal)_node->y + 1, (dReal)_node->z};
    hi = getVirtualAddress (newpos3) % sz;
    if (table[hi] && table[hi]->tagged == false && table[hi]->nextFreeNode != table[hi]) findNeighboringCluster(table[hi], _cluster, _tail);

    dVector3 newpos4 = { (dReal)_node->x, (dReal)_node->y - 1, (dReal)_node->z};
    hi = getVirtualAddress (newpos4) % sz;
    if (table[hi] && table[hi]->tagged == false && table[hi]->nextFreeNode != table[hi]) findNeighboringCluster(table[hi], _cluster, _tail);

    dVector3 newpos5 = { (dReal)_node->x, (dReal)_node->y, (dReal)_node->z + 1};
    hi = getVirtualAddress (newpos5) % sz;
    if (table[hi] && table[hi]->tagged == false && table[hi]->nextFreeNode != table[hi]) findNeighboringCluster(table[hi], _cluster, _tail);

    dVector3 newpos6 = { (dReal)_node->x, (dReal)_node->y, (dReal)_node->z - 1};
    hi = getVirtualAddress (newpos6) % sz;
    if (table[hi] && table[hi]->tagged == false && table[hi]->nextFreeNode != table[hi]) findNeighboringCluster(table[hi], _cluster, _tail);
}

void dxClusteredWorldAndSpace::cleanupMemory()
{
      dxClusterAABB *aabb = first_aabb;
      dxClusterAABB *temp;
      while (aabb)
      {
          temp = aabb->next;
          DEALLOCA(aabb);
          aabb = temp;
      }
      first_aabb = NULL;

      aabb = big_boxes;
      while (aabb)
      {
          temp = aabb->next;
          DEALLOCA(aabb);
          aabb = temp;
      }
      big_boxes = NULL;

      cleanupTable(table);
      DEALLOCA(tested);
      tested = NULL;

      for (int i = 0; i < clusterCount; ++i)
      {
          ClusterNode *cluster = clusterArray[i];
          while (cluster)
          {
            ClusterNode *temp = cluster->next;
            DEALLOCA(cluster);
            cluster = temp;
          }
          clusterArray[i] = NULL;

          dxClusterNode *cluster2 = clusterAABBs[i];
          while (cluster2)
          {
            dxClusterNode *temp2 = cluster2->next;
            DEALLOCA(cluster2);
            cluster2 = temp2;
          }
          clusterAABBs[i] = NULL;

          cleanupTable(clusterTables[i]);

          delete(spaces[i]);
          delete(worlds[i]);
          spaces[i] = NULL;
          worlds[i] = NULL;

          if (geomDuplicationMap.find(i) != geomDuplicationMap.end())
          {
              geomMap& kidMap = geomDuplicationMap[i];
              for (geomMap::iterator it=kidMap.begin(); it!=kidMap.end(); ++it)
              {
                DEALLOCA(it->second);
              }
              kidMap.clear();
          }
      }

      geomDuplicationMap.clear();
      activeClusterCount = deactivatedClusterCount = 0;
}

bool dxClusteredWorldAndSpace::isGeomBig(dxGeom* geom)
{
    return (geom->type == dPlaneClass || geom->aabb[0] == -dInfinity || geom->aabb[1] == dInfinity || geom->aabb[2] == -dInfinity || geom->aabb[3] == dInfinity || geom->aabb[4] == -dInfinity || geom->aabb[5] == dInfinity);
}

bool dxClusteredWorldAndSpace::isGeomStatic(dxGeom* geom)
{
    return geom->is_dynamic == false;
}

bool dxClusteredWorldAndSpace::isGeomBodyless(dxGeom* geom)
{
    dSpaceID parentSpace = (dxSpace*)geom;
    while (parentSpace != NULL)
    {
      if (parentSpace->body != NULL || parentSpace->is_dynamic)
        return false;
      parentSpace = parentSpace->parent_space;
    }
    if (geom->type >= dSimpleSpaceClass)
    {
      for (dxGeom* geom2 = ((dxSpace*)geom)->first; geom2; geom2=geom2->next)
        if (geom2->body != NULL || geom2->is_dynamic)
          return false;
    }
    return true;
}

void dxClusteredWorldAndSpace::flagDynamicGeoms(dSpaceID _space)
{
    for (dxGeom* geom=_space->first; geom; geom=geom->next)
    {
        if (geom->is_dynamic == false)
            geom->is_dynamic = !isGeomBodyless(geom);
        if (geom->type >= dSimpleSpaceClass)
            flagDynamicGeoms((dxSpace*)geom);
    }
}

dxGeom *dxClusteredWorldAndSpace::retrieveBodySpaceAndFlagInnerGeoms(dxBody *body)
{
    if (body == NULL || body->geom == NULL)
        return NULL;

    // retrieve upper space containing all body's geoms
    // and mark geometries contained in the space so that no additional
    // dxClusterAABB is created for them
    dxGeom *geom = body->geom;
    dxGeom *space = geom->parent_space;
    dxGeom *upper_geom = geom; // in case of a simple body with geom no upper space exists
                               // other than the whole scene space
    while (space)
    {
      space->tag = true;
      dxGeom *parent_space = space->parent_space;
      if (parent_space != NULL && parent_space->parent_space == NULL)
      {
          // the upper geom associated with the current body is found
          // parent_space represents the whole scene space
          upper_geom = space;
          break;
      }
      space = parent_space;
    }

    while (geom)
    {
        geom->tag = true;
        geom = geom->body_next;
    }

    return upper_geom;
}

void dxClusteredWorldAndSpace::createAABB(dxBody *body, dxGeom *geom, dxClusterAABB *&_firstaabb, dxClusterAABB *&_bigboxes, bool isbigbox)
{
    dxGeom *body_geom = geom;
    if (body)
    {
        body_geom = retrieveBodySpaceAndFlagInnerGeoms(body);
        if (body_geom == NULL)
            body_geom = geom;

    }
    // if geom is superspace, split into its child geoms and create aabb for each.
    else if (geom && isGeomStatic(geom) && geom->type >= dSimpleSpaceClass)
    {
      ODE_PRINT("Subspace count: %d with body %p (bigbox: %d)\n", ((dxSpace*)geom)->count, body, isbigbox);
      for (dxGeom* geom2 = ((dxSpace*)geom)->first; geom2; geom2 = geom2->next)
      {
          ODE_PRINT("    Subspace geom: %p with body %p\n", geom2, geom2->body);
          createAABB(geom2->body, geom2, _firstaabb, _bigboxes, isbigbox);
      }
      return;
    }

    dxClusterAABB *aabb = (dxClusterAABB*) ALLOCA (sizeof(dxClusterAABB));
    aabb->geom = body_geom;
    aabb->body = body;
    aabb->cluster = NULL;
    aabb->clusterID = -1;
    aabb->next = NULL;
    aabb->tracker = -1;

    // if geometry is static, put in static objects list
    if (body_geom && isGeomBig(body_geom))
    {
      // do nothing, this geom will not be clustered
      aabb->next = _bigboxes;
      _bigboxes = aabb;
      bigboxCount++;

    }
    // if geometry is not subspace, put it in dynamic object list
    else
    {
      // aabb goes in main list
      aabb->next = _firstaabb;
      _firstaabb = aabb;
      aabb->level = 0;
      aabb->cellsize = (dReal) ldexp (gridstep, aabb->level);

      // discretize AABB position to cell size
      if (body_geom)
      {
          for (int i = 0; i < 6; ++i)
              aabb->dbounds[i] = (int) 1.0f * floorf ((body_geom->aabb[i] - hsAxis[i/2]) / aabb->cellsize);
          setMinMax(body_geom->aabb);
      }
      aabb->index = algoN++;
    }

}

void dxClusteredWorldAndSpace::updateAABBs(dxClusterAABB *_dynamicList, dxClusterAABB *_staticList)
{
    xmin = 999; xmax = -999; ymin = 999; ymax = -999; zmin = 999; zmax = -999;
    xsizemin = 999; xsizemax = -999; ysizemin = 999; ysizemax = -999; zsizemin = 999; zsizemax = -999;
    xsizeavg = 0.0f; ysizeavg = 0.0f; zsizeavg = 0.0f;

    for (dxClusterAABB *aabb=_dynamicList; aabb; aabb = aabb->next)
    {
        if (aabb->body) aabb->geom = retrieveBodySpaceAndFlagInnerGeoms(aabb->body);

        dxGeom *geom = aabb->geom;
        if (geom == NULL)
            continue;

        dUASSERT(geom->parent_space != NULL, "null space found");

        // discretize AABB position to cell size
        for (int i = 0; i < 6; ++i)
            aabb->dbounds[i] = (int) 1.0f * floorf ((geom->aabb[i] - hsAxis[i/2]) / gridstep);

        setMinMax(geom->aabb);
    }
}

void dxClusteredWorldAndSpace::initAABBs(dxWorld *_world, dxSpace *_space, dxClusterAABB *&_dynamicList, dxClusterAABB *&_staticList)
{
    xmin = 999; xmax = -999; ymin = 999; ymax = -999; zmin = 999; zmax = -999;
    xsizemin = 999; xsizemax = -999; ysizemin = 999; ysizemax = -999; zsizemin = 999; zsizemax = -999;
    xsizeavg = 0.0f; ysizeavg = 0.0f; zsizeavg = 0.0f;

    algoN = 0;
    for (dxBody *body = _world->firstbody; body; body = (dxBody*) body->next)
    {
      dxGeom *geom = body->geom;
      if (geom) geom->tag = false;
    }
    for (dxGeom *geom = _space->first; geom; geom = (dxGeom*) geom->next)
    {
      geom->tag = false;
    }

    ODE_PRINT("Processing world with %d bodies and space with %d geoms\n", _world->nb, _space->count);
    // First, go through all the bodies in the world and create bounding box information
    for (dxBody *body = _world->firstbody; body; body = (dxBody*) body->next)
    {
      dxGeom* geom = body->geom;

      if (geom) dUASSERT(geom->parent_space != NULL, "null space found");
      if (geom && (geom->tag == true))
        continue;

      if (geom) geom->tag = true;
      createAABB(body, geom, _dynamicList, _staticList, false);
    }

    bigboxCount = 0;
    // Then, go through all the geometries in the space and create bounding box information
    for (dxGeom *geom = _space->first; geom; geom = (dxGeom*) geom->next)
    {
      dUASSERT(geom->parent_space != NULL, "null space found");
      if (geom->tag == true)
      {
          ODE_PRINT("continue: %d\n", geom->type);
          continue;
      }

      ODE_PRINT("type: %d (%d)\n", geom->type, bigboxCount);
      geom->tag = true;
      createAABB(geom->body, geom, _dynamicList, _staticList, true);
    }

    ODE_PRINT("%d dynamic bodies, %d static bodies\n", algoN, bigboxCount);
}

void dxClusteredWorldAndSpace::calculateGridStep()
{
    const dReal dx = xmax - xmin;
    const dReal dy = ymax - ymin;
    const dReal dz = zmax - zmin;
    gridstep = util_MT::odeMax(dx, dy, dz) / gridsize;
    if (gridstep < 0) gridstep *= - 1;

    ODE_INFO("Gridstep value: %f\n", gridstep);

    hsAxis[0] = (xmax + xmin) / 2.0f;
    hsAxis[1] = (ymax + ymin) / 2.0f;
    hsAxis[2] = (zmax + zmin) / 2.0f;

    const dReal halfInvGridStep = 0.5 / gridstep;
    xsteps = 2 * ceilf(dx * halfInvGridStep) + 3;
    ysteps = 2 * ceilf(dy * halfInvGridStep) + 3;
    zsteps = 2 * ceilf(dz * halfInvGridStep) + 3;

    if (oldN != algoN || tested == NULL)
    {
      DEALLOCA(tested);
      // for `n' objects, an n*n array of bits is used to record if those objects
      // have been intersection-tested against each other yet. this array can
      // grow large with high n, but oh well...
      tested_rowsize = (algoN + 7) >> 3;	// number of bytes needed for n bits
      tested = (unsigned char *) ALLOCA (algoN * tested_rowsize);
    }
    memset (tested, 0, algoN * tested_rowsize);
    oldN = algoN;

    const int newHashSpaceSize = (xsteps + 1) * (ysteps + 1) * (zsteps + 1);

    // create a hash table to store all AABBs. each AABB may take up to 8 cells.
    // we use chaining to resolve collisions, but we use a relatively large table
    // to reduce the chance of collisions.

    // compute hash table size sz to be a prime > xsteps*ysteps*zsteps
    if (oldHashSpaceSize != newHashSpaceSize || table == NULL)
    {
      ODE_INFO("Creating new table... %d to %d\n", oldHashSpaceSize > 0 ? oldHashSpaceSize : 1, newHashSpaceSize > 0 ? newHashSpaceSize : 1);
      if (table != NULL)
      {
          cleanupTable(table);
      }

      sz = newHashSpaceSize > 0 ? newHashSpaceSize : 1; // the size is negative iff the there is no dGeoms

      // allocate and initialize hash table node pointers
      table = (dxClusterNode **) ALLOCA (sizeof(dxClusterNode*) * sz);
      for (int i = 0; i < sz; ++i) table[i] = NULL;

      oldHashSpaceSize = newHashSpaceSize;
    }
}

void dxClusteredWorldAndSpace::makeClusters()
{
    calculateGridStep();

    dxClusterAABB *aabb;
    // create a list of auxiliary information for all geom axis aligned bounding
    // boxes. set the level for all AABBs. put AABBs larger than the space's
    // global_maxlevel in the big_boxes list, check everything else against
    // that list at the end. for AABBs that are not too big, record the maximum
    // level that we need.

    ODE_PRINT("before: %d, %d\n", originalWorld->nb, originalSpace->count);

    // we need to tag nodes in table as free
    for (int i = 0; i < sz; ++i)
    if (table[i])
    {
        table[i]->nextFreeNode = table[i];
        table[i]->count = 0;
        table[i]->tagged = false;
    }

    clusterCount = 0;
    int tableCount = 0;
    int firstaabbCount = 0;
    int oldNodeCount = 0, newNodeCount = 0;
    // add each AABB to the hash table (may need to add it to up to 8 cells)
    for (aabb = first_aabb; aabb; aabb = aabb->next) {
    ++firstaabbCount;

    ClusterNode *head = NULL;
    ClusterNode *tail = NULL;
    bool bIsGeomStatic = false;
    if (aabb->body == NULL && aabb->geom) bIsGeomStatic = isGeomStatic(aabb->geom);

    // we need to recalculate bounding volumes since objects have moved and average position has changed
    if (aabb->geom)
        for (int i = 0; i < 6; ++i)
            aabb->dbounds[i] = (float) 1.0f * floorf ((aabb->geom->aabb[i] - hsAxis[i/2]) / gridstep);
    else
        for (int i = 0; i < 6; ++i) aabb->dbounds[i] = 0;

    const int *dbounds = aabb->dbounds;
    for (int xi = dbounds[0]; (xi) <= (dbounds[1]); ++xi) {
    for (int yi = dbounds[2]; (yi) <= (dbounds[3]); ++yi) {
    for (int zi = dbounds[4]; (zi) <= (dbounds[5]); ++zi) {
      // get the hash index
      dVector3 pos = { (dReal)xi, (dReal)yi, (dReal)zi};
      unsigned long hi = getVirtualAddress (pos) % sz;
      ++tableCount;

      dxClusterNode *node = NULL;
      // first check to see if we have a free node
      if (table[hi] && table[hi]->nextFreeNode)
      {
          node = table[hi]->nextFreeNode;
          table[hi]->nextFreeNode = node->next;

          if (node->next) node->next->count = node->count + 1;
          ++oldNodeCount;
      }
      // add a new node to the hash table if required
      else
      {
          node = (dxClusterNode*) ALLOCA (sizeof (dxClusterNode));
          node->nextFreeNode = NULL;

          if (table[hi]) node->count = table[hi]->count + 1;
          else node->count = 1;

          node->next = table[hi];
          table[hi] = node;
          ++newNodeCount;
      }

      node->x = xi;
      node->y = yi;
      node->z = zi;
      node->isStatic = bIsGeomStatic;

      node->aabb = aabb;
      node->tagged = false;
      node->cluster = NULL;
      node->space = NULL;
      node->world = NULL;

      if (bIsGeomStatic)
        addClusterNodeToCluster(node, head, tail, false);

    }
    }
    }

    // if it was a static geom, add to cluster list
    if (bIsGeomStatic)
    {
        clusterStaticTypes[clusterCount] = true;
        clusterArray[clusterCount] = head;
        if (aabb->geom)
            staticGeomClusterIDMap[aabb->geom] = clusterCount;
        clusterCount++;

        for (int i=0; i<clusterCount; i++)
        {
            staticClusterLinks[clusterCount-1][i] = false;
            staticClusterLinks[i][clusterCount-1] = false;
        }
    }

    }
    ODE_INFO("%d new nodes, %d old nodes\n", newNodeCount, oldNodeCount);

    // Finally, separate clusters based on proximity of hashspace cells
    // if 2 neighboring cells in the hashspace have objects inside,
    // they belong to the same cluster. In this way, we only need to traverse
    // the hashspace once and come with a list of clusters. Pretty fast.
    int nodeCount = 0;
    for (int i = 0; i < sz; ++i)
    {
        dxClusterNode *node = table[i];
        if (node && node->isStatic==false && node->tagged == false  && table[i]->nextFreeNode != table[i])
        {
            ++nodeCount;
            node->tagged = true;

            ClusterNode *head = NULL;
            ClusterNode *tail = NULL;
            findNeighboringCluster(node, head, tail);

            clusterStaticTypes[clusterCount] = false;
            clusterArray[clusterCount] = head;
            clusterCount++;

            for (int i=0; i<clusterCount; i++)
            {
                staticClusterLinks[clusterCount-1][i] = false;
                staticClusterLinks[i][clusterCount-1] = false;
            }
        }
    }

#ifndef dNODEBUG
{
    ODE_INFO("Geom count: %d\n", algoN);
    ODE_INFO("Big box count: %d\n", bigboxCount);
    ODE_INFO("Limits: (%f, %f), (%f, %f), (%f, %f)\n", xmin, xmax, ymin, ymax, zmin, zmax);
    ODE_INFO("%d out of %d table additions\n", tableCount, sz);
    ODE_IMPORTANT("dxClusterNode Count: %d\n", nodeCount);
    ODE_IMPORTANT("gridstep: %f, sz: %d, Cluster Count: %d\n", gridstep, sz, clusterCount);

    for (int i = 0; i < clusterCount; ++i)
    {
        ODE_INFO("Cluster %d: %d\n", i, clusterArray[i]->count);
    }
}
#endif
}

void dxClusteredWorldAndSpace::attachBodyAndGeom(int &bodyCount, int &geomCount, dxBody *body, dxGeom *geom, dxWorld *oldWorld, dxSpace *oldSpace, dxWorld *newWorld, dxSpace *newSpace)
{
    ODE_PRINT("--AttachBody&Geom (bodyCount=%d, geomCount=%d, body=%p, geom=%p, oldWorld=%p, oldSpace=%p, newWorld=%p, newSpace=%p)\n", bodyCount, geomCount, body, geom, oldWorld, oldSpace, newWorld, newSpace);
    if (body) dUASSERT(body->world != NULL, "NULL world in body!");
    if (body && oldWorld && body->world != newWorld) dUASSERT(body->world == oldWorld, "Body belongs to a different world than the one being removed!");
    if (geom) dUASSERT(geom->parent_space != NULL, "NULL parent space in geom!");
    if (body && newWorld && body->world != newWorld) dUASSERT(body != newWorld->firstbody, "Double insertion!");

    if (geom) geom->tag = true;
    if (body) body->mtTag = true;

    if (oldWorld && oldWorld == newWorld)
    {
        ODE_PRINT("Returning because oldWorld and newWorld are the same\n");
        return;
    }

    if (body && body->world != newWorld)
    {
        if (oldWorld == NULL) oldWorld = body->world;
        if (newWorld == NULL) newWorld = body->old_world; //currentWorld;

        removeObjectFromList(body);
        addObjectToList(body, (dObject**) &newWorld->firstbody);

        body->old_world = body->world;
        body->world = newWorld;
        newWorld->nb++;
        oldWorld->nb--;
        ++bodyCount;

        if (oldWorld->nb == 0) oldWorld->firstbody = NULL;

#ifndef dNODEBUG
        dUASSERT(util_MT::findBodyCount(oldWorld) == oldWorld->nb, "Body count mismatch in world!" );
        dUASSERT(util_MT::findBodyCount(newWorld) == newWorld->nb, "Body count mismatch in world!" );
#endif
    }

    if (oldSpace == NULL && newSpace == NULL) return;

    dxGeom *g = geom;

    if (oldSpace)
        while (g && g->parent_space != oldSpace)
            g = g->parent_space;

    if (!g)
    {
        ODE_PRINT("returning bcz of null geom!\n\n");
        return;
    }

    if (g->body) ODE_PRINT("Attach body 2 %p (%d) and geom %p (%d)\n", g->body, g->body->geom->tag, g, g->tag);

    g->tag = true;

    if (newSpace == NULL)
    {
        dUASSERT(g->old_space != NULL, "reverting to null space!");
        dSpaceRemove(g->parent_space, g);
        util_MT::addGeomToSpaceFunc(g->old_space, g);
        ++geomCount;
    }
    else if (g->parent_space != newSpace)
    {
        g->old_space = g->parent_space;
        dSpaceRemove(g->parent_space, g);
        util_MT::addGeomToSpaceFunc(newSpace, g);
        ++geomCount;
    }
    else
    {
        ODE_PRINT("not doin anything to geom!\n\n");
        return;
    }

}

void dxClusteredWorldAndSpace::attachJoint(int &jointCount, dxJoint *j, dxWorld *oldWorld, dxWorld *newWorld)
{
    ODE_PRINT("Attaching joint %p from world %p to %p\n", j, oldWorld, newWorld);
    if (oldWorld) dUASSERT(j->world == oldWorld, "the joint being reattached does not belong to the correct world!");

    j->mtTag = true;

    if (newWorld == NULL)
    {
        newWorld = j->old_world ? j->old_world : originalWorld;
    }

    // if already in correct world, abort
    if (j->world == newWorld)
    {
        ODE_PRINT("returning bcz joint is already in the correct world\n");
        return;
    }

    ODE_PRINT("World joint counts before: %d and %d\n", oldWorld->nj, newWorld->nj);

    removeObjectFromList(j);
    addObjectToList(j, (dObject**) &newWorld->firstjoint);

    j->old_world = j->world;
    j->world = newWorld;
    newWorld->nj++;
    oldWorld->nj--;
    ++jointCount;

    if (oldWorld->nj == 0) oldWorld->firstjoint = 0;

    ODE_PRINT("World joint counts after: %d and %d\n", oldWorld->nj, newWorld->nj);
}

void dxClusteredWorldAndSpace::reattachJoints(int &bodyCount, int &geomCount, int &jointCount, dxBody *body, dxGeom *geom, dxWorld *oldWorld, dxSpace *oldSpace, dxWorld *newWorld, dxSpace *newSpace, bool bTreatSpaces)
{
    ODE_PRINT("-reattachjoints: bodyCount %d  geomCount %d jointCount %d body %p, geom %p, oldWorld %p, newWorld %p, oldSpace %p, newSpace %p\n", bodyCount, geomCount, jointCount, body, geom, oldWorld, newWorld, oldSpace, newSpace);
    if (geom) dUASSERT(geom->parent_space != NULL, "parent space null!");

    bool bWorkToDo = false;
    if ((body && body->mtTag == false) || (geom && geom->tag == false))
    {
        bWorkToDo = true;
    }
    if (!bWorkToDo)
    {
        ODE_PRINT("returning bcz no work to do\n");
        return;
    }

    // if geometry is a subspace, add all of its children, but only update the body not the geom!
    if (bTreatSpaces && geom && geom->type >= dSimpleSpaceClass)
    {
        ODE_PRINT("Processing subspace %p\n", geom);
        dxSpace *gSpace = (dxSpace*) geom;
        for (dxGeom *g = gSpace->first; g; g = (dxGeom*) g->next)
        {
            reattachJoints(bodyCount, geomCount, jointCount, g->body, g, oldWorld, oldSpace, newWorld, newSpace, bTreatSpaces);
        }
    }

    // first transfer body and geometry
    if (body && body->geom)
      ODE_PRINT("Attach body 1 %p (%d) and geom %p (%d)\n", body, body->geom->tag, geom, geom->tag);
    else if (body)
      ODE_PRINT("Attach body 1 %p with no geom\n", body);

    attachBodyAndGeom(bodyCount, geomCount, body, geom, oldWorld, oldSpace, newWorld, newSpace);

#ifndef dNODEBUG
    if (oldWorld) dUASSERT(util_MT::findBodyCount(oldWorld) == oldWorld->nb, "Body count mismatch in world!" );
    if (newWorld) {
        dUASSERT(util_MT::findBodyCount(newWorld) == newWorld->nb, "Body count mismatch in world!" );
    } else if (body){
        dUASSERT(util_MT::findBodyCount(body->world) == body->world->nb, "Body count mismatch in world!" );
    }
#endif

    // then transfer all joints not connected to a tagged node
    ODE_PRINT("Processing joints\n");
    if (body) {
      for (dxJointNode *j = body->firstjoint; j; j = j->next)
      {
          // attach the joint if the connected body is either null or not tagged
          if (j->body == NULL || !j->body->geom || j->body->geom->tag == false)
          {
              dUASSERT(j->body != body, "self-joints allowed?");
              attachJoint(jointCount, j->joint, j->joint->world, newWorld);
#ifndef dNODEBUG
              if (oldWorld) dUASSERT(util_MT::findJointCount(oldWorld) == oldWorld->nj, "Joint count mismatch in world!" );
              if (newWorld) {
                  dUASSERT(util_MT::findJointCount(newWorld) == newWorld->nj, "Joint count mismatch in world!" );
              } else {
                  dUASSERT(util_MT::findJointCount(j->joint->world) == j->joint->world->nj, "Joint count mismatch in world!" );
              }
#endif
              dUASSERT(j->body != body, "self-joints allowed?");
          }
      }
    }
    ODE_PRINT("Processing connected bodies\n");
    // finally transfer all connected bodies which are not tagged
    if (body) {
      for (dxJointNode *j = body->firstjoint; j; j = j->next)
      {
          // recurse through the joint if the connected body has either no geometry or its geometry is not tagged
          dxBody *connectedBody = j->body;
          if (connectedBody == NULL || connectedBody->mtTag)
          {
              ODE_PRINT("Not processing connected body bcz already processed\n");
              continue;
          }

          if (!connectedBody->geom || connectedBody->geom->tag == false)
          {
              dUASSERT(connectedBody != body, "self-joints allowed?");
              reattachJoints(bodyCount, geomCount, jointCount, connectedBody, connectedBody->geom, oldWorld, oldSpace, newWorld, newSpace, bTreatSpaces);
              dUASSERT(j->body != body, "self-joints allowed?");
          }
      }
    }
}

// Note: this function is possibly called in a multithread way
dxGeom* dxClusteredWorldAndSpace::duplicateBodyAndGeom(int &bodyCount, int &geomCount, dxBody* body, dxGeom* geom, dxWorld *oldWorld, dxSpace *oldSpace, dxWorld *newWorld, dxSpace *newSpace, int kid)
{
    ODE_PRINT("Duplicating body %p and geom %p\n", body, geom);
    dxGeom* newGeom = 0x0;

    if (geom != 0x0)
    {
        switch(geom->type)
        {
            case dSphereClass:
            {
                newGeom = (dxSphere*)ALLOCA(sizeof(dxSphere));
                memcpy((void*)newGeom, (void*)geom, sizeof(dxSphere));
                break;
            }
            case dBoxClass:
            {
                newGeom = (dxBox*)ALLOCA(sizeof(dxBox));
                memcpy((void*)newGeom, (void*)geom, sizeof(dxBox));
                break;
            }
            case dCapsuleClass:
            {
                newGeom = (dxCapsule*)ALLOCA(sizeof(dxCapsule));
                memcpy((void*)newGeom, (void*)geom, sizeof(dxCapsule));
                break;
            }
            case dCylinderClass:
            {
                newGeom = (dxCylinder*)ALLOCA(sizeof(dxCylinder));
                memcpy((void*)newGeom, (void*)geom, sizeof(dxCylinder));
                break;
            }
            case dPlaneClass:
            {
                newGeom = (dxPlane*)ALLOCA(sizeof(dxPlane));
                memcpy((void*)newGeom, (void*)geom, sizeof(dxPlane));
                break;
            }
            case dRayClass:
            {
                newGeom = (dxRay*)ALLOCA(sizeof(dxRay));
                memcpy((void*)newGeom, (void*)geom, sizeof(dxRay));
                break;
            }
            case dConvexClass:
            {
                break;
            }
            case dGeomTransformClass:
            {
                newGeom = (dxGeomTransform*)ALLOCA(sizeof(dxGeomTransform));
                memcpy((void*)newGeom, (void*)geom, sizeof(dxGeomTransform));
                dxGeom* obj = ((dxGeomTransform*)geom)->obj;
                ((dxGeomTransform*)newGeom)->obj = duplicateBodyAndGeom(bodyCount, geomCount, obj->body, obj, oldWorld, oldSpace, newWorld, newSpace, kid);
                break;
            }
            case dTriMeshClass:
            {
                newGeom = (dxTriMesh*)ALLOCA(sizeof(dxTriMesh));
                memcpy((void*)newGeom, (void*)geom, sizeof(dxTriMesh));
                break;
            }
            case dHeightfieldClass:
            {
                newGeom = (dxHeightfield*)ALLOCA(sizeof(dxHeightfield));
                memcpy((void*)newGeom, (void*)geom, sizeof(dxHeightfield));
                ((dxHeightfield*)newGeom)->resetBufferPointers();
                break;
            }
            default:
            {
                ODE_PRINT("Duplicating superspace\n");
            }

        }
        newGeom->parent_space = 0x0;
        newGeom->next = 0x0;
        if (geom->parent_space != 0x0)
        {
            util_MT::addGeomToSpaceFunc(newSpace, newGeom);
            pthread_mutex_lock(&geomDuplicationMapMutex);
            geomDuplicationMap[kid][geom] = newGeom;
            pthread_mutex_unlock(&geomDuplicationMapMutex);
        }
        // transform geom class case:
        // we should also store the map transformed child -> copied children
        else if (geom->parent_geom) {
          pthread_mutex_lock(&geomDuplicationMapMutex);
          geomDuplicationMap[kid][geom] = newGeom;
          pthread_mutex_unlock(&geomDuplicationMapMutex);
        }
    }

    return newGeom;

}

void dxClusteredWorldAndSpace::duplicateJoint(int &jointCount, dxJoint *j, dxWorld *oldWorld, dxWorld *newWorld)
{
    ODE_PRINT("Duplicating joint %p\n", j);
}

// Note: this function is possibly called in a multithread way
void dxClusteredWorldAndSpace::duplicateBodiesAndGeoms(int &bodyCount, int &geomCount, int &jointCount, dxBody* body, dxGeom* geom, dxWorld *oldWorld, dxSpace *oldSpace, dxWorld *newWorld, dxSpace *newSpace, int kid)
{
    ODE_PRINT("-duplicateBodiesAndGeoms: bodyCount %d  geomCount %d jointCount %d body %p, geom %p, oldWorld %p, newWorld %p, oldSpace %p, newSpace %p\n", bodyCount, geomCount, jointCount, body, geom, oldWorld, newWorld, oldSpace, newSpace);
    if (geom) dUASSERT(geom->parent_space != NULL, "parent space null!");

    bool bWorkToDo = false;
    if ((body && body->mtTag == false) || (geom && geom->tag == false))
    {
        bWorkToDo = true;
    }
    if (!bWorkToDo)
    {
        return;
    }

    // first transfer body and geometry
    duplicateBodyAndGeom(bodyCount, geomCount, body, geom, oldWorld, oldSpace, newWorld, newSpace, kid);

    // then transfer all joints not connected to a tagged node
    if (body) {
      for (dxJointNode *j = body->firstjoint; j; j = j->next)
      {
          // attach the joint if the connected body is either null or not tagged
          if (j->body == NULL || !j->body->geom || j->body->geom->tag == false)
          {
              dUASSERT(j->body != body, "self-joints allowed?");
              duplicateJoint(jointCount, j->joint, j->joint->world, newWorld);
              dUASSERT(j->body != body, "self-joints allowed?");
          }
      }
  }
}

void dxClusteredWorldAndSpace::recombineClusters()
{
    for (int i=0; i<clusterCount && worlds[i]; i++)
        util_MT::cleanTags(worlds[i], this);

    // After the clusters are made, we separate the objects into different worlds, one per thread
    for (int k = 0; k < clusterCount; ++k)
    {
        for (dxClusterNode *node = clusterAABBs[k]; node; node = (dxClusterNode*) node->next)
        {
            if (!node->aabb->geom || node->aabb->geom->tag == false)
            {
#ifndef dNODEBUG
                if (node->aabb->body)
                {
                  for (dxJointNode *j = node->aabb->body->firstjoint; j; j = j->next)
                  {
                      dUASSERT(j->joint->world == node->aabb->body->world, "fishy business going with joints and body worlds");
                      if (j->body) dUASSERT(j->joint->world == j->body->world, "fishy business going with joints and body worlds");
                  }
                }
#endif

                dxWorld *nodeWorld = NULL;
                if (node->aabb->body) nodeWorld = worlds[k];
                dxSpace *nodeSpace = 0x0;
                if (node->aabb->geom) nodeSpace = spaces[k];

                int bCount = 0, gCount = 0, jCount = 0;
                reattachJoints(bCount, gCount, jCount, node->aabb->body, node->aabb->geom, nodeWorld, nodeSpace, originalWorld, originalSpace, false);
                ODE_INFO("%d bodies, %d geoms and %d joints reclaimed from cluster %d\n", bCount, gCount, jCount, k);

#ifndef dNODEBUG
                if (nodeWorld) checkWorld(nodeWorld);
#endif
            }
        }
    }

#ifdef TEST_COLLISIONS
  // Reclaim back static geometries
  for (int i = 0; i < threadCount; ++i)
  {
        dxGeom *g2;
        for (dxGeom *g = currentClusterSpace->spaces[i]->first; g; g = g2)
        {
            g2 = g->next;
            g->old_space = g->parent_space;
            dSpaceRemove(g->parent_space, g);
            addGeomToSpaceFunc(currentClusterSpace->originalSpace, g);
        }
  }
#endif
    cleanupMemory();
    clusterCount = 0;
    activeClusterCount = 0;
}

// Note: this function is possibly called in a multithread way
bool dxClusteredWorldAndSpace::updateClusterAABBsAndTable(int kid)
{
    // create cluster table if it doesnt already exist
    if (clusterTables[kid] == NULL)
    {
        // allocate and initialize hash table node pointers
        clusterTables[kid] = (dxClusterNode **) ALLOCA (sizeof(dxClusterNode*) * sz);
        for (int i = 0; i < sz; i++) clusterTables[kid][i] = 0;
    }
    // else, reset free node pointers
    else
    {
      // we need to tag nodes in table as free
      for (int i = 0; i < sz; ++i)
        if (clusterTables[kid][i])
        {
            clusterTables[kid][i]->nextFreeNode = clusterTables[kid][i];
            clusterTables[kid][i]->tagged = false;
        }
    }

    // then go through each cluster AABB and update dbounds and table
    int tableCount = 0, newNodeCount = 0, oldNodeCount = 0;
    int updateCount = 0;
    for (dxClusterNode *node = clusterAABBs[kid]; node; node = node->next)
    {
        dxClusterAABB *aabb = node->aabb;
        dxBody *body = aabb->body;
        dxGeom *geom = aabb->geom;
        dxJointNode* joint = NULL;

        // if aabb does not contain geom, get geom from body
        if (geom == NULL && body)
            geom = body->geom;

        // if we still(!) don't have a geom, we traverse the joints
        // through jointnodes instead of bodies
        if (body)
            joint = body->firstjoint;
        while (geom == NULL && joint)
        {
            body = joint->body;
            if (body)
                geom = body->geom;
            joint = joint->next;
        }

        updateCount++;

        // if geom was found, use absolute position of geom to update node aabb
        if (geom) // discretize AABB position to cell size
          for (int i = 0; i < 6; ++i)
            aabb->dbounds[i] = (int) 1.0f * floorf ((geom->aabb[i] - hsAxis[i/2])/gridstep);

        // else if no geom was found, we have a case of a geomless body possibly connected to other geomless bodies
        // this is a special case, and we use the position of the body here
        // WARNING: Not a generally good idea. Why?
        // Because body positions are not final, theyre used as offsets to geom positions, which are final.
        // EXAMPLE: Consider a robot with many geoms and bodies connected to each other
        // They also connect one geomless body. Now, when this robot moves, all geoms and bodies might move coherently
        // except the one body which is geomless. Using the body position now will split the robot cluster into 2 clusters
        // and we have a constant hysterics where the robot cluster is declustered every frame based on proximity
        // and reclustered the next frame (because of the joint connecting the geomless body).
        // TODO: Find a proper way to handle geomless bodies
        else
          for (int i = 0; i < 6; ++i)
            aabb->dbounds[i] = (int) 1.0f * floorf ((aabb->body->posr.pos[i%3] - hsAxis[i/2])/gridstep);

        int *dbounds = aabb->dbounds;
        for (int xi = dbounds[0]; (xi) <= (dbounds[1]); xi += 1) {
        for (int yi = dbounds[2]; (yi) <= (dbounds[3]); yi += 1) {
        for (int zi = dbounds[4]; (zi) <= (dbounds[5]); zi += 1) {
          // get the hash index
          dVector3 pos = { (dReal)xi, (dReal)yi, (dReal)zi};
          unsigned long hi = getVirtualAddress (pos) % sz;
          if (bCalculateNewSZ == true)
              ODE_PRINT("SCENE SIZE CHANGED! Aborting current frame\n");

          tableCount++;

          dxClusterNode *node2 = NULL;
          // first check to see if we have a free node
          if (clusterTables[kid][hi] && clusterTables[kid][hi]->nextFreeNode)
          {
              node2 = clusterTables[kid][hi]->nextFreeNode;
              clusterTables[kid][hi]->nextFreeNode = node2->next;

              oldNodeCount++;
          }
          else
          {
              ODE_PRINT("%d Adding new clusternode to cluster %d\n", newNodeCount, kid);
              node2 = (dxClusterNode*) ALLOCA (sizeof (dxClusterNode));
              node2->nextFreeNode = NULL;

              if (clusterTables[kid][hi]) node2->count = clusterTables[kid][hi]->count + 1;
              else node2->count = 1;

              node2->next = clusterTables[kid][hi];
              clusterTables[kid][hi] = node2;
              newNodeCount++;
          }

          node2->x = xi;
          node2->y = yi;
          node2->z = zi;

          node2->aabb = aabb;
          node2->tagged = false;
          node2->cluster = NULL;
          node2->space = NULL;
          node2->world = NULL;

        }
        }
        }
    }
    ODE_PRINT("%d geoms updated\n", updateCount);
    ODE_PRINT("%d table additions\n", tableCount);

    return bCalculateNewSZ;
}

void dxClusteredWorldAndSpace::recalculateActiveClusters()
{
    activeClusterCount = 0;
    for (int k = 0; k < clusterCount; ++k)
    {
        if (clusterAABBs[k] && clusterStaticTypes[k] == false)
        {
            activeClusters[activeClusterCount++] = k;
        }
    }
    ODE_INFO("Recalculated cluster count to %d\n", activeClusterCount);
}

void dxClusteredWorldAndSpace::transferNodeBetweenClusters(dxClusterNode *node, int oldClusterID, int newClusterID, bool bRefreshActiveClusters)
{
    ODE_PRINT("Transferring node %p from cluster %d to cluster %d\n", node, oldClusterID, newClusterID);
    dxClusterAABB *aabb = node->aabb;
    dxClusterNode *nodeToBeTransferred;

    if (node->aabb->body) dUASSERT(node->aabb->body->world == worlds[newClusterID], "Transferring node which is in wrong world");

    // if node is first in list, remove and update head
    if (node->aabb->node == NULL)
    {
        nodeToBeTransferred = clusterAABBs[oldClusterID];
        ODE_PRINT("removing head of list\n");
        dUASSERT(node->aabb == clusterAABBs[oldClusterID]->aabb, "node expected to be head of cluster list.");

        clusterAABBs[oldClusterID] = nodeToBeTransferred->next;

        if (clusterAABBs[oldClusterID]) clusterAABBs[oldClusterID]->aabb->node = NULL;
        else
        {
            // first off, clear the cluster table
            if (clusterTables[oldClusterID]) {
              for (int i = 0; i < sz; ++i)
              {
                  dxClusterNode* node = clusterTables[oldClusterID][i];
                  dxClusterNode* temp;
                  while (node)
                  {
                      temp = node->next;
                      DEALLOCA(node);
                      node = temp;
                  }
                  clusterTables[oldClusterID][i] = 0x0;
              }
            }
            // we have lost a cluster
            ODE_PRINT("Removing cluster %d completely\n", oldClusterID);

            deactivatedClusters[deactivatedClusterCount++] = oldClusterID;

            if (bRefreshActiveClusters)
            {
                recalculateActiveClusters();
                clusterChangeCallback();
            }
        }
    }
    // node not head of list, simply remove
    else
    {
        dxClusterNode *previousNode = node->aabb->node;
        nodeToBeTransferred = previousNode->next;
        previousNode->next = nodeToBeTransferred->next;
        if (previousNode->next) previousNode->next->aabb->node = previousNode;
    }

    aabb->clusterID = newClusterID;
    nodeToBeTransferred->aabb->node = NULL;
    // update parent node for AABB. used for crossover clustering
    if (clusterAABBs[newClusterID]) {
      clusterAABBs[newClusterID]->aabb->node = nodeToBeTransferred;
      nodeToBeTransferred->count = clusterAABBs[newClusterID]->count + 1;
    } else
      nodeToBeTransferred->count = 1;

    nodeToBeTransferred->next = clusterAABBs[newClusterID];
    clusterAABBs[newClusterID] = nodeToBeTransferred;
}

// Note: this function is possibly called in a multithread way
void dxClusteredWorldAndSpace::crossOver(int smallerCluster, int biggerCluster, int cellID)
{
    ODE_INFO("Crossover at cell %d between cluster %d and cluster %d\n", cellID, smallerCluster, biggerCluster);
    if (clusterAABBs[biggerCluster] == NULL)
    {
        ODE_PRINT("returning because of empty cluster\n");
        return;
    }

    // only update hashspace data if both clusters are dynamic
    bool bTransferNodeBetweenClusters = false;
    if (clusterStaticTypes[smallerCluster] == false && clusterStaticTypes[biggerCluster] == false)
        bTransferNodeBetweenClusters = true;
    else if (clusterStaticTypes[smallerCluster] == false && clusterStaticTypes[biggerCluster] == true)
    // if both clusters are dynamic, smallerCluster has fewer objects
    // else if one or both clusters are static, smallerCluster must be the static one.
    {
        ODE_PRINT("Switching small and big clusters\n");
        int temp = smallerCluster;
        smallerCluster = biggerCluster;
        biggerCluster = temp;
    }

    // first off, remove AABB from old cluster and add to new one
    for (dxClusterNode *node = clusterTables[smallerCluster][cellID]; node && node != clusterTables[smallerCluster][cellID]->nextFreeNode; node = node->next)
    {
        dxClusterAABB *aabb = node->aabb;
        // Make sure node hasnt already transferred. if not, crossover
        if (aabb->clusterID == biggerCluster || aabb->clusterID != smallerCluster)
        {
            ODE_PRINT("Continuing because node is already transferred\n");
            continue;
        }

        //ODE_PRINT("-Crossing over geom %p (type %d) body %p from space %p to %p\n", aabb->geom, aabb->geom->type, aabb->body, aabb->geom->parent_space, newSpace);

        util_MT::cleanTags(worlds[smallerCluster], this);
        util_MT::cleanTags(worlds[biggerCluster], this);

        // only if both clusters are dynamic do we transfer bodies/geoms between worlds/spaces
        // if one of the clusters is static, we duplicate this static world/space in the dynamic cluster
        int bCount = 0, gCount = 0, jCount = 0;
        if (bTransferNodeBetweenClusters)
        {
            ODE_INFO("moving geom %p from cluster %d to cluster %d\n", aabb->geom, smallerCluster, biggerCluster);
            // last parameter (treatSpaces) flag is true here since the joining of clusters might be
            // triggered by a low-level geom we therefore need to transfer the whole hierarchy.
            reattachJoints(bCount, gCount, jCount, aabb->body, aabb->geom, worlds[smallerCluster], spaces[smallerCluster], worlds[biggerCluster], spaces[biggerCluster], true);
            ODE_INFO("%d bodies, %d geoms, %d joints transferred from cluster %d to %d\n", bCount, gCount, jCount, smallerCluster, biggerCluster);

            if (aabb->body) dUASSERT(worlds[biggerCluster] == aabb->body->world, "unsuccessful reattachjoints");
            checkWorld(worlds[smallerCluster]);
            checkWorld(worlds[biggerCluster]);

            if (bCount > 0 || gCount > 0)
                transferNodeBetweenClusters(node, smallerCluster, biggerCluster);
        } else
        {
            ODE_INFO("duplicating geom %p from cluster %d to cluster %d\n", aabb->geom, smallerCluster, biggerCluster);
            duplicateBodiesAndGeoms(bCount, gCount, jCount, aabb->body, aabb->geom, worlds[smallerCluster], spaces[smallerCluster], worlds[biggerCluster], spaces[biggerCluster], biggerCluster);
            ODE_INFO("%d bodies changed worlds, %d geoms changed spaces, %d joints changed worlds\n", bCount, gCount, jCount);
        }

        if (bTransferNodeBetweenClusters)
        {
            // connected bodies will have changed worlds, we need to crossover these too
            dxClusterNode *node2 = clusterAABBs[smallerCluster];
            while (node2)
            {
                dxClusterNode *temp = node2->next;
                //if (node2->aabb->geom->parent_space == spaces[biggerCluster])
                if ((node2->aabb->body && node2->aabb->body->world == worlds[biggerCluster]) ||
                    (node2->aabb->geom && node2->aabb->geom->parent_space == spaces[biggerCluster]))
                    transferNodeBetweenClusters(node2, smallerCluster, biggerCluster);

                node2 = temp;
            }
        }

        if (clusterTables[smallerCluster][cellID] == 0) break;
    }
}

void dxClusteredWorldAndSpace::checkHashSpaceConsistency()
{
    if (bCalculateNewSZ)
    {
        ODE_PRINT("Calculating new sz...\n");

        cleanupTable(table);
        for (int k = 0; k < clusterCount; ++k)
          cleanupTable(clusterTables[k]);

        updateAABBs(first_aabb, big_boxes);
        calculateGridStep();

        for (int k = 0; k < clusterCount; ++k)
          updateClusterAABBsAndTable(k);

        ODE_PRINT("New sz: %d\n", sz);

        bCalculateNewSZ = false;
    }
}

// Note: this function is possibly called in a multithread way
void dxClusteredWorldAndSpace::crossOverClusters(int kid1, int kid2, int cellId)
{
    ODE_INFO("\nCrossover between clusters %d (%d) and %d (%d) in cell %d\n", kid1, clusterTables[kid1][cellId]->count, kid2, clusterTables[kid2][cellId]->count, cellId);
    int cellCount1 = clusterTables[kid1][cellId]->count;
    if (clusterTables[kid1][cellId]->nextFreeNode) cellCount1 -= clusterTables[kid1][cellId]->nextFreeNode->count;
    int cellCount2 = clusterTables[kid2][cellId]->count;
    if (clusterTables[kid2][cellId]->nextFreeNode) cellCount2 -= clusterTables[kid2][cellId]->nextFreeNode->count;

    // find bigger cell and merge the smaller cell with it
    int smallerCluster, biggerCluster;
    if (cellCount1 < cellCount2)
    {
        smallerCluster = kid1;
        biggerCluster = kid2;
    }
    else
    {
        smallerCluster = kid2;
        biggerCluster = kid1;
    }

    // only if one of the clusters atleast is dynamic, crossover
    if (!clusterStaticTypes[smallerCluster] || !clusterStaticTypes[biggerCluster])
    {
        bool bCrossOver = true;
        if (clusterAABBs[smallerCluster]->aabb->body && clusterAABBs[biggerCluster]->aabb->body &&
            clusterAABBs[smallerCluster]->aabb->body->world == clusterAABBs[biggerCluster]->aabb->body->world)
            bCrossOver = false;
        if (clusterAABBs[smallerCluster]->aabb->geom && clusterAABBs[biggerCluster]->aabb->geom &&
            clusterAABBs[smallerCluster]->aabb->geom->parent_space == clusterAABBs[biggerCluster]->aabb->geom->parent_space)
            bCrossOver = false;

        if (clusterStaticTypes[smallerCluster] == true && staticClusterLinks[smallerCluster][biggerCluster] == true)
            bCrossOver = false;
        if (clusterStaticTypes[biggerCluster] == true && staticClusterLinks[biggerCluster][smallerCluster] == true)
            bCrossOver = false;

        if (bCrossOver)
            crossOver(smallerCluster, biggerCluster, cellId);

        if (clusterStaticTypes[smallerCluster] == true)
            staticClusterLinks[smallerCluster][biggerCluster] = true;
        if (clusterStaticTypes[biggerCluster] == true)
            staticClusterLinks[biggerCluster][smallerCluster] = true;
    }
}

#include <vector>

// Note: this function is possibly called in a multithread way
void dxClusteredWorldAndSpace::checkClusterConsistency(int kid)
{
    std::vector<int> collidingClusterIds;
    int crossoverCount = 0;

    // go through the dynamic cluster and check for possible crossover with static objects
    int k = kid;
    for (dxClusterNode* node=clusterAABBs[k]; node; node=node->next)
    {
        const int *dbounds = node->aabb->dbounds;
        for (int xi = dbounds[0]; (xi) <= (dbounds[1]); ++xi) {
        for (int yi = dbounds[2]; (yi) <= (dbounds[3]); ++yi) {
        for (int zi = dbounds[4]; (zi) <= (dbounds[5]); ++zi) {
            // get the hash index
            dVector3 pos = { (dReal)xi, (dReal)yi, (dReal)zi};
            unsigned long hi = getVirtualAddress (pos) % sz;

            for (int j=0; j<clusterCount && j!=k; j++)
            {
                // if dynamic cluster or already duplicated, skip
                if (clusterStaticTypes[j] == false) // || staticClusterLinks[j][k] == true)
                    continue;

                if (clusterTables[j] && clusterTables[j][hi] && clusterTables[j][hi] != clusterTables[j][hi]->nextFreeNode)
                {
                    collidingClusterIds.push_back(j);
                    if (clusterTables[k][hi] && staticClusterLinks[j][k] == false)
                    {
                        ODE_PRINT("static geom %p (cluster %d) being duplicated to dynamic cluster %d\n\n", node->aabb->geom, j, k);
                        crossOverClusters(j, k, hi);
                    }

                }

            }
        }
        }
        }
    }

    crossoverCount = collidingClusterIds.size();
    if (crossoverCount == 0)
        return;

    // see which static objects need to be removed
    for (int j=0; j<clusterCount; j++)
    {
        // if static cluster is already linked in a previous frame, see if it is still linked in this frame
        if (staticClusterLinks[j][k] == true)
        {
            int l;
            for (l=0; l<crossoverCount; l++)
            {
                if (collidingClusterIds[l] == j)
                    break;
            }
            if (l==crossoverCount)
            {
                //ODE_PRINT("static cluster %d needs to be removed from dynamic cluster %d\n\n", j, k);
                ODE_PRINT("static geom %p (cluster %d) being removed from dynamic cluster %d\n\n", geomDuplicationMap[k][clusterAABBs[j]->aabb->geom], j, k);
                dSpaceRemove(spaces[k], geomDuplicationMap[k][clusterAABBs[j]->aabb->geom]);
                staticClusterLinks[j][k] = false;
            }
        }
    }
}

void dxClusteredWorldAndSpace::addTrackerToNode(int &bodyCount, int& geomCount, int& jointCount, int kid, int _xi, int _yi, int _zi, int tracker)
{
    dVector3 _pos = { (dReal)_xi, (dReal)_yi, (dReal)_zi};
    unsigned long _hi = getVirtualAddress(_pos) % sz;
    if (clusterTables[kid][_hi] == NULL)
        return;

    for (dxClusterNode* node2=clusterTables[kid][_hi]; node2 && node2!=clusterTables[kid][_hi]->nextFreeNode; node2=node2->next)
    {
        if (node2->aabb->tracker != tracker)
        {
            node2->aabb->tracker = tracker;
            bodyCount++;
            const int *dbounds = node2->aabb->dbounds;
            for (int xi = dbounds[0]; (xi) <= (dbounds[1]); ++xi) {
            for (int yi = dbounds[2]; (yi) <= (dbounds[3]); ++yi) {
            for (int zi = dbounds[4]; (zi) <= (dbounds[5]); ++zi) {
                // get the hash index and color all aabbs in cell
                addTrackerToNode(bodyCount, geomCount, jointCount, kid, xi, yi, zi, tracker);

                addTrackerToNode(bodyCount, geomCount, jointCount, kid, xi + 1, yi, zi, tracker);
                addTrackerToNode(bodyCount, geomCount, jointCount, kid, xi - 1, yi, zi, tracker);
                addTrackerToNode(bodyCount, geomCount, jointCount, kid, xi, yi + 1, zi, tracker);
                addTrackerToNode(bodyCount, geomCount, jointCount, kid, xi, yi - 1, zi, tracker);
                addTrackerToNode(bodyCount, geomCount, jointCount, kid, xi, yi, zi + 1, tracker);
                addTrackerToNode(bodyCount, geomCount, jointCount, kid, xi, yi, zi - 1, tracker);
            }
            }
            }
        }
    }

}

void dxClusteredWorldAndSpace::checkDynamicClusterConsistencies()
{
    ODE_PRINT("CheckDynamicClusters Start. calculateNewSz=%d\n", bCalculateNewSZ);
    int bCount=0, gCount=0, jCount=0;
    bool bCrossover = false;

    tracker++;
    int ccount = activeClusterCount;
    // check crossover between all dynamic clusters
    for (int i=0; i<ccount; i++)
    {
        int count=0;
        int k = activeClusters[i];
        std::map<dxGeom*, int> geomTags;
        for (dxClusterNode* node=clusterAABBs[k]; node; node=node->next)
        {
            // color current node with tracker id
            if (node->aabb->tracker != tracker)
            {
                //ODE_PRINT("node tracker: %d actual tracker: %d\n", node->aabb->tracker, tracker);
                node->aabb->tracker = ++tracker;
                if (++count > 1 && !bCrossover && !bCalculateNewSZ)
                {

                    // shit just got real. we need to add remaining geom/bodies into new cluster
                    int newClusterID;
                    if (deactivatedClusterCount > 0)
                        newClusterID = deactivatedClusters[--deactivatedClusterCount];
                    else
                        newClusterID = clusterCount++;

                    clusterStaticTypes[newClusterID] = false;
                    for (int l=0; l<clusterCount; l++)
                    {
                        staticClusterLinks[newClusterID][l] = false;
                        staticClusterLinks[l][newClusterID] = false;
                    }

                    if (newClusterID == clusterCount-1)
                    {
                        createNewWorldAndSpace(worlds[newClusterID], spaces[newClusterID]);
                    } else
                    {
                        dUASSERT(clusterAABBs[newClusterID] == NULL, "Previously deactivated cluster AABBs not empty!\n");
                        dUASSERT(worlds[newClusterID]->nb == 0, "Previously deactivated cluster world not empty!\n");
                        dxGeom *g,*n;
                        for (g = spaces[newClusterID]->first; g; g=n) {
                          n = g->next;
                          dSpaceRemove(spaces[newClusterID], g);
                        }
                    }

                    for (dxClusterAABB* bigbox = big_boxes; bigbox; bigbox = bigbox->next)
                        duplicateBodyAndGeom(bCount, gCount, bigbox->body, bigbox->geom, originalWorld, originalSpace, worlds[newClusterID], spaces[newClusterID], newClusterID);

                    util_MT::cleanTags(worlds[k], this);
                    util_MT::cleanTags(worlds[newClusterID], this);
                    bCount = gCount = jCount = 0;
                    // last parameter (treatSpaces) flag is true here since the separation of clusters might be
                    // triggered by a low-level geom we therefore need to transfer the whole hierarchy.
                    reattachJoints(bCount, gCount, jCount, node->aabb->body, node->aabb->geom, worlds[k], spaces[k], worlds[newClusterID], spaces[newClusterID], true);
                    ODE_INFO("%p Dynamic Cluster Split: %d bodies, %d geoms and %d joints transferred from cluster %d to cluster %d...\n", node->aabb->geom, bCount, gCount, jCount, k, newClusterID);
                    if (bCount > 0 || gCount > 0)
                        transferNodeBetweenClusters(node, k, newClusterID, false);

                    dxClusterNode *node2 = clusterAABBs[k];
                    while (node2)
                    {
                        dxClusterNode *temp = node2->next;
                        if ((node2->aabb->body && node2->aabb->body->world == worlds[newClusterID]) ||
                        (node2->aabb->geom && node2->aabb->geom->parent_space == spaces[newClusterID]))
                        {
                            transferNodeBetweenClusters(node2, k, newClusterID, false);
                        }

                        node2 = temp;
                    }

                    updateClusterAABBsAndTable(newClusterID);
                    recalculateActiveClusters();
                    clusterChangeCallback();
                }

            }

            const int *dbounds = node->aabb->dbounds;
            for (int xi = dbounds[0]; (xi) <= (dbounds[1]); ++xi) {
            for (int yi = dbounds[2]; (yi) <= (dbounds[3]); ++yi) {
            for (int zi = dbounds[4]; (zi) <= (dbounds[5]); ++zi) {
                // get the hash index
                dVector3 pos = { (dReal)xi, (dReal)yi, (dReal)zi};
                unsigned long hi = getVirtualAddress (pos) % sz;

                addTrackerToNode(bCount, gCount, jCount, k, xi, yi, zi, tracker);

                int crossoverCount = 0;
                dxMaintainedArray<int> collidingClusterIds;
                for (int j=i+1; j<ccount; j++)
                {
                    int k2 = activeClusters[j];
                    if (clusterTables[k2] && clusterTables[k2][hi] && clusterTables[k2][hi] != clusterTables[k2][hi]->nextFreeNode)
                    {
                        ODE_PRINT("Adding %d at cell %ld with clusterTable %p to colliding cluster Ids\n", k2, hi, clusterTables[k2][hi]);
                        collidingClusterIds[crossoverCount++] = k2;
                    }

                }

                // go through all colliding clusters and combine pairwise
                for (int j=0; j<crossoverCount; j++)
                {
                    if (clusterTables[k][hi] && clusterTables[collidingClusterIds[j]][hi])
                    {
                        crossOverClusters(collidingClusterIds[j], k, hi);
                        bCrossover = true;
                    }

                }

            }
            }
            }
        }

        if (count > 1)
            ODE_PRINT("kid: %d Tracker: %d count: %d\n", k, tracker, count);
    }

}

void dxClusteredWorldAndSpace::update(bool bRefreshClusters)
{

    if (bCalculateNewSZ == false)
        checkDynamicClusterConsistencies();
    checkHashSpaceConsistency();

    if (bRefreshClusters || ((originalWorld->nb > 0) && clusterCount < 1))
    {
        cleanupMemory();
        originalSpace->cleanGeoms();
        flagDynamicGeoms(originalSpace);
        initAABBs(originalWorld, originalSpace, first_aabb, big_boxes);
        makeClusters();
        assignClustersToWorlds();
        recalculateActiveClusters();
        clusterChangeCallback();
        for (int k = 0; k < clusterCount; ++k)
        {
            updateClusterAABBsAndTable(k);
            if (geomDuplicationMap.find(k) == geomDuplicationMap.end())
                geomDuplicationMap[k] = *(new geomMap());
        }
    }
}

void dxClusteredWorldAndSpace::assignClustersToWorlds()
{
  // First, turn off tags for all geoms, joints and bodies, so we can test for a 1-to-1 relationship between nodes and clusters
  util_MT::cleanTags(originalWorld, this);

  // After the clusters are made, we separate the objects into different worlds, one per thread
  int kid;
  int createdClusterCount = 0;
  entityCount = 0;
  for (kid = 0; kid < clusterCount; ++kid)
  {
    ODE_PRINT("Assigning new cluster to world %p and space %p...\n", worlds[kid], spaces[kid]);
    int bCount = 0, gCount = 0, jCount = 0;
    if (worlds[kid] == NULL && spaces[kid] == NULL) {
        createNewWorldAndSpace(worlds[kid], spaces[kid]);
        createdClusterCount++;
    }
    dUASSERT(worlds[kid] != NULL, "Newly created world is NULL!\n");
    dUASSERT(spaces[kid] != NULL, "Newly created space is NULL!\n");
    // if dynamic cluster, duplicate all planes into it first
    if (clusterStaticTypes[kid] != true)
    {
        for (dxClusterAABB* bigbox = big_boxes; bigbox; bigbox = bigbox->next)
            duplicateBodyAndGeom(bCount, gCount, bigbox->body, bigbox->geom, originalWorld, originalSpace, worlds[kid], spaces[kid], kid);
    }

    for (dxClusterNode *node = clusterAABBs[kid]; node; node = (dxClusterNode*) node->next)
    {
        if (!node->aabb->geom || node->aabb->geom->tag == false)
        {
#ifndef dNODEBUG
            if (node->aabb->body) {
              for (dxJointNode *j = node->aabb->body->firstjoint; j; j = j->next)
              {
                  if (node->aabb->body) dUASSERT(j->joint->world == node->aabb->body->world, "fishy business going with joints and body worlds");
                  if (j->body) dUASSERT(j->joint->world == j->body->world, "fishy business going with joints and body worlds");
              }
            }
#endif

            if (clusterStaticTypes[kid] != true)
            {
                bCount = gCount = jCount = 0;
                // edge case: a geomless connected body might have been assigned to this cluster
                // its connected bodies might have been assigned to a different cluster
                // in this case, this body is already transferred to the same cluster as its connected bodies
                // we therefore need to check if the body hasn't already been processed before over here
                if ((node->aabb->body && node->aabb->body->mtTag == false) || (node->aabb->geom && node->aabb->geom->tag == false))
                    reattachJoints(bCount, gCount, jCount, node->aabb->body, node->aabb->geom, originalWorld, originalSpace, worlds[kid], spaces[kid], false);
                else
                    continue;

                if (node->aabb->body) dUASSERT(node->aabb->body->world == worlds[kid], "Body not successfully inserted!\n");
                if (node->aabb->geom) dUASSERT(node->aabb->geom->parent_space != originalSpace, "Geom not successfully inserted!\n");

                ODE_INFO("%d bodies, %d geoms, %d joints were assigned to world/space %d\n", bCount, gCount, jCount, kid);
                if (bCount > 0)
                    entityCount++;

#ifndef dNODEBUG
                checkWorld(originalWorld);
                checkWorld(worlds[kid]);
#endif
            }

        } else
            ODE_PRINT("skipping cluster %d due to already tagged or nonexistant geom\n", kid);
    }
  }

  clusterCount = activeClusterCount = createdClusterCount;

  // bodies might have changed clusters due to joints. update
  for (int kid = 0; kid < clusterCount; ++kid)
  {
      if (!clusterAABBs[kid] || clusterStaticTypes[kid])
        continue;

      dxClusterNode *node = clusterAABBs[kid];
      while (node)
      {
          dxClusterNode *temp = node->next;
          if (node->aabb->body && node->aabb->body->world != worlds[kid])
          {
              int newKID = getWorldClusterID(node->aabb->body->world);
              transferNodeBetweenClusters(node, kid, newKID);
          }
          node = temp;
      }
  }

#ifdef TEST_COLLISIONS
{
  // We assume the remaining geometries from the space are all static. They belong to all subspaces
  dxGeom *g2;
  if (clusterCount > 0)
  for (dxGeom *g = currentClusterSpace->originalSpace->first; g; g = g2)
  {
      g2 = g->next;
      g->old_space = g->parent_space;
      dSpaceRemove(g->parent_space, g);
      addGeomToSpaceFunc(currentClusterSpace->spaces[0], g);
  }
}
#endif
}

int dxClusteredWorldAndSpace::getSpaceClusterID(dxSpace *_space)
{
    for (int kid = 0; kid < clusterCount; ++kid)
    {
        if (spaces[kid] == _space)
        {
            return kid;
        }
    }

    return 0;
}

int dxClusteredWorldAndSpace::getWorldClusterID(dxWorld *_world)
{
    for (int kid = 0; kid < clusterCount; ++kid)
    {
        if (worlds[kid] == _world)
        {
            return kid;
        }
    }

    return 0;
}

// Note: this function is possibly called in a multithread way
void dxClusteredWorldAndSpace::cleanTags()
{
    for (int k = 0; k < clusterCount; ++k)
    for (dxClusterNode *node = clusterAABBs[k]; node; node = node->next)
    {
        if (node->aabb->geom)
        {
            node->aabb->geom->tag = false;
            if (node->aabb->geom->type >= dSimpleSpaceClass)
            {
                dxSpace *gSpace = (dxSpace*) node->aabb->geom;
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
        }
        if (node->aabb->body) {
            if (node->aabb->body->geom)
                node->aabb->body->geom->tag = false;
            node->aabb->body->mtTag = false;
            for (dxJointNode *j = node->aabb->body->firstjoint; j; j = (dxJointNode*) j->next)
                j->joint->mtTag = false;
        }
    }

    for (int i = 0; i < clusterCount; ++i)
    {
        if (worlds[i] == NULL)
            continue;
        for (dxBody *b = worlds[i]->firstbody; b; b = (dxBody*) b->next) {
            b->mtTag = false;
            if (b->geom) b->geom->tag = false;
        }
        for (dxJoint *j = worlds[i]->firstjoint; j; j = (dxJoint*) j->next)
        {
            if (j->node[0].body && j->node[0].body->geom) j->node[0].body->geom->tag = false;
            if (j->node[1].body && j->node[1].body->geom) j->node[1].body->geom->tag = false;
        }

    }
}

dxClusteredWorldAndSpace::dxClusteredWorldAndSpace(clusterChangeCallbackFunc* callback)
{

    static bool first_call = true;
    if (first_call) {
      pthread_mutex_init(&geomDuplicationMapMutex, NULL);
      first_call = false;
    }

    clusterChangeCallback = callback;

    table = NULL;
    xmin = 999, xmax = -999, ymin = 999, ymax = -999, zmin = 999, zmax = -999;
    xsizemin = 999, xsizemax = -999, ysizemin = 999, ysizemax = -999, zsizemin = 999, zsizemax = -999;
    xsizeavg = 0.0f, ysizeavg = 0.0f, zsizeavg = 0.0f;

    sz = 0;
    algoN = 0;
    entityCount = 0;
    gridstep = 0.2f;
    gridsize = 5.0f;

    tested = NULL;
    tracker = 0;
    oldN = 0;
    oldHashSpaceSize = 0;

    clusterCount = 0;
    activeClusterCount = 0;
    deactivatedClusterCount = 0;
    first_aabb = NULL;	// list of AABBs in hash table
    big_boxes = NULL;	// list of AABBs too big for hash table
    bigboxCount = 0;
    bCalculateNewSZ = false;

    for (int k=0; k<clusterCount; k++)
    {
        worlds[k]           = NULL;
        spaces[k]           = NULL;
        clusterAABBs[k]     = NULL;
        clusterTables[k]    = NULL;
        clusterArray[k]     = NULL;
        activeClusters[k]   = 0;
    }
}

dxClusteredWorldAndSpace::~dxClusteredWorldAndSpace()
{

}

void dxClusteredWorldAndSpace::propagateGeomPosition(dxGeom *g, dReal x, dReal y, dReal z, dGeomSetPositionFunction* _func)
{
    for (int i=0; i<activeClusterCount; i++)
    {
        int k = activeClusters[i];
        if (geomDuplicationMap[k][g])
            _func(geomDuplicationMap[k][g], x, y, z);
    }
    updateClusterAABBsAndTable(staticGeomClusterIDMap[g]);
}

void dxClusteredWorldAndSpace::propagateGeomRotation(dxGeom *g, const dMatrix3 R, dGeomSetRotationFunction* _func)
{
    for (int i=0; i<activeClusterCount; i++)
    {
        int k = activeClusters[i];
        if (geomDuplicationMap[k][g])
            _func(geomDuplicationMap[k][g], R);
    }
    updateClusterAABBsAndTable(staticGeomClusterIDMap[g]);
}

void dxClusteredWorldAndSpace::propagateGeomQuaternion(dxGeom *g, const dQuaternion quat, dGeomSetQuaternionFunction* _func)
{
  for (int i=0; i<activeClusterCount; i++)
  {
    int k = activeClusters[i];
    if (geomDuplicationMap[k][g])
      _func(geomDuplicationMap[k][g], quat);
  }
  updateClusterAABBsAndTable(staticGeomClusterIDMap[g]);
}

void dxClusteredWorldAndSpace::propagateGeomTransformUpdate(dxGeom *g)
{
  for (int i=0; i<activeClusterCount; i++)
  {
    int k = activeClusters[i];
    if (geomDuplicationMap[k][g]) {
      dGeomMoved(geomDuplicationMap[k][g]);
      geomDuplicationMap[k][g]->recomputeAABB();
    }
  }
  updateClusterAABBsAndTable(staticGeomClusterIDMap[g]);
}

void dxClusteredWorldAndSpace::propagateGeomBoxSetLengths(dxGeom *g, dReal lx, dReal ly, dReal lz, dGeomBoxSetLengthsFunction* _func)
{
  for (int i=0; i<activeClusterCount; i++)
  {
    int k = activeClusters[i];
    if (geomDuplicationMap[k][g])
      _func(geomDuplicationMap[k][g], lx, ly, lz);
  }
  updateClusterAABBsAndTable(staticGeomClusterIDMap[g]);
}

void dxClusteredWorldAndSpace::propagateGeomCapsuleSetParams(dxGeom *g, dReal radius, dReal length, dGeomCapsuleSetParamsFunction* _func) {

  for (int i=0; i<activeClusterCount; i++)
  {
    int k = activeClusters[i];
    if (geomDuplicationMap[k][g])
      _func(geomDuplicationMap[k][g], radius, length);
  }
  updateClusterAABBsAndTable(staticGeomClusterIDMap[g]);
}

void dxClusteredWorldAndSpace::propagateGeomCylinderSetParams(dxGeom *g, dReal radius, dReal length, dGeomCylinderSetParamsFunction* _func) {

  for (int i=0; i<activeClusterCount; i++)
  {
    int k = activeClusters[i];
    if (geomDuplicationMap[k][g])
      _func(geomDuplicationMap[k][g], radius, length);
  }
  updateClusterAABBsAndTable(staticGeomClusterIDMap[g]);
}

void dxClusteredWorldAndSpace::propagateGeomTriMeshSetData(dxGeom *g, dTriMeshDataID data, dGeomTriMeshSetDataFunction* _func)
{
  for (int i=0; i<activeClusterCount; i++)
  {
    int k = activeClusters[i];
    if (geomDuplicationMap[k][g])
      _func(geomDuplicationMap[k][g], data);
  }
  updateClusterAABBsAndTable(staticGeomClusterIDMap[g]);

}

void dxClusteredWorldAndSpace::propagateGeomSphereSetRadius(dxGeom *g, dReal radius, dGeomSphereSetRadiusFunction* _func) {

  for (int i=0; i<activeClusterCount; i++)
  {
    int k = activeClusters[i];
    if (geomDuplicationMap[k][g])
      _func(geomDuplicationMap[k][g], radius);
  }
  updateClusterAABBsAndTable(staticGeomClusterIDMap[g]);
}

void dxClusteredWorldAndSpace::propagatePlaneParams(dxGeom *g, dReal a, dReal b, dReal c, dReal d, dGeomPlaneSetParamsFunction *_func)
{
    for (int i=0; i<activeClusterCount; i++)
    {
        int k = activeClusters[i];
        if (geomDuplicationMap[k][g])
            _func(geomDuplicationMap[k][g], a, b, c, d);
    }
    updateClusterAABBsAndTable(staticGeomClusterIDMap[g]);
}

void dxClusteredWorldAndSpace::propagateOffsetChange(dxGeom *g) {
  updateClusterAABBsAndTable(staticGeomClusterIDMap[g]);
}
