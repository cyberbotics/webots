#ifndef WR_FILE_LOADER
#define WR_FILE_LOADER

#ifdef __cplusplus
extern "C" {
#endif

struct WrSkeleton;
typedef struct WrSkeleton WrSkeleton;

struct WrStaticMesh;
typedef struct WrStaticMesh WrStaticMesh;

struct WrDynamicMesh;
typedef struct WrDynamicMesh WrDynamicMesh;

// Utility methods to load WREN objects from files.
// Return false in case of failure.

// Load static mesh from OBJ file
bool wr_import_static_mesh_from_obj(const char *fileName, WrStaticMesh **mesh);

// Load skeleton and dynamic meshes from FBX/Ogre file
const char *wr_import_skeleton_from_file(const char *fileName, WrSkeleton **skeleton, WrDynamicMesh ***meshes,
                                         const char ***materials, int *count);
const char *wr_import_skeleton_from_memory(const char *data, int size, const char *hint, WrSkeleton **skeleton,
                                           WrDynamicMesh ***meshes, const char ***materials, int *count);

#ifdef __cplusplus
}
#endif

#endif  // WR_FILE_LOADER
