#ifndef WR_TRANSFORM_H
#define WR_TRANSFORM_H

#define WR_TRANSFORM(x) ((WrTransform *)(x))

#ifdef __cplusplus
extern "C" {
#endif

/* Inheritance diagram: WrNode <- WrTransform */
struct WrTransform;
typedef struct WrTransform WrTransform;

struct WrNode;
typedef struct WrNode WrNode;

/* Use wr_node_delete(WR_NODE(transform)) to delete an instance */
WrTransform *wr_transform_new();
/* Returns a copy of 'transform' (without children & parent, parent matrices are applied at copy) */
WrTransform *wr_transform_copy(WrTransform *transform);
/* Returns the matrix equivalent to this transformation (column major, parent transforms included) */
const float *wr_transform_get_matrix(WrTransform *transform);

void wr_transform_attach_child(WrTransform *transform, WrNode *child);
void wr_transform_detach_child(WrTransform *transform, WrNode *child);

/* Expects a 3-component array */
void wr_transform_set_position(WrTransform *transform, const float *position);
void wr_transform_set_absolute_position(WrTransform *transform, const float *position);
/* Expects a 4-component array */
void wr_transform_set_orientation(WrTransform *transform, const float *angle_axis);
void wr_transform_set_absolute_orientation(WrTransform *transform, const float *angle_axis);
/* Expects a 3-component array */
void wr_transform_set_scale(WrTransform *transform, const float *scale);
/* Optimization to set both position and orientation of a transform */
void wr_transform_set_position_and_orientation(WrTransform *transform, const float *position, const float *angle_axis);
#ifdef __cplusplus
}
#endif

#endif  // WR_TRANSFORM_H
