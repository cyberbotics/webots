#ifndef WR_NODE_H
#define WR_NODE_H

#define WR_NODE(x) ((WrNode *)(x))

#ifdef __cplusplus
extern "C" {
#endif

struct WrNode;
typedef struct WrNode WrNode;

struct WrTransform;
typedef struct WrTransform WrTransform;

void wr_node_delete(WrNode *node);

WrTransform *wr_node_get_parent(WrNode *node);
void wr_node_set_visible(WrNode *node, bool is_visible);
bool wr_node_is_visible(WrNode *node);

#ifdef __cplusplus
}
#endif

#endif  // WR_NODE_H
