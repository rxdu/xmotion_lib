#ifndef QUAD_TREE_
#define QUAD_TREE_

#include <cstdint>
#include <vector>
#include <cstdint>

#include "qtree_types.h"

namespace srcl_ctrl {

// Definition of Tree Node
/*     Order of child nodes    */
/* 2 - top_left, 3 - top_right */
/* 0 - bot_left, 1 - bot_right */
class TreeNode{
public:
	TreeNode(BoundingBox bound, OccupancyType occupancy);
	~TreeNode();

public:
	// Node contents
	NodeType node_type_;
	OccupancyType occupancy_;
	BoundingBox bounding_box_;

	// Pointers to child nodes
	TreeNode* child_nodes_[4];

	// Extra information for neighbor search
	TreeNode* dummy_root_;
	bool has_dummy_;
};

class QTreeNodeManager{
public:
	QTreeNodeManager(uint8_t tree_depth);
	~QTreeNodeManager();

private:
	std::vector<TreeNode*> tree_nodes_;

public:
	uint16_t side_node_num_;

public:
	void SetNodeReference(uint16_t index_x, uint16_t index_y, TreeNode* node);
	TreeNode* GetNodeReference(uint16_t index_x, uint16_t index_y);
};

// Definition of Quad-Tree
class QuadTree{
public:
	QuadTree();
	QuadTree(uint16_t image_size, uint8_t depth);
	~QuadTree();

public:
	TreeNode* root_node_;
	uint8_t tree_depth_;
	uint16_t cell_res_;
	const uint8_t MAX_DEPTH;

	// Extra information for neighbor search
	QTreeNodeManager* node_manager_;

private:
	std::vector<TreeNode*> GetDummyNeighbours(TreeNode* dummy_leaf);

public:
	TreeNode* GetNodeAtPosition(uint16_t pixel_x, uint16_t pixel_y);
	std::vector<TreeNode*> FindNeighbours(TreeNode* node);
};

}

#endif
