#pragma once
#include "bbox.h"
#include "primitive.h"
#include "cudaBRT.h"
#include "../load_obj.h"
#include <vector> 



/**
* Bounding Volume Hierarchy for fast point-objects intersection.
* Note that the BVHAccel is an Aggregate (A Primitive itself) that contains
* all the primitives it was built from. Therefore once a BVHAccel Aggregate
* is created, the original input primitives can be ignored from the scene
* during point-objects intersection tests as they are contained in the aggregate.
*/
class BVHAccel : public Primitive {
public:

	BVHAccel() { }
	/**
	* Parameterized Constructor.
	* Create BVH from a list of primitives. Note that the BVHAccel Aggregate
	* stores pointers to the primitives and thus the primitives need be kept
	* in memory for the aggregate to function properly.
	* \param primitives primitives to build from
	* \param max_leaf_size maximum number of primitives to be stored in leaves
	*/
	BVHAccel(const std::vector<Primitive>& primitives, size_t max_leaf_size = 4);

	/**
	* Destructor.
	* The destructor only destroys the Aggregate itself, the primitives that
	* it contains are left untouched.
	*/
	~BVHAccel();


public:
	std::vector<Primitive> primitives;
	vector<glm::vec3> obj_vertices;



		bool intersect(const glm::vec3 point, int& idx) const;

		BRTreeNode* get_root() const;

		BRTreeNode* get_left_child(BRTreeNode* node) const;

		BRTreeNode* get_right_child(BRTreeNode* node) const;

		bool is_leaf(BRTreeNode* node) const;

		bool check_overlap(const glm::vec3 point, BRTreeNode* node) const;

	//显示包围盒之前需要调用，完成数据从GPU到CPU的拷贝
	void pre_drawoutline();  //for test
	void draw(BRTreeNode* root);
	void access(BRTreeNode* root, vector<BRTreeNode*>& bad_bode);
private:

	//functions for morton code based BVH construction algorithm
	unsigned int expandBits(unsigned int v);
	unsigned int morton3D(float x, float y, float z);
	unsigned int morton3D(glm::vec3 pos);
	static bool mortonCompare(const Primitive& p1, const Primitive& p2);
	void ParallelBVHFromBRTree(BRTreeNode* _d_leaf_nodes, BRTreeNode* _d_internal_nodes);
public:

	Primitive* d_primitives;
	BRTreeNode* d_leaf_nodes;
	BRTreeNode* d_internal_nodes;
	BRTreeNode* h_leaf_nodes;
	BRTreeNode* h_internal_nodes;

	int numInternalNode;
	int numLeafNode;
};

