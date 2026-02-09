#pragma once

#include <vector>

/** Generic pathfinding algorithm template. Must be implemented.
 */

template <class TNode>
class PathAlgorithm
{
public:
	typedef TNode node_type;

	/**
	 * @brief The core method of the algorithm
	 * @param[in] start Pointer
	 * @param[in] goal Pointer
	 * @param[out] path A vector of nodes, possibly a path
	 * @return true if a path is found, otherwise false
	 */
	virtual bool getPath(TNode *start, TNode *goal, std::vector<TNode *> &path) = 0;

	/**
	 * @brief Provides a way to clean data
	 */
	virtual void clear() = 0;

protected:
	/**
	 * @brief Computes the distance between nodes using
	 * Node::distanceTo()
	 * @param[in] n1 Source Pointer
	 * @param[in] n2 Destination Pointer
	 * @see Node::distanceTo()
	 */
	inline float distanceBetween(TNode *n1, TNode *n2) const
	{
		return n1->distanceTo(n2);
	}

	/**
	 * @brief Builds path backwards from goal
	 * @param[in] node Location
	 * @param[out] path Nodes
	 */
	void reconstructPath(TNode *node, std::vector<TNode *> &path)
	{
		TNode *parent = static_cast<TNode *>(node->getParent());
		path.push_back(node);
		while (parent != nullptr)
		{
			path.push_back(parent);
			parent = static_cast<TNode *>(parent->getParent());
		}
	}
};