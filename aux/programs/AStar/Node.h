#pragma once

#include <vector>

class Node
{
public:
	Node();
	virtual ~Node();

	/**
	 * @brief Assigns the parent.
	 * @param[in] parent Pointer
	 */
	void setParent(Node *parent);

	/**
	 * @brief Returns a pointer to the parent
	 * @reutrns parent pointer
	 */
	Node *getParent() const;

	/**
	 * @brief Add a node to the list of children
	 * @param[in] child Pointer
	 */
	void addChild(Node *child, float distance);

	/**
	 * @brief Returns a vector containing all children
	 * @return A vector of Node pointers
	 */
	std::vector<std::pair<Node *, float>> &getChildren();

protected:
	/**
	 * @brief Clears the children of the node
	 */
	void clearChildren();

	/**
	 * Pointer to the parent node.
	 */
	Node *m_parent;

	/**
	 * List of all the node's children.
	 */
	std::vector<std::pair<Node *, float>> m_children;
};
