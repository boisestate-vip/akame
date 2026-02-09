#pragma once

#include <vector>
#include <iterator>
#include "PathAlgorithm.h"

/**
 * Main class for path finding.
 * @tparam T The types of nodes derived from Node
 * @see Node
 */
template <class TNode>
class PathFinder
{
public:
	/**
	 * @brief Default constructor
	 */
	explicit PathFinder() : m_start(nullptr), m_goal(nullptr)
	{
	}

	/**
	 * @brief Sets the start node
	 * @param[in] start A refrence to the Start
	 */
	void setStart(TNode &start)
	{
		m_start = &start;
	}

	/**
	 * @brief Sets the goal node
	 * @param[in] start A refrence to the Goal
	 */
	void setGoal(TNode &goal)
	{
		m_goal = &goal;
	}

	/**
	 * @brief Returns the address of the Start
	 * @return The address of Start
	 */
	TNode *getStart() const
	{
		return m_start;
	}

	/**
	 * @brief Returns the address of the Goal
	 * @return The address of Goal
	 */
	TNode *getGoal() const
	{
		return m_goal;
	}

	/**
	 * @brief Use the algorithm to path find
	 * @tparam U The Algorithm to use
	 * @param[out] solution The vector path
	 * @param[in] hint Optional :  hint to lenght of the path
	 */
	template <class TAlgorithm>
	bool findPath(std::vector<TNode *> &solution, int hint = 0)
	{
		std::vector<typename TAlgorithm::node_type *> path;
		TAlgorithm &algorithm = TAlgorithm::getInstance();

		if (hint > 0)
			path.reserve(hint);

		bool pathFound = algorithm.getPath(m_start, m_goal, path);

		if (!pathFound)
			return false;

		if (hint > 0)
			solution.reserve(hint);

		for (auto rit = std::rbegin(path); rit != std::rend(path); ++rit)
			solution.push_back(static_cast<TNode *>(*rit));

		return true;
	}

private:
	TNode *m_start, *m_goal;
};
