#include "AStar.h"

AStar::AStar()
{
}

AStar::~AStar()
{
}

bool AStar::getPath(AStarNode *start, AStarNode *goal, std::vector<AStarNode *> &path)
{
	AStarNode *currentNode, *childNode;
	float f, g, h;

	std::make_heap(open.begin(), open.end(), CompareNodes());
	pushOpen(start);

	while (!open.empty())
	{
		std::sort(open.begin(), open.end(), CompareNodes());

		currentNode = open.front();
		popOpen(currentNode);

		currentNode->setClosed(true);
		closed.push_back(currentNode);

		if (currentNode == goal)
		{
			reconstructPath(currentNode, path);
			return true;
		}

		for (const auto &children : currentNode->getChildren())
		{
			childNode = static_cast<AStarNode *>(children.first);
			g = currentNode->getG() + children.second;
			if ((childNode->isOpen() || childNode->isClosed()) && childNode->getG() < g)
				continue;

			h = distanceBetween(childNode, goal);
			f = g + h;
			childNode->setF(f);
			childNode->setG(g);
			childNode->setH(h);
			childNode->setParent(currentNode);

			if (childNode->isClosed())
				childNode->setClosed(false);
			if (!childNode->isOpen())
				pushOpen(childNode);
		}
	}
	return false;
}

void AStar::pushOpen(AStarNode *node)
{
	open.push_back(node);
	std::push_heap(open.begin(), open.end(), CompareNodes());
	node->setOpen(true);
}

void AStar::popOpen(AStarNode *node)
{
	std::pop_heap(open.begin(), open.end(), CompareNodes());
	open.pop_back();
	node->setOpen(false);
}

void AStar::releaseNodes()
{
	for (const auto &node : open)
		node->release();
	for (const auto &node : closed)
		node->release();
}

void AStar::clear()
{
	releaseNodes();
	open.clear();
	closed.clear();
}
