#pragma once
#include "NavMesh.h"

class Pathfinder
{
public:
    static std::vector<int> FindPath(NavMesh& navMesh, const glm::vec3& start, const glm::vec3& end);
private:
    static float CalculateCost(NavMesh& navMesh, int nodeA, int nodeB);
    static std::vector<int> ReconstructPath(std::map<int, int>& cameFrom, int currentID);
};
