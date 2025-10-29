#pragma once
#include "NavMeshDebugger.h"
#include "Core/Scene.h"
#include "Core/Shader.h"
#include "Core/Camera.h"
#include <map>
#include "StructsForNavigationSystem.h"

class NavMesh
{
public:
    NavBuildParams BuildParams;
    DrawDebugInfo DebugInfo;

    glm::vec3 Start, End;

    NavMesh(const Scene& scene);
    ~NavMesh();
    
    void BuildNavMesh();
    void RenderDebugTool(Shader* shader, Camera& camera, const Scene& scene);
    void SetStartEndMarkers(const glm::vec3& start, const glm::vec3& end);
    int FindNodeIDByPosition(const glm::vec3& position);
    glm::vec3 GetNodeCenter(int nodeID);
    const std::vector<OptimizedEdge>& GetNodeNeighbors(int nodeID);
private:
    void BuildBorderFromParams();
    void CreateDebugger();
    
    void EarClipping();
    void CreateHalfEdgeStructure();
    void FindTwinHalfEdges();

    void OptimizeEarClipping();
    
    const Scene& m_Scene;
    NavMeshDebugger* m_NavMeshDebugger;
    std::vector<glm::vec3> m_BorderVerts3D;
    std::vector<glm::vec3> m_NavMeshVerts3D;
    std::vector<NavMeshTriangle> m_NavMeshTriangles;

    std::vector<HalfEdgeVertex> m_HalfEdgeVertices;
    std::vector<HalfEdgeFace> m_HalfEdgeFaces;
    std::vector<HalfEdge> m_HalfEdges;
    std::vector<NavMeshOptimizedNode> m_PathfindingNodes;

    std::vector<int> m_FoundPathNodeIDs;

    static bool IsPointInTriangleXZ(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);
    static bool CanClipEar(int prevIndex, int earIndex, int nextIndex, const std::vector<glm::vec3>& vertices);
    int FindHalfEdgeIndex(const glm::vec3& pos);
    
    void FindRemovableEdgeIndices(std::vector<int>& outRemovableEdgeIndices);
    void MergeTriangles(const std::vector<int>& removableEdgeIndices);
    void CreatePathfindingNodes(std::map<int, int>& faceIndexToNodeIndex, std::vector<std::vector<glm::vec3>>& mergedPolygonsForDebug);
    void FindNeighborsForPathfindingNodes(std::map<int, int>& faceIndexToNodeIndex);

    
    static bool IntersectLineSegmentsXZ(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& q1, const glm::vec3& q2, float& outLambda1, float& outLambda2, float epsilon = 1e-6f);
    static bool RaycastXZ(const std::vector<glm::vec3>& vertices, const glm::vec3& rayP1, const glm::vec3& rayP2, NavMeshHitInfo& hitInfo);
    std::vector<std::vector<glm::vec3>> GetSceneObstacleSlices(float buildPlaneY) const;
    void SortObstaclesByMaxX(std::vector<std::vector<glm::vec3>>& obstacleSlices);
    void InsertObstacle(const std::vector<glm::vec3>& obstacleSlice);
};
