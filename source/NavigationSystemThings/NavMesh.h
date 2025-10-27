#pragma once
#include "NavMeshDebugger.h"
#include "Core/Scene.h"
#include "Core/Shader.h"
#include "Core/Camera.h"

struct NavBuildParams
{
    glm::vec3 origin = glm::vec3(-15.f, 0.4f, -15.f);
    glm::vec3 maxBounds = glm::vec3(15.f, 2.f, 15.f);
};
struct NavMeshTriangle
{
    glm::vec3 verts[3];
};
struct NavMeshHitInfo
{
    float lambda = 0.0f;
    glm::vec3 intersectPoint = glm::vec3(0.0f);
    glm::vec3 normal = glm::vec3(0.0f);

    glm::vec3 firstPointOfIntersectedLine = glm::vec3(0.0f);
    glm::vec3 secondPointOfIntersectedLine = glm::vec3(0.0f);

    glm::vec3 rayStartPoint = glm::vec3(0.0f);
    glm::vec3 rayEndPoint = glm::vec3(0.0f);
};

struct HalfEdgeVertex
{
    glm::vec3 position;
    int halfEdgeIndex = -1; // Index of one of the half-edges originating from this vertex
};
struct HalfEdgeFace
{
    int halfEdgeIndex = -1;
};
struct HalfEdge
{
    int originVertexIndexID = -1;
    int twinHalfEdgeIndexID = -1;
    int nextHalfEdgeIndexID = -1;
    int faceIndexID = -1;
};

class NavMesh
{
public:
    NavMesh(Scene& scene);
    ~NavMesh();
    
    void BuildNavMesh();
    void RenderDebugTool(Shader* shader, Camera& camera, const Scene& scene);
private:
    void BuildBorderFromParams();
    void CreateDebugger();
    
    void EarClipping();
    void CreateHalfEdgeStructure();
    void FindTwinHalfEdges();
    
    Scene& m_Scene;
    NavMeshDebugger* m_NavMeshDebugger;
    NavBuildParams m_BuildParams;
    std::vector<glm::vec3> m_BorderVerts3D;
    std::vector<glm::vec3> m_NavMeshVerts3D;
    std::vector<NavMeshTriangle> m_NavMeshTriangles;

    std::vector<HalfEdgeVertex> m_HalfEdgeVertices;
    std::vector<HalfEdgeFace> m_HalfEdgeFaces;
    std::vector<HalfEdge> m_HalfEdges;

    static bool IsPointInTriangleXZ(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);
    static bool CanClipEar(int prevIndex, int earIndex, int nextIndex, const std::vector<glm::vec3>& vertices);
    int FindHalfEdgeIndex(const glm::vec3& pos);
    
    static bool IntersectLineSegmentsXZ(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& q1, const glm::vec3& q2, float& outLambda1, float& outLambda2, float epsilon = 1e-6f);
    static bool RaycastXZ(const std::vector<glm::vec3>& vertices, const glm::vec3& rayP1, const glm::vec3& rayP2, NavMeshHitInfo& hitInfo);
    std::vector<std::vector<glm::vec3>> GetSceneObstacleSlices(float buildPlaneY) const;
    void SortObstaclesByMaxX(std::vector<std::vector<glm::vec3>>& obstacleSlices);
    void InsertObstacle(const std::vector<glm::vec3>& obstacleSlice);
};
