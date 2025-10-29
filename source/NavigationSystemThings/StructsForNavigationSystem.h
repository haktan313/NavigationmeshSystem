
#pragma once
#include <glm/vec3.hpp>

struct NavBuildParams
{
    glm::vec3 origin = glm::vec3(-15.f, 0.4f, -15.f);
    glm::vec3 maxBounds = glm::vec3(15.f, 2.f, 15.f);
    float agentRadius = 0.2f;
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
    bool bIsValid = true;
};
struct HalfEdge
{
    int originVertexIndexID = -1;
    int twinHalfEdgeIndexID = -1;
    int nextHalfEdgeIndexID = -1;
    int faceIndexID = -1;
};

struct OptimizedEdge
{
    int neighborFaceIndex = -1;
    glm::vec3 edgeStart;
    glm::vec3 edgeEnd;
};
struct NavMeshOptimizedNode
{
    int originalFaceIndex = -1;
    glm::vec3 centerPoint;
    std::vector<glm::vec3> polygonVerts;
    std::vector<OptimizedEdge> neighbors;
};

struct DrawDebugInfo
{
    bool bDrawBorder = true;
    bool bDrawHoles = true;
    bool bFillEmptyAreas = false;
    bool bDrawTriangles = false;
    bool bDrawMergePolygons = true;
    
    bool bDrawTwinEdges = false;
    bool bDrawRemovableEdges = false;
    bool bDrawCannotRemoveEdges = false;
};