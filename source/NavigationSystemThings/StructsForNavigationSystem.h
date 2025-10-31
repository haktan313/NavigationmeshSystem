
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
    float proccessOnRay = 0.0f;
    glm::vec3 hitPoint = glm::vec3(0.0f);
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
    int originVertexIndex = -1;
    int nextHalfEdgeIndex = -1;
    int faceID = -1;
    int twinHalfEdgeIndex = -1;
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