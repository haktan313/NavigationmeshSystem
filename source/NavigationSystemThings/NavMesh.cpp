#include "NavMesh.h"
#include <glm/gtc/epsilon.hpp>
#include <algorithm>
#include <deque>

static bool IsItEqual(const glm::vec3& a, const glm::vec3& b, float epsilon = 1e-5f)
{
    return glm::all(glm::epsilonEqual(a, b, epsilon));
}
static float CrossProductXZ(const glm::vec3& v1, const glm::vec3& v2)
{
    // (x1*z2 - z1*x2) -> (x1*y2 - y1*x2) 
    return v1.x * v2.z - v1.z * v2.x;
}

NavMesh::NavMesh(const Scene& scene) : m_NavMeshDebugger(nullptr), m_Scene(scene), m_Start(glm::vec3(0.f, 0.0f,0.0f)), m_End(glm::vec3(-1.f,0.0f,-1.0f))
{
    CreateDebugger();
    BuildBorderFromParams();
}

NavMesh::~NavMesh()
{
    delete m_NavMeshDebugger;
    m_NavMeshDebugger = nullptr;
    
}

void NavMesh::BuildBorderFromParams()
{
    m_BorderVerts3D.clear();
    
    glm::vec3 origin = m_BuildParams.origin;
    glm::vec3 maxBounds = m_BuildParams.maxBounds;
    const float radius = m_BuildParams.agentRadius;

    glm::vec3 minPosition
    {
        std::min(origin.x,maxBounds.x) + radius,
        std::min(origin.y,maxBounds.y),
        std::min(origin.z,maxBounds.z) + radius
    };
    glm::vec3 maxPosition
    {
        std::max(origin.x,maxBounds.x) - radius,
        std::max(origin.y,maxBounds.y),
        std::max(origin.z,maxBounds.z) - radius
    };
    float y = minPosition.y;
    
    // CCW: (minX,minZ) -> (maxX,minZ) -> (maxX,maxZ) -> (minX,maxZ)
    glm::vec3 p0{minPosition.x, y, minPosition.z};
    glm::vec3 p1{maxPosition.x, y, minPosition.z};
    glm::vec3 p2{maxPosition.x, y, maxPosition.z};
    glm::vec3 p3{minPosition.x, y, maxPosition.z};

    m_BorderVerts3D = { p0, p1, p2, p3 };
    if (m_NavMeshDebugger)
        m_NavMeshDebugger->SetBorderVertices(m_BorderVerts3D);
    m_NavMeshVerts3D = m_BorderVerts3D;
}

void NavMesh::BuildNavMesh()
{
    m_NavMeshDebugger->CleanBuffers();
    m_NavMeshDebugger->m_bNavMeshBuilt = true;
    m_NavMeshTriangles.clear();
    m_NavMeshDebugger->SetTrianglesFill(m_NavMeshTriangles);
    
    const float buildPlaneY = std::min(m_BuildParams.origin.y, m_BuildParams.maxBounds.y);
    BuildBorderFromParams(); 
    m_NavMeshVerts3D = m_BorderVerts3D;
    std::vector<std::vector<glm::vec3>> obstacleSlices = GetSceneObstacleSlices(buildPlaneY);

    if (m_NavMeshDebugger)
    {
        m_NavMeshDebugger->SetHoles(obstacleSlices);
    }
    
    SortObstaclesByMaxX(obstacleSlices);
    for (const auto& slice : obstacleSlices)
        InsertObstacle(slice);
    
    //m_NavMeshDebugger->SetBorderVertices(m_NavMeshVerts3D);
    EarClipping();
    CreateHalfEdgeStructure();
    OptimizeEarClipping();
}

//----- Render the debug tool -----

void NavMesh::RenderDebugTool(Shader* shader, Camera& camera, const Scene& scene)
{
    if (m_NavMeshDebugger)
        m_NavMeshDebugger->RenderDebugTool(shader, camera, scene, m_DebugInfo);
}

void NavMesh::CreateDebugger()
{
    m_NavMeshDebugger = new NavMeshDebugger();
    if (m_NavMeshDebugger)
        SetStartEndMarkers(m_Start, m_End);
}

void NavMesh::SetStartEndMarkers(const glm::vec3& start, const glm::vec3& end)
{
    m_Start = start;
    m_End = end;
    if (m_NavMeshDebugger)
        m_NavMeshDebugger->SetStartEndMarkers(m_Start, m_End);
}

//----- Build Functions -----

void NavMesh::EarClipping()
{
    bool bClippedEar = false;
    //std::vector<glm::vec3> connections;
    while (m_NavMeshVerts3D.size() >= 3)
    {
        bClippedEar = false;
        for (int i = 0; i < m_NavMeshVerts3D.size(); i++)
        {
            int prevIndex = i - 1;
            if (prevIndex < 0)
                prevIndex += m_NavMeshVerts3D.size();
            int nextIndex = (i + 1) % m_NavMeshVerts3D.size();

            std::vector<glm::vec2> triangleDummy
            {
                {m_NavMeshVerts3D[prevIndex].x, m_NavMeshVerts3D[prevIndex].z},
                {m_NavMeshVerts3D[i].x, m_NavMeshVerts3D[i].z},
                {m_NavMeshVerts3D[nextIndex].x, m_NavMeshVerts3D[nextIndex].z}
            };
            glm::vec2 vector1 = {triangleDummy[1].x - triangleDummy[0].x, triangleDummy[1].y - triangleDummy[0].y};
            glm::vec2 vector2 = {triangleDummy[2].x - triangleDummy[1].x, triangleDummy[2].y - triangleDummy[1].y};
            float cross = vector1.x * vector2.y - vector1.y * vector2.x;
            
            if (cross > 0.f)
            {
                if (CanClipEar(prevIndex, i, nextIndex, m_NavMeshVerts3D))
                {
                    if (m_NavMeshDebugger)
                    {
                        std::vector<glm::vec3> connection
                        {
                            m_NavMeshVerts3D[prevIndex],
                            m_NavMeshVerts3D[nextIndex]
                        };
                        NavMeshTriangle triangle = 
                        {
                            m_NavMeshVerts3D[prevIndex],
                            m_NavMeshVerts3D[i],
                            m_NavMeshVerts3D[nextIndex]
                        };
                        m_NavMeshTriangles.push_back(triangle);
                        //connections.insert(connections.end(), connection.begin(), connection.end());
                    }
                    bClippedEar = true;
                    m_NavMeshVerts3D.erase(m_NavMeshVerts3D.begin() + i);
                    break; 
                }
            }
        }
        if (!bClippedEar)
        {
            m_NavMeshDebugger->SetTrianglesFill(m_NavMeshTriangles);
            std::cerr << "Ear Clipping failed! No valid ear found." << std::endl;
            return;
        }
    }
    //m_NavMeshDebugger->SetEarClipping(connections);
    m_NavMeshDebugger->SetTrianglesFill(m_NavMeshTriangles);
}

void NavMesh::CreateHalfEdgeStructure()
{
    m_HalfEdgeVertices.clear();
    m_HalfEdgeFaces.clear();
    m_HalfEdges.clear();

    for (const auto& triangle : m_NavMeshTriangles)
    {
        int vertex0Index = FindHalfEdgeIndex(triangle.verts[0]);
        int vertex1Index = FindHalfEdgeIndex(triangle.verts[1]);
        int vertex2Index = FindHalfEdgeIndex(triangle.verts[2]);

        m_HalfEdgeFaces.push_back(HalfEdgeFace());
        int faceIndex = static_cast<int>(m_HalfEdgeFaces.size() - 1);
        HalfEdgeFace& newFace = m_HalfEdgeFaces.back();

        m_HalfEdges.push_back(HalfEdge());
        int halfEdge0Index = static_cast<int>(m_HalfEdges.size() - 1);
        m_HalfEdges.push_back(HalfEdge());
        int halfEdge1Index = static_cast<int>(m_HalfEdges.size() - 1);
        m_HalfEdges.push_back(HalfEdge());
        int halfEdge2Index = static_cast<int>(m_HalfEdges.size() - 1);

        HalfEdge& halfEdge0 = m_HalfEdges[halfEdge0Index];
        halfEdge0.originVertexIndexID = vertex0Index;
        halfEdge0.nextHalfEdgeIndexID = halfEdge1Index;
        halfEdge0.faceIndexID = faceIndex;

        HalfEdge& halfEdge1 = m_HalfEdges[halfEdge1Index];
        halfEdge1.originVertexIndexID = vertex1Index;
        halfEdge1.nextHalfEdgeIndexID = halfEdge2Index;
        halfEdge1.faceIndexID = faceIndex;

        HalfEdge& halfEdge2 = m_HalfEdges[halfEdge2Index];
        halfEdge2.originVertexIndexID = vertex2Index;
        halfEdge2.nextHalfEdgeIndexID = halfEdge0Index;
        halfEdge2.faceIndexID = faceIndex;

        newFace.halfEdgeIndex = halfEdge0Index;

        if (m_HalfEdgeVertices[vertex0Index].halfEdgeIndex == -1)
            m_HalfEdgeVertices[vertex0Index].halfEdgeIndex = halfEdge0Index;
        if (m_HalfEdgeVertices[vertex1Index].halfEdgeIndex == -1)
            m_HalfEdgeVertices[vertex1Index].halfEdgeIndex = halfEdge1Index;
        if (m_HalfEdgeVertices[vertex2Index].halfEdgeIndex == -1)
            m_HalfEdgeVertices[vertex2Index].halfEdgeIndex = halfEdge2Index;
    }
    std::cout << "Half-Edge structure created with " << m_HalfEdgeVertices.size() << " vertices, "
              << m_HalfEdges.size() << " half-edges, and " << m_HalfEdgeFaces.size() << " faces." << std::endl;
    FindTwinHalfEdges();
}

void NavMesh::FindTwinHalfEdges()
{
    std::map<std::pair<int, int>, int> edgeMap; // Key: (start, end vertex), Value: halfEdgeIndex
    for (size_t i = 0; i < m_HalfEdges.size(); ++i)
    {
        const HalfEdge& halfEdge = m_HalfEdges[i];
        int endVertexIndex = m_HalfEdges[halfEdge.nextHalfEdgeIndexID].originVertexIndexID;
        edgeMap[{halfEdge.originVertexIndexID, endVertexIndex}] = static_cast<int>(i);
    }
    
    int twinCount = 0;
    for (size_t i = 0; i < m_HalfEdges.size(); ++i)
    {
        HalfEdge& halfEdge = m_HalfEdges[i];
        if (halfEdge.twinHalfEdgeIndexID != -1)
            continue; // already assigned
        int endVertexIndex = m_HalfEdges[halfEdge.nextHalfEdgeIndexID].originVertexIndexID;
        
        auto twinValue = edgeMap.find({endVertexIndex, halfEdge.originVertexIndexID});
        if (twinValue != edgeMap.end())
        {
            int twinIndex = twinValue->second;
            HalfEdge& twinHalfEdge = m_HalfEdges[twinIndex];
            halfEdge.twinHalfEdgeIndexID = twinIndex;
            twinHalfEdge.twinHalfEdgeIndexID = static_cast<int>(i);
            twinCount++;
        }
    }
    std::cout << "Found and assigned " << twinCount << " twin half-edges." << std::endl;
    
    std::vector<glm::vec3> internalEdgesLines;
    for (size_t i = 0; i < m_HalfEdges.size(); ++i)
    {
        const HalfEdge& halfEdge = m_HalfEdges[i];
        
        if (halfEdge.twinHalfEdgeIndexID != -1)
        {
            if (i < halfEdge.twinHalfEdgeIndexID)
            {
                const glm::vec3& p1 = m_HalfEdgeVertices[halfEdge.originVertexIndexID].position;
                
                const HalfEdge& nextHalfEdge = m_HalfEdges[halfEdge.nextHalfEdgeIndexID];
                const glm::vec3& p2 = m_HalfEdgeVertices[nextHalfEdge.originVertexIndexID].position;
    
                internalEdgesLines.push_back(p1);
                internalEdgesLines.push_back(p2);
            }
        }
    }
    std::cout << "Found " << internalEdgesLines.size() / 2 << " internal edges." << std::endl;
    m_NavMeshDebugger->SetTwinEdges(internalEdgesLines);
}

void NavMesh::OptimizeEarClipping()
{
    if (m_HalfEdges.empty() || !m_NavMeshDebugger)
        return;
    
    std::vector<int> removableEdgeIndices;
    FoundRemovableEdgeIndices(removableEdgeIndices);

    MergeTriangles(removableEdgeIndices);

    std::map<int, int> faceIndexToNodeIndex;
    std::vector<std::vector<glm::vec3>> mergedPolygonsForDebug;
    CreatePathfindingNodes(faceIndexToNodeIndex, mergedPolygonsForDebug);
    FindNeighborsForPathfindingNodes(faceIndexToNodeIndex);
    
    if (m_NavMeshDebugger)
    {
        m_NavMeshDebugger->SetMergePolygons(mergedPolygonsForDebug);
        m_NavMeshDebugger->SetMergePolygonsFill(mergedPolygonsForDebug);
    }
}

//----- Utility Functions -----

bool NavMesh::IsPointInTriangleXZ(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c)
{
    float v0x = c.x - a.x;
    float v0z = c.z - a.z;
    
    float v1x = b.x - a.x;
    float v1z = b.z - a.z;
    
    float v2x = p.x - a.x;
    float v2z = p.z - a.z;

    float dot00 = v0x * v0x + v0z * v0z;
    float dot01 = v0x * v1x + v0z * v1z;
    float dot02 = v0x * v2x + v0z * v2z;
    
    float dot11 = v1x * v1x + v1z * v1z;
    float dot12 = v1x * v2x + v1z * v2z;

    float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
    float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    float v = (dot00 * dot12 - dot01 * dot02) * invDenom;
    
    const float epsilon = -1e-5f; 
    return (u >= epsilon) && (v >= epsilon) && (u + v <= 1.0f + epsilon);
}

bool NavMesh::CanClipEar(int prevIndex, int earIndex, int nextIndex, const std::vector<glm::vec3>& vertices)
{
    const glm::vec3& a = vertices[prevIndex];
    const glm::vec3& b = vertices[earIndex];
    const glm::vec3& c = vertices[nextIndex];

    for (size_t i = 0; i < vertices.size(); ++i)
    {
        if (i == prevIndex || i == earIndex || i == nextIndex)
            continue;
        
        const glm::vec3& currentPoint = vertices[i];

        if (IsItEqual(currentPoint, a) || IsItEqual(currentPoint, b) || IsItEqual(currentPoint, c))
            continue;

        if (IsPointInTriangleXZ(currentPoint, a, b, c))
            return false;
    }
    return true;
}

int NavMesh::FindHalfEdgeIndex(const glm::vec3& pos)
{
    for (size_t i = 0; i < m_HalfEdgeVertices.size(); ++i)
        if (IsItEqual(m_HalfEdgeVertices[i].position, pos))
            return static_cast<int>(i);
    
    HalfEdgeVertex newVertex;
    newVertex.position = pos;
    m_HalfEdgeVertices.push_back(newVertex);
    return static_cast<int>(m_HalfEdgeVertices.size() - 1);
}

void NavMesh::FoundRemovableEdgeIndices(std::vector<int>& outRemovableEdgeIndices)
{
    std::vector<glm::vec3> removableEdgeLines;
    std::vector<glm::vec3> essentialEdgeLines;

     for (size_t i = 0; i < m_HalfEdges.size(); ++i)
    {
        const HalfEdge& halfEdge = m_HalfEdges[i];
        
        if (halfEdge.twinHalfEdgeIndexID != -1 && i < halfEdge.twinHalfEdgeIndexID)
        {
            const HalfEdge& twinHe = m_HalfEdges[halfEdge.twinHalfEdgeIndexID];
            
            const glm::vec3& vA = m_HalfEdgeVertices[halfEdge.originVertexIndexID].position;
            const glm::vec3& vB = m_HalfEdgeVertices[twinHe.originVertexIndexID].position;
            
            // he -> nextHe -> nextNextHe
            // vA -> vB -> vC
            const HalfEdge& nextHe = m_HalfEdges[halfEdge.nextHalfEdgeIndexID];
            const HalfEdge& nextNextHe = m_HalfEdges[nextHe.nextHalfEdgeIndexID];
            const glm::vec3& vC = m_HalfEdgeVertices[nextNextHe.originVertexIndexID].position;
            
            // twinHe -> twinNextHe -> twinNextNextHe
            // vB -> vA -> vD
            const HalfEdge& twinNextHe = m_HalfEdges[twinHe.nextHalfEdgeIndexID];
            const HalfEdge& twinNextNextHe = m_HalfEdges[twinNextHe.nextHalfEdgeIndexID];
            const glm::vec3& vD = m_HalfEdgeVertices[twinNextNextHe.originVertexIndexID].position;
            
            // vA -> vC -> vB -> vD -> vA
            // Face 1 = A-B-C Face 2 = B-A-D
            glm::vec3 vAC = vC - vA;
            glm::vec3 vCB = vB - vC;
            glm::vec3 vBD = vD - vB;
            glm::vec3 vDA = vA - vD;

            float cross1 = CrossProductXZ(vAC, vCB);
            float cross2 = CrossProductXZ(vCB, vBD);
            float cross3 = CrossProductXZ(vBD, vDA);
            float cross4 = CrossProductXZ(vDA, vAC);
            
            const float epsilon = 1e-5f;
            bool bIsConvex = (cross1 > epsilon && cross2 > epsilon && cross3 > epsilon && cross4 > epsilon) ||
                             (cross1 < -epsilon && cross2 < -epsilon && cross3 < -epsilon && cross4 < -epsilon);
            
            if (bIsConvex)
            {
                removableEdgeLines.push_back(vA);
                removableEdgeLines.push_back(vB);
                outRemovableEdgeIndices.push_back(static_cast<int>(i));
            }
            else
            {
                essentialEdgeLines.push_back(vA);
                essentialEdgeLines.push_back(vB);
            }
        }
    }

    std::cout << "Found " << removableEdgeLines.size() / 2 << " removable (green) edges." << std::endl;
    std::cout << "Found " << essentialEdgeLines.size() / 2 << " essential (red) edges." << std::endl;
    m_NavMeshDebugger->SetRemovableEdges(removableEdgeLines);
    m_NavMeshDebugger->SetCannotRemoveEdges(essentialEdgeLines);
}

void NavMesh::MergeTriangles(const std::vector<int>& removableEdgeIndices)
{
    std::cout << "Starting merge... Found " << removableEdgeIndices.size() << " removable edges." << std::endl;
    
    for (auto& face : m_HalfEdgeFaces)
        face.bIsValid = true;

    for (int heIndex : removableEdgeIndices)
    {
        HalfEdge& halfEdge = m_HalfEdges[heIndex];
        int twinIndex = halfEdge.twinHalfEdgeIndexID;
        
        if (twinIndex == -1)
            continue; 
        
        HalfEdge& twinHalfEdge = m_HalfEdges[twinIndex];
        
        if (!m_HalfEdgeFaces[halfEdge.faceIndexID].bIsValid || !m_HalfEdgeFaces[twinHalfEdge.faceIndexID].bIsValid)
            continue; 
        
        int halfEdgePrevID = m_HalfEdges[halfEdge.nextHalfEdgeIndexID].nextHalfEdgeIndexID;
        int twinPrevID = m_HalfEdges[twinHalfEdge.nextHalfEdgeIndexID].nextHalfEdgeIndexID;

        HalfEdge& halfEdgePrev = m_HalfEdges[halfEdgePrevID];
        HalfEdge& twinPrev = m_HalfEdges[twinPrevID];
        
        halfEdgePrev.nextHalfEdgeIndexID = twinHalfEdge.nextHalfEdgeIndexID;
        twinPrev.nextHalfEdgeIndexID = halfEdge.nextHalfEdgeIndexID;
        
        int faceToKeepIndex = halfEdge.faceIndexID;
        int faceToKillIndex = twinHalfEdge.faceIndexID;

        int currentEdgeIdx = twinPrev.nextHalfEdgeIndexID; 
        while (currentEdgeIdx != halfEdgePrevID)
        {
            m_HalfEdges[currentEdgeIdx].faceIndexID = faceToKeepIndex;
            currentEdgeIdx = m_HalfEdges[currentEdgeIdx].nextHalfEdgeIndexID;
            
            if (currentEdgeIdx == twinPrev.nextHalfEdgeIndexID)
            {
                std::cerr << "Infinite loop detected in face merge!" << std::endl;
                break; 
            }
        }
        
        m_HalfEdgeFaces[faceToKeepIndex].halfEdgeIndex = halfEdgePrevID; 
        m_HalfEdgeFaces[faceToKillIndex].bIsValid = false;
        
        halfEdge.faceIndexID = -1;
        twinHalfEdge.faceIndexID = -1;
    }
    std::cout << "Merge complete." << std::endl;
}

void NavMesh::CreatePathfindingNodes(std::map<int, int>& faceIndexToNodeIndex, std::vector<std::vector<glm::vec3>>& mergedPolygonsForDebug)
{
    m_PathfindingNodes.clear();

    for (int i = 0; i < m_HalfEdgeFaces.size(); ++i)
    {
        const auto& face = m_HalfEdgeFaces[i];
        if (face.bIsValid)
        {
            NavMeshOptimizedNode newNode;
            newNode.originalFaceIndex = i;

            glm::vec3 centerSum(0.0f);
            std::vector<glm::vec3> polygonVerts;
            int startEdgeIdx = face.halfEdgeIndex;
            int currentEdgeIdx = startEdgeIdx;
            
            do
            {
                const HalfEdge& currentEdge = m_HalfEdges[currentEdgeIdx];
                const glm::vec3& vertPos = m_HalfEdgeVertices[currentEdge.originVertexIndexID].position;
                
                polygonVerts.push_back(vertPos);
                centerSum += vertPos;
                
                currentEdgeIdx = currentEdge.nextHalfEdgeIndexID;
            }
            while (currentEdgeIdx != startEdgeIdx);

            newNode.polygonVerts = polygonVerts;
            if (!polygonVerts.empty())
            {
                newNode.centerPoint = centerSum / static_cast<float>(polygonVerts.size());
            }
            m_PathfindingNodes.push_back(newNode);
            faceIndexToNodeIndex[i] = static_cast<int>(m_PathfindingNodes.size() - 1);
            
            mergedPolygonsForDebug.push_back(polygonVerts);
        }
    }
    
    std::cout << "Found " << mergedPolygonsForDebug.size() << " final merged polygons." << std::endl;
}

void NavMesh::FindNeighborsForPathfindingNodes(std::map<int, int>& faceIndexToNodeIndex)
{
    for (int i = 0; i < m_PathfindingNodes.size(); ++i)
    {
        NavMeshOptimizedNode& node = m_PathfindingNodes[i];
        const auto& face = m_HalfEdgeFaces[node.originalFaceIndex];

        int startEdgeIdx = face.halfEdgeIndex;
        int currentEdgeIdx = startEdgeIdx;

        do
        {
            const HalfEdge& currentEdge = m_HalfEdges[currentEdgeIdx];
            int twinIndex = currentEdge.twinHalfEdgeIndexID;

            if (twinIndex != -1)
            {
                const HalfEdge& twinEdge = m_HalfEdges[twinIndex];
                int neighborFaceOrigIndex = twinEdge.faceIndexID;
                
                if (neighborFaceOrigIndex != -1 && m_HalfEdgeFaces[neighborFaceOrigIndex].bIsValid)
                {
                    auto it = faceIndexToNodeIndex.find(neighborFaceOrigIndex);
                    if (it != faceIndexToNodeIndex.end())
                    {
                        OptimizedEdge newNeighborEdge;
                        newNeighborEdge.neighborFaceIndex = it->second;

                        newNeighborEdge.edgeStart = m_HalfEdgeVertices[currentEdge.originVertexIndexID].position;
                        newNeighborEdge.edgeEnd = m_HalfEdgeVertices[twinEdge.originVertexIndexID].position;
                        
                        node.neighbors.push_back(newNeighborEdge);
                    }
                }
            }
            
            currentEdgeIdx = currentEdge.nextHalfEdgeIndexID;
        }
        while (currentEdgeIdx != startEdgeIdx);
    }

    std::cout << "Populated neighbors for all " << m_PathfindingNodes.size() << " nodes." << std::endl;
}

//----- Raycasting Functions -----

bool NavMesh::IntersectLineSegmentsXZ(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& q1,
    const glm::vec3& q2, float& outLambda1, float& outLambda2, float epsilon)
{
    const glm::vec3 p1p2 = p2 - p1;
    const glm::vec3 q1q2 = q2 - q1;
    
    const float denom = CrossProductXZ(p1p2, q1q2);
    
    if (std::abs(denom) > epsilon)
    {
        const glm::vec3 p1q1 = q1 - p1;

        const float num1 = CrossProductXZ(p1q1, q1q2);
        const float num2 = CrossProductXZ(p1q1, p1p2);
        
        outLambda1 = num1 / denom;
        outLambda2 = num2 / denom;
        
        return true;
    }
    
    outLambda1 = 0.0f;
    outLambda2 = 0.0f;
    return false;
}

bool NavMesh::RaycastXZ(const std::vector<glm::vec3>& vertices, const glm::vec3& rayP1, const glm::vec3& rayP2, NavMeshHitInfo& hitInfo)
{
    if (vertices.empty())
        return false;

    std::vector<NavMeshHitInfo> hits;
    
    float rayMinX = std::min(rayP1.x, rayP2.x);
    float rayMaxX = std::max(rayP1.x, rayP2.x);
    float rayMinZ = std::min(rayP1.z, rayP2.z);
    float rayMaxZ = std::max(rayP1.z, rayP2.z);
    
    for (size_t i = 0; i < vertices.size(); ++i)
    {
        const glm::vec3& q1 = vertices[i];
        const glm::vec3& q2 = vertices[(i + 1) % vertices.size()];
        
        float lineMinX = std::min(q1.x, q2.x);
        float lineMaxX = std::max(q1.x, q2.x);
        float lineMinZ = std::min(q1.z, q2.z);
        float lineMaxZ = std::max(q1.z, q2.z);
        
        if (rayMaxX < lineMinX || rayMinX > lineMaxX || rayMaxZ < lineMinZ || rayMinZ > lineMaxZ)
            continue;
        
        float lambda1 = 0.0f;
        float lambda2 = 0.0f;
        if (IntersectLineSegmentsXZ(rayP1, rayP2, q1, q2, lambda1, lambda2))
        {
            if (lambda1 > 1e-6f && lambda1 <= 1.0f && lambda2 > 1e-6f && lambda2 <= 1.0f)
            {
                NavMeshHitInfo currentHit;
                currentHit.lambda = lambda1;
                
                currentHit.intersectPoint = rayP1 + (rayP2 - rayP1) * lambda1;
                
                glm::vec3 lineVec = q2 - q1;
                currentHit.normal = glm::normalize(glm::vec3(-lineVec.z, 0.0f, lineVec.x));
                
                currentHit.firstPointOfIntersectedLine = q1;
                currentHit.secondPointOfIntersectedLine = q2;
                currentHit.rayStartPoint = rayP1;
                currentHit.rayEndPoint = rayP2;

                hits.push_back(currentHit);
            }
        }
    }

    if (hits.empty())
        return false;
    
    auto closestHit =
        std::min_element(hits.begin(), hits.end(),[](const NavMeshHitInfo& a, const NavMeshHitInfo& b)
        {
            return a.lambda < b.lambda;
        }
    );

    hitInfo = *closestHit;
    return true;
}

std::vector<std::vector<glm::vec3>> NavMesh::GetSceneObstacleSlices(float buildPlaneY) const
{
    std::vector<std::vector<glm::vec3>> allSlices;
    const float radius = m_BuildParams.agentRadius;
    
    const std::vector<Vec3f> localCubeVerts = {
        {-0.5f, -0.5f, -0.5f}, // 0
        { 0.5f, -0.5f, -0.5f}, // 1
        { 0.5f,  0.5f, -0.5f}, // 2
        {-0.5f,  0.5f, -0.5f}, // 3
        {-0.5f, -0.5f,  0.5f}, // 4
        { 0.5f, -0.5f,  0.5f}, // 5
        { 0.5f,  0.5f,  0.5f}, // 6
        {-0.5f,  0.5f,  0.5f}  // 7
    };
    
    for (const auto& object : m_Scene.GetObjects())
    {
        glm::vec3 minBounds(std::numeric_limits<float>::max());
        glm::vec3 maxBounds(std::numeric_limits<float>::lowest());
        
        for (const auto& localVert : localCubeVerts)
        {
            glm::vec4 worldPos = object.modelMatrix * glm::vec4(localVert.x, localVert.y, localVert.z, 1.0f);
            
            minBounds.x = std::min(minBounds.x, worldPos.x);
            minBounds.y = std::min(minBounds.y, worldPos.y);
            minBounds.z = std::min(minBounds.z, worldPos.z);
            
            maxBounds.x = std::max(maxBounds.x, worldPos.x);
            maxBounds.y = std::max(maxBounds.y, worldPos.y);
            maxBounds.z = std::max(maxBounds.z, worldPos.z);
        }
        
        if (buildPlaneY < minBounds.y || buildPlaneY > maxBounds.y)
            continue;

        maxBounds.x += radius;
        maxBounds.z += radius;
        minBounds.x -= radius;
        minBounds.z -= radius;
        
        std::vector<glm::vec3> sliceFootprint;
        const float y = buildPlaneY;
        
        // (minX, minZ) -> (maxX, minZ) -> (maxX, maxZ) -> (minX, maxZ)
        sliceFootprint.push_back(glm::vec3(minBounds.x, y, minBounds.z));
        sliceFootprint.push_back(glm::vec3(maxBounds.x, y, minBounds.z));
        sliceFootprint.push_back(glm::vec3(maxBounds.x, y, maxBounds.z));
        sliceFootprint.push_back(glm::vec3(minBounds.x, y, maxBounds.z));
        
        allSlices.push_back(sliceFootprint);
    }

    return allSlices;
}

void NavMesh::SortObstaclesByMaxX(std::vector<std::vector<glm::vec3>>& obstacleSlices)
{
    auto compareX = [](const glm::vec3& a, const glm::vec3& b)
    {
        return a.x < b.x;
    };
    
    std::sort(obstacleSlices.begin(), obstacleSlices.end(), [&compareX](const std::vector<glm::vec3>& lhs, const std::vector<glm::vec3>& rhs) 
    {
        auto max_it_lhs = std::max_element(lhs.begin(), lhs.end(), compareX);
        auto max_it_rhs = std::max_element(rhs.begin(), rhs.end(), compareX);
        
        if (max_it_lhs == lhs.end())
            return false;
        if (max_it_rhs == rhs.end())
            return true;
        
        return max_it_lhs->x > max_it_rhs->x;
    });
}

void NavMesh::InsertObstacle(const std::vector<glm::vec3>& obstacleSlice)
{
    if (obstacleSlice.empty())
        return;
    
    auto mostRightPointIt =
        std::max_element(obstacleSlice.begin(), obstacleSlice.end(),[](const glm::vec3& a, const glm::vec3& b)
        {
            return a.x < b.x;
        });

    const glm::vec3 mostRightPoint = *mostRightPointIt;
    
    const glm::vec3 rayP1 = mostRightPoint;
    const glm::vec3 rayP2 = mostRightPoint + glm::vec3(10000.0f, 0.0f, 0.0f);
    NavMeshHitInfo hitInfo;

    if (!RaycastXZ(m_NavMeshVerts3D, rayP1, rayP2, hitInfo))
    {
        std::cerr << "Obstacle line didnt hit border" << std::endl;
        return;
    }
    
    //m_NavMeshDebugger->SetEarClipping({ rayP1, hitInfo.intersectPoint });
    
    const glm::vec3 connectToPos = hitInfo.firstPointOfIntersectedLine;

    std::vector<size_t> indices;
    for (size_t i = 0; i < m_NavMeshVerts3D.size(); ++i)
        if (IsItEqual(m_NavMeshVerts3D[i], connectToPos))
            indices.push_back(i);
        

    if (indices.empty())
    {
        std::cerr << "The hit point is not found in the main list" << std::endl;
        return;
    }

    size_t connectToIndex = indices.back();
    const float mostRightPointZ = mostRightPoint.z;

    for (size_t index : indices)
    {
        size_t nextIndex = (index + 1) % m_NavMeshVerts3D.size();
        if (m_NavMeshVerts3D[nextIndex].z > mostRightPointZ)
        {
            connectToIndex = index;
            break;
        }
    }
    
    auto connectToIt = m_NavMeshVerts3D.begin() + connectToIndex;
    
    std::vector<glm::vec3> verticesToInsert = obstacleSlice;
    
    std::reverse(verticesToInsert.begin(), verticesToInsert.end());
    auto mostRightPointIt_CW = std::find_if(verticesToInsert.begin(), verticesToInsert.end(), 
        [&mostRightPoint](const glm::vec3& v){ return IsItEqual(v, mostRightPoint); });
    if (mostRightPointIt_CW == verticesToInsert.end())
    {
        std::cerr << "Internal error: mostRightPoint not found after reverse." << std::endl;
        return; 
    }
    std::rotate(verticesToInsert.begin(), mostRightPointIt_CW, verticesToInsert.end());
    
    verticesToInsert.push_back(verticesToInsert[0]); 
    verticesToInsert.push_back(*connectToIt);
    
    m_NavMeshVerts3D.insert(connectToIt + 1, verticesToInsert.begin(), verticesToInsert.end());
}
