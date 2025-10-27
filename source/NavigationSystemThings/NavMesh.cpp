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

NavMesh::NavMesh(Scene& scene) : m_NavMeshDebugger(nullptr), m_Scene(scene)
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

    glm::vec3 minPosition
    {
        std::min(origin.x,maxBounds.x),
        std::min(origin.y,maxBounds.y),
        std::min(origin.z,maxBounds.z)
    };
    glm::vec3 maxPosition
    {
        std::max(origin.x,maxBounds.x),
        std::max(origin.y,maxBounds.y),
        std::max(origin.z,maxBounds.z)
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
    m_NavMeshTriangles.clear();
    m_NavMeshDebugger->SetTriangles(m_NavMeshTriangles);
    
    const float buildPlaneY = std::min(m_BuildParams.origin.y, m_BuildParams.maxBounds.y);
    BuildBorderFromParams(); 
    m_NavMeshVerts3D = m_BorderVerts3D;
    std::vector<std::vector<glm::vec3>> obstacleSlices = GetSceneObstacleSlices(buildPlaneY);
    
    SortObstaclesByMaxX(obstacleSlices);
    for (const auto& slice : obstacleSlices)
        InsertObstacle(slice);
    
    m_NavMeshDebugger->SetBorderVertices(m_NavMeshVerts3D);
    EarClipping();
}

//----- Render the debug tool -----

void NavMesh::RenderDebugTool(Shader* shader, Camera& camera, const Scene& scene)
{
    if (m_NavMeshDebugger)
        m_NavMeshDebugger->RenderDebugTool(shader, camera, scene);
}

void NavMesh::CreateDebugger()
{
    m_NavMeshDebugger = new NavMeshDebugger();
}


//----- Build Functions -----

void NavMesh::EarClipping()
{
    bool bClippedEar = false;
    std::vector<glm::vec3> connections;
    while (m_NavMeshVerts3D.size() >= 3)
    {
        bClippedEar = false;
        for (int i = 0; i < m_NavMeshVerts3D.size(); i++)
        {
            int prevIndex = i - 1;
            if (prevIndex < 0)
                prevIndex += m_NavMeshVerts3D.size();
            int nextIndex = (i + 1) % m_NavMeshVerts3D.size();

            std::vector<glm::vec2> triangles
            {
                {m_NavMeshVerts3D[prevIndex].x, m_NavMeshVerts3D[prevIndex].z},
                {m_NavMeshVerts3D[i].x, m_NavMeshVerts3D[i].z},
                {m_NavMeshVerts3D[nextIndex].x, m_NavMeshVerts3D[nextIndex].z}
            };
            glm::vec2 vector1 = {triangles[1].x - triangles[0].x, triangles[1].y - triangles[0].y};
            glm::vec2 vector2 = {triangles[2].x - triangles[1].x, triangles[2].y - triangles[1].y};
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
                        connections.insert(connections.end(), connection.begin(), connection.end());
                    }
                    bClippedEar = true;
                    m_NavMeshVerts3D.erase(m_NavMeshVerts3D.begin() + i);
                    break; 
                }
            }
        }
        if (!bClippedEar)
        {
            m_NavMeshDebugger->SetEarClipping(connections);
            m_NavMeshDebugger->SetTriangles(m_NavMeshTriangles);
            std::cerr << "Ear Clipping failed! No valid ear found." << std::endl;
            return;
        }
    }
    m_NavMeshDebugger->SetEarClipping(connections);
    m_NavMeshDebugger->SetTriangles(m_NavMeshTriangles);
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
