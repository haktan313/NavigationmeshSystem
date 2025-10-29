#include "NavMeshDebugger.h"
#include "NavMesh.h"


NavMeshDebugger::NavMeshDebugger()
    :  m_BorderVAO(0), m_BorderVBO(0), m_BorderCount(0), m_TriangleVAO(0), m_TriangleVBO(0), m_TriangleCount(0), m_EarClipingVAO(0), m_EarClippingVBO(0),
        m_EarClipCount(0), m_MergedPolygonFillVAO(0), m_MergedPolygonFillVBO(0), m_TwinEdgeVAO(0), m_TwinEdgeVBO(0), m_TwinEdgeCount(0), m_RemovableEdgeVAO(0),
            m_RemovableEdgeVBO(0), m_RemovableEdgeCount(0), m_CannotRemoveEdgeVAO(0), m_CannotRemoveEdgeVBO(0), m_CannotRemoveEdgeCount(0),
                m_StartEndMarkerCount(0), m_StartEndMarkerVAO(0), m_StartEndMarkerVBO(0), m_PathPointCount(0), m_PathPointVAO(0), m_PathPointVBO(0), bNavMeshBuilt(false)
{

}

NavMeshDebugger::~NavMeshDebugger()
{
    CleanBuffers();
}
void NavMeshDebugger::CleanBuffers()
{
    if (m_BorderVAO)
        glDeleteVertexArrays(1, &m_BorderVAO);
    if (m_BorderVBO)
        glDeleteBuffers(1, &m_BorderVBO);
    for (auto vao : m_HoleVAOs)
        glDeleteVertexArrays(1, &vao);
    if (m_TriangleVAO)
        glDeleteVertexArrays(1, &m_TriangleVAO);
    if (m_TriangleVBO)
        glDeleteBuffers(1, &m_TriangleVBO);
    
    glDeleteVertexArrays(1, &m_EarClipingVAO);
    glDeleteBuffers(1, &m_EarClippingVBO);
    for (auto vao : m_MergedPolygonVAOs)
        glDeleteVertexArrays(1, &vao);
    if (m_MergedPolygonFillVAO)
        glDeleteVertexArrays(1, &m_MergedPolygonFillVAO);
    if (m_MergedPolygonFillVBO)
        glDeleteBuffers(1, &m_MergedPolygonFillVBO);
    
    if (m_TwinEdgeVAO)
        glDeleteVertexArrays(1, &m_TwinEdgeVAO);
    if (m_TwinEdgeVBO)
        glDeleteBuffers(1, &m_TwinEdgeVBO);
    if (m_RemovableEdgeVAO)
        glDeleteVertexArrays(1, &m_RemovableEdgeVAO);
    if (m_RemovableEdgeVBO)
        glDeleteBuffers(1, &m_RemovableEdgeVBO);
    if (m_CannotRemoveEdgeVAO)
        glDeleteVertexArrays(1, &m_CannotRemoveEdgeVAO);
    if (m_CannotRemoveEdgeVBO)
        glDeleteBuffers(1, &m_CannotRemoveEdgeVBO);

    if (m_StartEndMarkerVAO)
        glDeleteVertexArrays(1, &m_StartEndMarkerVAO);
    if (m_StartEndMarkerVBO)
        glDeleteBuffers(1, &m_StartEndMarkerVBO);
    if (m_PathPointVAO)
        glDeleteVertexArrays(1, &m_PathPointVAO);
    if (m_PathPointVBO)
        glDeleteBuffers(1, &m_PathPointVBO);
}

void NavMeshDebugger::RenderDebugTool(Shader* shader, Camera& camera, const Scene& scene, const DrawDebugInfo& debugInfo)
{
    shader->use();
    glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), 1280.0f/720.0f, 0.1f, 100.0f);
    glm::mat4 view = camera.GetViewMatrix();
    shader->setMat4("projection", projection);
    shader->setMat4("view", view);
    shader->setMat4("model", glm::mat4(1.0f));
    RenderStartEndMarkers(shader);

    if (debugInfo.bDrawBorder)
        RenderBorder(shader);
    if (!bNavMeshBuilt)
        return;
    if (debugInfo.bDrawHoles)
        RenderHoles(shader);
    if (debugInfo.bFillEmptyAreas && !debugInfo.bDrawTriangles && !debugInfo.bDrawMergePolygons)
        FillSpaces(shader);
    if (debugInfo.bDrawTriangles)
        RenderEarClipping(shader);
    if (debugInfo.bDrawMergePolygons)
        RenderMergePolygons(shader);
    if (debugInfo.bDrawTwinEdges)
        RenderTwinEdges(shader);
    if (debugInfo.bDrawRemovableEdges)
        RenderRemovableEdges(shader);
    if (debugInfo.bDrawCannotRemoveEdges)
        RenderCannotRemoveEdges(shader);
}

void NavMeshDebugger::SetBorderVertices(const std::vector<glm::vec3>& borderVerts)
{
    m_BorderCount = static_cast<GLsizei>(borderVerts.size());
    if (m_BorderCount == 0)
        return;

    if (m_BorderVAO == 0)
    {
        glGenVertexArrays(1, &m_BorderVAO);
        glGenBuffers(1, &m_BorderVBO);
    }

    glBindVertexArray(m_BorderVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_BorderVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * m_BorderCount, borderVerts.data(), GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0);//position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);

    glBindVertexArray(0);
}

void NavMeshDebugger::RenderBorder(Shader* shader)
{
    if (m_BorderVAO)
    {
        if (m_BorderCount >= 1)
        {
            shader->setVec4("ourColor", glm::vec4(0.1f, 0.4f, 1.0f, 1.0f));
            glPointSize(8.f);
            glBindVertexArray(m_BorderVAO);
            glDrawArrays(GL_POINTS, 0, m_BorderCount);
            glBindVertexArray(0);
        }
        if (m_BorderCount >= 2)
        {
            shader->setVec4("ourColor", glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
            glLineWidth(2.f);
            glBindVertexArray(m_BorderVAO);
            glDrawArrays(GL_LINE_LOOP, 0, m_BorderCount);
            glBindVertexArray(0);
        }
    }
}

void NavMeshDebugger::SetHoles(const std::vector<std::vector<glm::vec3>>& holeVerts)
{
    for (unsigned int vao : m_HoleVAOs)
        glDeleteVertexArrays(1, &vao);
    m_HoleVAOs.clear();
    m_HoleCounts.clear();

    for (const auto& polygonVerts : holeVerts)
    {
        GLsizei count = static_cast<GLsizei>(polygonVerts.size());
        if (count == 0) continue;

        unsigned int vao, vbo;
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * count, polygonVerts.data(), GL_DYNAMIC_DRAW);
        
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
        
        glBindVertexArray(0);
        glDeleteBuffers(1, &vbo);

        m_HoleVAOs.push_back(vao);
        m_HoleCounts.push_back(count);
    }
}

void NavMeshDebugger::RenderHoles(Shader* shader)
{
    if (m_HoleVAOs.empty())
        return;

    for (size_t i = 0; i < m_HoleVAOs.size(); ++i)
    {
        shader->setVec4("ourColor", glm::vec4(0.1f, 0.4f, 1.0f, 1.0f));
        glPointSize(8.f);
        glBindVertexArray(m_HoleVAOs[i]);
        glDrawArrays(GL_POINTS, 0, m_HoleCounts[i]);
        
        shader->setVec4("ourColor", glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
        glLineWidth(2.f); 
        glBindVertexArray(m_HoleVAOs[i]);
        glDrawArrays(GL_LINE_LOOP, 0, m_HoleCounts[i]);
    }
    glBindVertexArray(0);
}

void NavMeshDebugger::FillSpaces(Shader* shader)
{
    glEnable(GL_STENCIL_TEST);
    glStencilMask(0xFF);
    glClear(GL_STENCIL_BUFFER_BIT);
    
    glStencilFunc(GL_ALWAYS, 1, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glDepthMask(GL_FALSE);

    if (!m_HoleVAOs.empty())
    {
        for (size_t i = 0; i < m_HoleVAOs.size(); ++i)
        {
            glBindVertexArray(m_HoleVAOs[i]);
            glDrawArrays(GL_TRIANGLE_FAN, 0, m_HoleCounts[i]); 
        }
    }
    
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthMask(GL_TRUE);

    glStencilFunc(GL_NOTEQUAL, 1, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

    shader->setVec4("ourColor", glm::vec4(0.0f, 1.0f, 0.0f, 0.1f));

    if (m_BorderVAO)
    {
        glBindVertexArray(m_BorderVAO);
        glDrawArrays(GL_TRIANGLE_FAN, 0, m_BorderCount);
    }
    glBindVertexArray(0);

    glDisable(GL_STENCIL_TEST);
    glStencilMask(0x00);
}


void NavMeshDebugger::SetEarClipping(std::vector<glm::vec3> clippedEars)
{
    glGenVertexArrays(1, &m_EarClipingVAO);
    glGenBuffers(1, &m_EarClippingVBO);
    
    glBindVertexArray(m_EarClipingVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_EarClippingVBO);
    m_EarClipCount = static_cast<GLsizei>(clippedEars.size());
    
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * clippedEars.size(), clippedEars.data(), GL_DYNAMIC_DRAW);
    
    glEnableVertexAttribArray(0);//position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    
    glBindVertexArray(0);
}

void NavMeshDebugger::RenderEarClipping(Shader* shader)
{
    if (m_TwinEdgeVAO)
    {
        shader->setVec4("ourColor", glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
        glLineWidth(2.f);
        glBindVertexArray(m_TwinEdgeVAO);
        glDrawArrays(GL_LINES, 0, m_TwinEdgeCount);
        glBindVertexArray(0);
    }
    RenderBorder(shader);
    RenderTrianglesFill(shader);
}

void NavMeshDebugger::SetTrianglesFill(const std::vector<NavMeshTriangle>& triangles)
{
    glGenVertexArrays(1, &m_TriangleVAO);
    glGenBuffers(1, &m_TriangleVBO);

    glBindVertexArray(m_TriangleVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_TriangleVBO);

    m_TriangleCount = static_cast<GLsizei>(triangles.size() * 3);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * 3 * triangles.size() , triangles.data(), GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);//position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glBindVertexArray(0);
}

void NavMeshDebugger::RenderTrianglesFill(Shader* shader)
{
    if (m_TriangleVAO)
    {
        shader->setVec4("ourColor", glm::vec4(0.0f, 1.0f, 0.0f, 0.1f));
        glBindVertexArray(m_TriangleVAO);
        glDrawArrays(GL_TRIANGLES, 0, m_TriangleCount);
        glBindVertexArray(0);
    }
}

void NavMeshDebugger::SetMergePolygons(const std::vector<std::vector<glm::vec3>>& mergedPolygonVerts)
{
    for (unsigned int vao : m_MergedPolygonVAOs)
    {
        glDeleteVertexArrays(1, &vao);
    }
    m_MergedPolygonVAOs.clear();
    m_MergedPolygonCounts.clear();
    
    for (const auto& polygonVerts : mergedPolygonVerts)
    {
        GLsizei count = static_cast<GLsizei>(polygonVerts.size());
        if (count == 0)
            continue;

        unsigned int vao, vbo;
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo); 

        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * count, polygonVerts.data(), GL_DYNAMIC_DRAW);

        glEnableVertexAttribArray(0); // position
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);

        glBindVertexArray(0);
        
        glDeleteBuffers(1, &vbo);

        m_MergedPolygonVAOs.push_back(vao);
        m_MergedPolygonCounts.push_back(count);
    }
}

void NavMeshDebugger::RenderMergePolygons(Shader* shader)
{
    if (m_MergedPolygonVAOs.empty())
        return;
    
    shader->setVec4("ourColor", glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
    glLineWidth(2.f); 

    for (size_t i = 0; i < m_MergedPolygonVAOs.size(); ++i)
    {
        glBindVertexArray(m_MergedPolygonVAOs[i]);
        glDrawArrays(GL_LINE_LOOP, 0, m_MergedPolygonCounts[i]);
    }
    
    glBindVertexArray(0);
    glLineWidth(1.f);
    RenderMergePolygonsFill(shader);
}

void NavMeshDebugger::SetMergePolygonsFill(const std::vector<std::vector<glm::vec3>>& mergedPolygonVerts)
{
    std::vector<glm::vec3> fillTriangles;

    for (const auto& polygon : mergedPolygonVerts)
    {
        if (polygon.size() < 3)
            continue;
        
        // (v0, v1, v2), (v0, v2, v3), (v0, v3, v4), ...
        const glm::vec3& v0 = polygon[0];
        for (size_t i = 1; i < polygon.size() - 1; ++i)
        {
            const glm::vec3& v1 = polygon[i];
            const glm::vec3& v2 = polygon[i + 1];
            
            fillTriangles.push_back(v0);
            fillTriangles.push_back(v1);
            fillTriangles.push_back(v2);
        }
    }

    m_MergedPolygonFillCount = static_cast<GLsizei>(fillTriangles.size());
    if (m_MergedPolygonFillCount == 0)
        return;
    
    if (m_MergedPolygonFillVAO == 0)
    {
        glGenVertexArrays(1, &m_MergedPolygonFillVAO);
        glGenBuffers(1, &m_MergedPolygonFillVBO);
    }

    glBindVertexArray(m_MergedPolygonFillVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_MergedPolygonFillVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * m_MergedPolygonFillCount, fillTriangles.data(), GL_DYNAMIC_DRAW);
    
    glEnableVertexAttribArray(0); // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    
    glBindVertexArray(0);
}

void NavMeshDebugger::RenderMergePolygonsFill(Shader* shader)
{
    if (m_MergedPolygonFillVAO)
    {
        shader->setVec4("ourColor", glm::vec4(0.0f, 1.0f, 0.0f, 0.1f));
        glBindVertexArray(m_MergedPolygonFillVAO);
        glDrawArrays(GL_TRIANGLES, 0, m_MergedPolygonFillCount);
        glBindVertexArray(0);
    }
}



void NavMeshDebugger::SetTwinEdges(const std::vector<glm::vec3>& twinEdgeVerts)
{
    m_TwinEdgeCount = static_cast<GLsizei>(twinEdgeVerts.size());
    if (m_TwinEdgeCount == 0)
        return;

    if (m_TwinEdgeVAO == 0)
    {
        glGenVertexArrays(1, &m_TwinEdgeVAO);
        glGenBuffers(1, &m_TwinEdgeVBO);
    }

    glBindVertexArray(m_TwinEdgeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_TwinEdgeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * m_TwinEdgeCount, twinEdgeVerts.data(), GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);

    glBindVertexArray(0);
}

void NavMeshDebugger::RenderTwinEdges(Shader* shader)
{
    if (m_TwinEdgeVAO && m_TwinEdgeCount > 0)
    {
        shader->setVec4("ourColor", glm::vec4(1.0f, 1.0f, 1.0f, 1.0f)); 
        glLineWidth(4.f);
        glBindVertexArray(m_TwinEdgeVAO);
        glDrawArrays(GL_LINES, 0, m_TwinEdgeCount);
        glBindVertexArray(0);
        glLineWidth(1.f); 
    }
}

void NavMeshDebugger::SetRemovableEdges(const std::vector<glm::vec3>& removableEdgeVerts)
{
    m_RemovableEdgeCount = static_cast<GLsizei>(removableEdgeVerts.size());
    if (m_RemovableEdgeCount == 0)
        return;

    if (m_RemovableEdgeVAO == 0)
    {
        glGenVertexArrays(1, &m_RemovableEdgeVAO);
        glGenBuffers(1, &m_RemovableEdgeVBO);
    }

    glBindVertexArray(m_RemovableEdgeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_RemovableEdgeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * m_RemovableEdgeCount, removableEdgeVerts.data(), GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0); // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);

    glBindVertexArray(0);
}

void NavMeshDebugger::RenderRemovableEdges(Shader* shader)
{
    if (m_RemovableEdgeVAO && m_RemovableEdgeCount > 0)
    {
        shader->setVec4("ourColor", glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));
        glLineWidth(3.f); 
        glBindVertexArray(m_RemovableEdgeVAO);
        glDrawArrays(GL_LINES, 0, m_RemovableEdgeCount);
        glBindVertexArray(0);
        glLineWidth(1.f); 
    }
}

void NavMeshDebugger::SetCannotRemoveEdges(const std::vector<glm::vec3>& cannotRemoveEdgeVerts)
{
    m_CannotRemoveEdgeCount = static_cast<GLsizei>(cannotRemoveEdgeVerts.size());
    if (m_CannotRemoveEdgeCount == 0)
        return;

    if (m_CannotRemoveEdgeVAO == 0)
    {
        glGenVertexArrays(1, &m_CannotRemoveEdgeVAO);
        glGenBuffers(1, &m_CannotRemoveEdgeVBO);
    }

    glBindVertexArray(m_CannotRemoveEdgeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_CannotRemoveEdgeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * m_CannotRemoveEdgeCount, cannotRemoveEdgeVerts.data(), GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0); // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);

    glBindVertexArray(0);
}

void NavMeshDebugger::RenderCannotRemoveEdges(Shader* shader)
{
    if (m_CannotRemoveEdgeVAO && m_CannotRemoveEdgeCount > 0)
    {
        shader->setVec4("ourColor", glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
        glLineWidth(3.f); 
        glBindVertexArray(m_CannotRemoveEdgeVAO);
        glDrawArrays(GL_LINES, 0, m_CannotRemoveEdgeCount);
        glBindVertexArray(0);
        glLineWidth(1.f);
    }
}



void NavMeshDebugger::SetStartEndMarkers(const glm::vec3& start, const glm::vec3& end)
{
    if (m_StartEndMarkerVAO == 0) 
    {
        glGenVertexArrays(1, &m_StartEndMarkerVAO);
        glGenBuffers(1, &m_StartEndMarkerVBO);
    }

    float s = 0.5f;
    float yOffset = 0.1f;

    std::vector<glm::vec3> markers =
    {
        glm::vec3(start.x - s, start.y + yOffset, start.z), glm::vec3(start.x + s, start.y + yOffset, start.z),
        glm::vec3(start.x, start.y + yOffset - s, start.z), glm::vec3(start.x, start.y + yOffset + s, start.z),
        glm::vec3(start.x, start.y + yOffset, start.z - s), glm::vec3(start.x, start.y + yOffset, start.z + s),

        glm::vec3(end.x - s, end.y + yOffset, end.z), glm::vec3(end.x + s, end.y + yOffset, end.z),
        glm::vec3(end.x, end.y + yOffset - s, end.z), glm::vec3(end.x, end.y + yOffset + s, end.z),
        glm::vec3(end.x, end.y + yOffset, end.z - s), glm::vec3(end.x, end.y + yOffset, end.z + s)
    };

    m_StartEndMarkerCount = static_cast<GLsizei>(markers.size());

    glBindVertexArray(m_StartEndMarkerVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_StartEndMarkerVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * m_StartEndMarkerCount, markers.data(), GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glBindVertexArray(0);
}

void NavMeshDebugger::RenderStartEndMarkers(Shader* shader)
{
    if (m_StartEndMarkerVAO && m_StartEndMarkerCount > 0)
    {
        glLineWidth(3.f);
        glBindVertexArray(m_StartEndMarkerVAO);
        
        shader->setVec4("ourColor", glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
        glDrawArrays(GL_LINES, 0, 6);
        
        shader->setVec4("ourColor", glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));
        glDrawArrays(GL_LINES, 6, 6);

        glBindVertexArray(0);
        glLineWidth(1.f);
    }
}

void NavMeshDebugger::SetPath(const std::vector<int>& nodeIDs, NavMesh& navMesh)
{
    if (nodeIDs.size() < 2)
    {
        m_PathPointCount = 0;
        return;
    }
    
    std::vector<glm::vec3> pathPoints;
    for (int nodeID : nodeIDs)
        pathPoints.push_back(navMesh.GetNodeCenter(nodeID));
    
    m_PathPointCount = static_cast<GLsizei>(pathPoints.size());

    if (m_PathPointVAO == 0)
    {
        glGenVertexArrays(1, &m_PathPointVAO);
        glGenBuffers(1, &m_PathPointVBO);
    }

    glBindVertexArray(m_PathPointVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_PathPointVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * m_PathPointCount, pathPoints.data(), GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0); // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);

    glBindVertexArray(0);
}

void NavMeshDebugger::RenderPath(Shader* shader)
{
    if (m_PathPointVAO && m_PathPointCount > 0)
    {
        shader->setVec4("ourColor", glm::vec4(1.0f, 1.0f, 0.0f, 1.0f));
        glLineWidth(4.f); 
        
        glBindVertexArray(m_PathPointVAO);
        glDrawArrays(GL_LINE_STRIP, 0, m_PathPointCount);
        
        glPointSize(10.f);
        glDrawArrays(GL_POINTS, 0, m_PathPointCount);
        
        glBindVertexArray(0);
        glLineWidth(1.f);
        glPointSize(1.f);
    }
}
