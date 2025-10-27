#include "NavMeshDebugger.h"
#include "NavMesh.h"


NavMeshDebugger::NavMeshDebugger()
    : m_DebugVAO(0), m_DebugVBO(0), m_BorderVAO(0), m_BorderVBO(0), m_BorderCount(0), m_TriangleVAO(0), m_TriangleVBO(0), m_TriangleCount(0)
{

}

NavMeshDebugger::~NavMeshDebugger()
{
    glDeleteVertexArrays(1, &m_DebugVAO);
    glDeleteBuffers(1, &m_DebugVBO);
    if (m_BorderVAO)
        glDeleteVertexArrays(1, &m_BorderVAO);
    if (m_BorderVBO)
        glDeleteBuffers(1, &m_BorderVBO);
    if (m_TriangleVAO)
        glDeleteVertexArrays(1, &m_TriangleVAO);
    if (m_TriangleVBO)
        glDeleteBuffers(1, &m_TriangleVBO);
}

void NavMeshDebugger::RenderDebugTool(Shader* shader, Camera& camera, const Scene& scene)
{
    shader->use();
    glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), 1280.0f/720.0f, 0.1f, 100.0f);
    glm::mat4 view = camera.GetViewMatrix();
    shader->setMat4("projection", projection);
    shader->setMat4("view", view);
    shader->setMat4("model", glm::mat4(1.0f));
    
    RenderBorder(shader);
    RenderEarClipping(shader);
    RenderTriangles(shader);
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

void NavMeshDebugger::SetEarClipping(std::vector<glm::vec3> clippedEars)
{
    glGenVertexArrays(1, &m_DebugVAO);
    glGenBuffers(1, &m_DebugVBO);
    
    glBindVertexArray(m_DebugVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_DebugVBO);
    m_EarClipCount = static_cast<GLsizei>(clippedEars.size());
    
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * clippedEars.size(), clippedEars.data(), GL_DYNAMIC_DRAW);
    
    glEnableVertexAttribArray(0);//position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    
    glBindVertexArray(0);
}

void NavMeshDebugger::RenderEarClipping(Shader* shader)
{
    if (m_DebugVAO)
    {
        shader->setVec4("ourColor", glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
        glLineWidth(2.f);
        glBindVertexArray(m_DebugVAO);
        glDrawArrays(GL_LINES, 0, m_EarClipCount);
        glBindVertexArray(0);
    }
}

void NavMeshDebugger::SetTriangles(const std::vector<NavMeshTriangle>& triangles)
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

void NavMeshDebugger::RenderTriangles(Shader* shader)
{
    if (m_TriangleVAO)
    {
        shader->setVec4("ourColor", glm::vec4(0.0f, 1.0f, 0.0f, 0.1f));
        glBindVertexArray(m_TriangleVAO);
        glDrawArrays(GL_TRIANGLES, 0, m_TriangleCount);
        glBindVertexArray(0);
    }
}