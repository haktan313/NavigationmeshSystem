#pragma once
#include <glm/vec3.hpp>

#include "Core/Camera.h"
#include "Core/Scene.h"

struct NavMeshTriangle;

class NavMeshDebugger
{
public:
    NavMeshDebugger();
    ~NavMeshDebugger();

    void RenderDebugTool(Shader* shader, Camera& camera, const Scene& scene);
    void SetBorderVertices(const std::vector<glm::vec3>& borderVerts);

    void SetEarClipping(std::vector<glm::vec3> earTriangleVerts);
    void RenderEarClipping(Shader* shader);
    
    void SetTriangles(const std::vector<NavMeshTriangle>& triangles);
    void RenderTriangles(Shader* shader);
private:
    GLsizei m_BorderCount; 
    GLsizei m_TriangleCount;
    GLsizei m_EarClipCount;
    unsigned int m_DebugVAO, m_DebugVBO;
    unsigned int m_BorderVAO, m_BorderVBO;
    unsigned int m_TriangleVAO, m_TriangleVBO;

    void RenderBorder(Shader* shader);
};
