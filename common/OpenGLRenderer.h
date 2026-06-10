#ifndef OPENGL_RENDERER_H
#define OPENGL_RENDERER_H

#include "Vec3.h"
#include "RigidBody.h"
#include "Collider.h"
#include <vector>

#ifdef _WIN32
    #include <GL/glut.h>
#else
    #ifdef __APPLE__
        #include <GLUT/glut.h>
    #else
        #include <GL/glut.h>
        #include <GL/freeglut.h>
    #endif
#endif

struct RenderObject {
    RigidBody* body;
    Collider* collider;
    float r, g, b;  // 颜色
};

// 前向声明物理更新函数（在 main_visible.cpp 中定义）
void UpdatePhysics();

class OpenGLRenderer {
private:
    static OpenGLRenderer* s_instance;
    
    int m_width;
    int m_height;
    std::vector<RenderObject> m_objects;
    Vec3 m_cameraPos;
    Vec3 m_cameraTarget;
    bool m_running;
    float m_dt;
    
    static void DisplayCallback();
    static void ReshapeCallback(int w, int h);
    static void KeyboardCallback(unsigned char key, int x, int y);
    static void IdleCallback();
    
    void Render();
    void DrawGrid();
    void DrawGround();
    void DrawBox(const Vec3& pos, const Mat3& rot, const Vec3& halfExtents, float r, float g, float b);
    void DrawSphere(const Vec3& pos, float radius, float r, float g, float b);
    
public:
    OpenGLRenderer();
    ~OpenGLRenderer();
    
    void AddObject(RigidBody* body, Collider* collider, float r, float g, float b);
    void ClearObjects();
    
    void Init(int argc, char** argv, int width = 800, int height = 600);
    void Run(float dt);
    void Stop();
    
    static OpenGLRenderer* GetInstance() { return s_instance; }
};

#endif