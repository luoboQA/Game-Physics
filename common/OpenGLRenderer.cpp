#include "OpenGLRenderer.h"
#include <cmath>
#include <cstdio>

OpenGLRenderer* OpenGLRenderer::s_instance = nullptr;

// 全局物理更新函数（需要在外部定义）
void UpdatePhysics();

OpenGLRenderer::OpenGLRenderer()
    : m_width(800), m_height(600)
    , m_cameraPos(15, 10, 15)
    , m_cameraTarget(0, 0, 0)
    , m_running(false)
    , m_dt(1.0f/60.0f) {
    s_instance = this;
}

OpenGLRenderer::~OpenGLRenderer() {
    s_instance = nullptr;
}

void OpenGLRenderer::Init(int argc, char** argv, int width, int height) {
    m_width = width;
    m_height = height;
    
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(width, height);
    glutInitWindowPosition(100, 100);  // 确保窗口在可见位置
    glutCreateWindow("Game Physics Engine");
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    
    GLfloat lightPos[] = { 5.0f, 10.0f, 5.0f, 1.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    
    glClearColor(0.2f, 0.2f, 0.3f, 1.0f);
    
    glutDisplayFunc(DisplayCallback);
    glutReshapeFunc(ReshapeCallback);
    glutKeyboardFunc(KeyboardCallback);
    glutIdleFunc(IdleCallback);
    
    printf("[OpenGL] Initialized successfully\n");
}

void OpenGLRenderer::AddObject(RigidBody* body, Collider* collider, float r, float g, float b) {
    RenderObject obj;
    obj.body = body;
    obj.collider = collider;
    obj.r = r;
    obj.g = g;
    obj.b = b;
    m_objects.push_back(obj);
}

void OpenGLRenderer::ClearObjects() {
    m_objects.clear();
}

void OpenGLRenderer::Run(float dt) {
    m_dt = dt;
    m_running = true;
    printf("[OpenGL] Entering main loop\n");
    glutMainLoop();
}

void OpenGLRenderer::Stop() {
    m_running = false;
    exit(0);
}

void OpenGLRenderer::DisplayCallback() {
    if (s_instance) {
        s_instance->Render();
    }
}

void OpenGLRenderer::ReshapeCallback(int w, int h) {
    if (s_instance) {
        s_instance->m_width = w;
        s_instance->m_height = h;
        glViewport(0, 0, w, h);
    }
}

void OpenGLRenderer::KeyboardCallback(unsigned char key, int x, int y) {
    if (!s_instance) return;
    
    switch(key) {
        case 'w': s_instance->m_cameraPos.z -= 1.0f; break;
        case 's': s_instance->m_cameraPos.z += 1.0f; break;
        case 'a': s_instance->m_cameraPos.x -= 1.0f; break;
        case 'd': s_instance->m_cameraPos.x += 1.0f; break;
        case 'r': s_instance->m_cameraPos.y += 1.0f; break;
        case 'f': s_instance->m_cameraPos.y -= 1.0f; break;
        case 'c': s_instance->m_cameraPos = Vec3(15, 10, 15); break;
        case 27:  // ESC
        case 'q': s_instance->Stop(); break;
    }
    glutPostRedisplay();
}

void OpenGLRenderer::IdleCallback() {
    if (s_instance && s_instance->m_running) {
        // 更新物理
        UpdatePhysics();
        // 请求重绘
        glutPostRedisplay();
    }
}

void OpenGLRenderer::Render() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (double)m_width / m_height, 0.1, 100.0);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(m_cameraPos.x, m_cameraPos.y, m_cameraPos.z,
              m_cameraTarget.x, m_cameraTarget.y, m_cameraTarget.z,
              0, 1, 0);
    
    DrawGrid();
    DrawGround();
    
    for (const auto& obj : m_objects) {
        if (!obj.body || !obj.collider) continue;
        
        Vec3 pos = obj.body->m_position;
        Mat3 rot = obj.body->m_orientation;
        
        if (obj.collider->type == COLLIDER_BOX) {
            BoxCollider* box = static_cast<BoxCollider*>(obj.collider);
            DrawBox(pos, rot, box->halfExtents, obj.r, obj.g, obj.b);
        } else if (obj.collider->type == COLLIDER_SPHERE) {
            SphereCollider* sphere = static_cast<SphereCollider*>(obj.collider);
            DrawSphere(pos, sphere->radius, obj.r, obj.g, obj.b);
        }
    }
    
    glutSwapBuffers();
}

void OpenGLRenderer::DrawGrid() {
    glDisable(GL_LIGHTING);
    glColor3f(0.5f, 0.5f, 0.5f);
    glBegin(GL_LINES);
    
    for (int i = -15; i <= 15; i++) {
        glVertex3f(i, -5.0f, -15.0f);
        glVertex3f(i, -5.0f, 15.0f);
        glVertex3f(-15.0f, -5.0f, i);
        glVertex3f(15.0f, -5.0f, i);
    }
    
    glEnd();
    glEnable(GL_LIGHTING);
}

void OpenGLRenderer::DrawGround() {
    glDisable(GL_LIGHTING);
    glColor3f(0.3f, 0.5f, 0.3f);
    glBegin(GL_QUADS);
    glVertex3f(-15, -4.5f, -15);
    glVertex3f( 15, -4.5f, -15);
    glVertex3f( 15, -4.5f,  15);
    glVertex3f(-15, -4.5f,  15);
    glEnd();
    glEnable(GL_LIGHTING);
}

void OpenGLRenderer::DrawBox(const Vec3& pos, const Mat3& rot, const Vec3& halfExtents, 
                              float r, float g, float b) {
    glPushMatrix();
    glTranslatef(pos.x, pos.y, pos.z);
    
    GLfloat matrix[16] = {
        rot.m[0][0], rot.m[1][0], rot.m[2][0], 0,
        rot.m[0][1], rot.m[1][1], rot.m[2][1], 0,
        rot.m[0][2], rot.m[1][2], rot.m[2][2], 0,
        0, 0, 0, 1
    };
    glMultMatrixf(matrix);
    
    glColor3f(r, g, b);
    glScalef(halfExtents.x * 2, halfExtents.y * 2, halfExtents.z * 2);
    
    glutSolidCube(1.0);
    
    glPopMatrix();
}

void OpenGLRenderer::DrawSphere(const Vec3& pos, float radius, float r, float g, float b) {
    glPushMatrix();
    glTranslatef(pos.x, pos.y, pos.z);
    glColor3f(r, g, b);
    glutSolidSphere(radius, 32, 16);
    glPopMatrix();
}