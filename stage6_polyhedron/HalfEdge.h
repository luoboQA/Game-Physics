#ifndef HALF_EDGE_H
#define HALF_EDGE_H

#include "Vec3.h"
#include <vector>
#include <map>
#include <cfloat>

// 半边数据结构（用于凸多面体）
struct HalfEdgeMesh {
    // 半边结构
    struct Edge {
        int vert;      // 指向的顶点索引
        int face;      // 左侧的面索引
        int next;      // 同面的下一条边
        int prev;      // 同面的上一条边
        int twin;      // 孪生边
        int freeLink;  // 空闲链表
        bool active;
        
        Edge() : vert(-1), face(-1), next(-1), prev(-1), 
                 twin(-1), freeLink(-1), active(false) {}
    };
    
    struct Vert {
        Vec3 position;
        int edge;      // 一条出边
        int freeLink;
        bool active;
        
        Vert() : edge(-1), freeLink(-1), active(false) {}
    };
    
    struct Face {
        int edge;      // 一条组成面的边
        int freeLink;
        bool active;
        
        Face() : edge(-1), freeLink(-1), active(false) {}
    };
    
    std::vector<Vert> verts;
    std::vector<Edge> edges;
    std::vector<Face> faces;
    
    int freeVerts;
    int freeEdges;
    int freeFaces;
    
    unsigned int numVerts;
    unsigned int numEdges;
    unsigned int numFaces;
    
    // 快速查找边对
    typedef std::pair<int, int> VertPair;
    std::map<VertPair, int> edgeMap;
    
    HalfEdgeMesh();
    
    int AddVert(const Vec3& pos);
    void RemoveVert(int v);
    int AddFace(int v0, int v1, int v2);
    void RemoveFace(int f);
    void Clear();
    
    // 爬山法支撑函数（O(√n) 时间复杂度）
    Vec3 Support(const Vec3& dir) const;
    
    // 获取所有顶点
    const std::vector<Vert>& GetVerts() const { return verts; }
    
    unsigned int GetNumVerts() const { return numVerts; }
    unsigned int GetNumFaces() const { return numFaces; }
    
private:
    int FindVertEdge(int v) const;
    
    // 模板函数必须在头文件中实现
    template <typename T>
    int Allocate(int& freeList, std::vector<T>& container) {
        int index = -1;
        if (freeList < 0) {
            index = static_cast<int>(container.size());
            container.resize(index + 1);
        } else {
            index = freeList;
            T& feature = container[freeList];
            freeList = feature.freeLink;
            feature.freeLink = -1;
        }
        T& feature = container[index];
        feature.active = true;
        return index;
    }
    
    template <typename T>
    void Free(int index, int& freeList, std::vector<T>& container) {
        T& feature = container[index];
        feature.freeLink = freeList;
        feature.active = false;
        freeList = index;
    }
};

#endif