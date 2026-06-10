#include "HalfEdge.h"
#include <cstdio>
#include <algorithm>

HalfEdgeMesh::HalfEdgeMesh() 
    : freeVerts(-1), freeEdges(-1), freeFaces(-1)
    , numVerts(0), numEdges(0), numFaces(0) {
}

int HalfEdgeMesh::AddVert(const Vec3& position) {
    const int v = Allocate(freeVerts, verts);
    Vert& vert = verts[v];
    vert.position = position;
    vert.edge = -1;
    ++numVerts;
    return v;
}

void HalfEdgeMesh::RemoveVert(int v) {
    if (v < 0 || v >= (int)verts.size()) return;
    
    Vert& vert = verts[v];
    if (!vert.active) return;
    
    // 删除与此顶点连接的所有面
    const int startEdge = vert.edge;
    if (startEdge >= 0) {
        int e = startEdge;
        do {
            if (e < 0 || e >= (int)edges.size()) break;
            Edge& edge = edges[e];
            if (edge.face >= 0) {
                RemoveFace(edge.face);
            }
            if (edge.twin < 0 || edge.twin >= (int)edges.size()) break;
            e = edges[edge.prev].twin;
        } while (e != startEdge);
    }
    
    // 释放顶点
    if (vert.edge >= 0)
        vert.edge = -1;
    
    Free<Vert>(v, freeVerts, verts);
    --numVerts;
}

int HalfEdgeMesh::AddFace(int v0, int v1, int v2) {
    const int faceVerts[3] = { v0, v1, v2 };
    
    // 分配面
    const int f = Allocate(freeFaces, faces);
    Face& face = faces[f];
    
    // 创建边
    int faceEdgeIndices[3] = {-1, -1, -1};
    for (unsigned int i = 2, j = 0; j < 3; i = j++) {
        const unsigned int v0_idx = faceVerts[i];
        const unsigned int v1_idx = faceVerts[j];
        
        if (v0_idx >= verts.size() || v1_idx >= verts.size()) {
            return -1;
        }
        
        Vert& vert0 = verts[v0_idx];
        Vert& vert1 = verts[v1_idx];
        
        // 检查边对是否存在
        const auto edgeIter = edgeMap.find(VertPair(v0_idx, v1_idx));
        const bool edgePairExists = (edgeIter != edgeMap.end());
        int e01 = -1;
        int e10 = -1;
        
        if (edgePairExists) {
            e01 = edgeIter->second;
            if (e01 >= 0 && e01 < (int)edges.size()) {
                e10 = edges[e01].twin;
            }
        } else {
            // 分配新的边对
            e01 = Allocate(freeEdges, edges);
            e10 = Allocate(freeEdges, edges);
            
            if (e01 >= 0 && e10 >= 0) {
                edges[e01].twin = e10;
                edges[e10].twin = e01;
                
                edgeMap[VertPair(v0_idx, v1_idx)] = e01;
                edgeMap[VertPair(v1_idx, v0_idx)] = e10;
                
                numEdges += 2;
            }
        }
        
        if (e01 < 0 || e10 < 0) continue;
        
        Edge& edge01 = edges[e01];
        Edge& edge10 = edges[e10];
        
        if (edge01.vert < 0)
            edge01.vert = v1_idx;
        if (edge10.vert < 0)
            edge10.vert = v0_idx;
        
        if (edge01.face < 0)
            edge01.face = f;
        
        if (vert0.edge < 0)
            vert0.edge = e01;
        
        if (face.edge < 0)
            face.edge = e01;
        
        faceEdgeIndices[i] = e01;
    }
    
    // 链接面的边
    for (unsigned int i = 2, j = 0; j < 3; i = j++) {
        const int eI = faceEdgeIndices[i];
        const int eJ = faceEdgeIndices[j];
        if (eI >= 0 && eJ >= 0) {
            edges[eI].next = eJ;
            edges[eJ].prev = eI;
        }
    }
    
    ++numFaces;
    return f;
}

void HalfEdgeMesh::RemoveFace(int f) {
    if (f < 0 || f >= (int)faces.size()) return;
    if (!faces[f].active) return;
    
    const int startEdge = faces[f].edge;
    if (startEdge < 0) {
        Free<Face>(f, freeFaces, faces);
        --numFaces;
        return;
    }
    
    int faceVertIndices[3] = {-1, -1, -1};
    int faceEdgeIndices[3] = {-1, -1, -1};
    int e = startEdge;
    int i = 0;
    
    do {
        if (e < 0 || e >= (int)edges.size()) break;
        
        Edge& edge = edges[e];
        const int t = edge.twin;
        
        if (t >= 0 && t < (int)edges.size()) {
            Edge& twin = edges[t];
            
            faceVertIndices[i] = edge.vert;
            faceEdgeIndices[i] = e;
            ++i;
            
            // 如果孪生面不存在，释放两条边
            if (twin.face < 0) {
                VertPair p1(edge.vert, edges[edge.prev].vert);
                VertPair p2(edges[edge.prev].vert, edge.vert);
                edgeMap.erase(p1);
                edgeMap.erase(p2);
                
                edge.twin = -1;
                twin.twin = -1;
                
                Free<Edge>(e, freeEdges, edges);
                Free<Edge>(t, freeEdges, edges);
                
                numEdges -= 2;
            }
        }
        
        e = (e >= 0 && e < (int)edges.size()) ? edges[e].next : -1;
    } while (e != startEdge && e >= 0);
    
    // 解除边与所有特征的链接
    for (int j = 0; j < 3; ++j) {
        const int edgeIdx = faceEdgeIndices[j];
        if (edgeIdx >= 0 && edgeIdx < (int)edges.size()) {
            Edge& edge = edges[edgeIdx];
            edge.next = -1;
            edge.prev = -1;
            edge.vert = -1;
            edge.face = -1;
        }
    }
    
    // 更新顶点的边索引
    for (int j = 0; j < 3; ++j) {
        const int vertIdx = faceVertIndices[j];
        if (vertIdx >= 0 && vertIdx < (int)verts.size()) {
            Vert& vert = verts[vertIdx];
            vert.edge = FindVertEdge(vertIdx);
        }
    }
    
    // 解除面与边的链接
    Face& face = faces[f];
    face.edge = -1;
    
    // 释放面
    Free<Face>(f, freeFaces, faces);
    --numFaces;
}

int HalfEdgeMesh::FindVertEdge(int v) const {
    for (const auto& pair : edgeMap) {
        if (v == pair.first.first) {
            return pair.second;
        }
    }
    return -1;
}

void HalfEdgeMesh::Clear() {
    verts.clear();
    edges.clear();
    faces.clear();
    edgeMap.clear();
    
    numVerts = 0;
    numEdges = 0;
    numFaces = 0;
    
    freeVerts = -1;
    freeEdges = -1;
    freeFaces = -1;
}

// 爬山法支撑函数
Vec3 HalfEdgeMesh::Support(const Vec3& dir) const {
    if (verts.empty()) return Vec3(0,0,0);
    
    // 从第一个激活的顶点开始
    int bestVertIndex = -1;
    for (size_t i = 0; i < verts.size(); ++i) {
        if (verts[i].active) {
            bestVertIndex = i;
            break;
        }
    }
    
    if (bestVertIndex < 0) return Vec3(0,0,0);
    
    const Vert* bestVert = &verts[bestVertIndex];
    float bestDot = dir.Dot(bestVert->position);
    
    // 爬山法优化
    bool end = false;
    int maxIterations = 100;
    int iter = 0;
    
    while (!end && iter++ < maxIterations) {
        int e = bestVert->edge;
        if (e < 0 || e >= (int)edges.size()) break;
        
        const int startEdge = e;
        const float oldBestDot = bestDot;
        
        do {
            const Edge& edge = edges[e];
            if (edge.vert < 0 || edge.vert >= (int)verts.size()) break;
            
            const Vert& vert = verts[edge.vert];
            if (vert.active) {
                const float dot = dir.Dot(vert.position);
                
                if (dot > bestDot + 1e-6f) {
                    bestVertIndex = edge.vert;
                    bestVert = &vert;
                    bestDot = dot;
                }
            }
            
            // 获取下一条出边
            if (edge.twin >= 0 && edge.twin < (int)edges.size() && 
                edges[edge.twin].next >= 0) {
                e = edges[edge.twin].next;
            } else {
                break;
            }
        } while (e != startEdge && e >= 0 && e < (int)edges.size());
        
        end = (bestDot <= oldBestDot + 1e-6f);
    }
    
    return bestVert->position;
}