#pragma once
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <vector>
#include "delaunator.hpp"
#define MAX_NEIGHBOURS 16

namespace WaterSurface {
typedef struct Rect {
    float x,y;
    float h,w;
} Rect;
typedef struct Vector2d {
    float x,y;
} Vector2d;
std::vector<Vector2d> RandomPoints(Rect border, int nPoints=255);
typedef struct Node {
    Vector2d position;
    int nNeighbours = 0;
    int iNeighbours[MAX_NEIGHBOURS];
    Vector2d velocity = { 0 };
    float originLen[MAX_NEIGHBOURS];
    bool isFixed = false;
} Node;
//typedef std::vector<Node> TriangleNet;
typedef struct TriangleNet {
    int nVerts = 0;
    std::vector<Node> verts;
    int nTriangles = 0;
    std::vector<int> triangles;
} TriangleNet;
TriangleNet Triangulate(std::vector<Vector2d> const& points);
typedef struct Polygon {
    int nVerts = 0;
    Vector2d verts[MAX_NEIGHBOURS];
} Polygon;
std::vector<Polygon> CalculateVoronoi(TriangleNet const& net, float margin = 2.5);
std::vector<Vector2d> BezierCurve(Polygon& polygon, int nSegments = 20);
void UpdatePhysics(TriangleNet &net, float tau, float k=1.0);
void FirstMove(TriangleNet &net, float energy=0.1);
} // namespace WaterSurface