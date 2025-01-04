// Copyright (c) [2025] [Initial_Equationor]
// [WaterSurface] is licensed under Mulan PubL v2.
// You can use this software according to the terms and conditions of the Mulan PubL v2.
// You may obtain a copy of Mulan PubL v2 at:
// http://license.coscl.org.cn/MulanPubL-2.0
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PubL v2 for more details.
#include "water_surface.hpp"
#include <random>
#include <unordered_set>
#include <algorithm>
#include <stdexcept>

namespace WaterSurface {
std::vector<Vector2d> RandomPoints(Rect border, int nPoints) {
    std::vector<Vector2d> points(nPoints);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> disX(border.x, border.x + border.w);
    std::uniform_real_distribution<float> disY(border.y, border.y + border.h);
    for (auto& point : points) {
        point.x = disX(gen);
        point.y = disY(gen);
    }
    return points;
} // std::vector<Vector2d> RandomPoints
TriangleNet Triangulate(std::vector<Vector2d> const& points) {
    TriangleNet net;
    net.nVerts = points.size();
    net.verts.reserve(points.size());
    std::vector<double> coords(2*net.nVerts);
    for (int i = 0; i < net.nVerts; ++i) {
        coords[2*i] = points[i].x;
        coords[2*i+1] = points[i].y;
        net.verts.push_back({ 0 });
        net.verts[i].position = points[i];
//        if (i % 3 == 1) {
//            net.verts[i].isFixed = true;
//        }
    }
    delaunator::Delaunator d(coords);
    std::vector<std::unordered_set<int>> iNeighbours(net.nVerts);
    net.nTriangles = d.triangles.size();
    net.triangles.reserve(3*net.nTriangles);
    for (std::size_t i = 0; i < net.nTriangles; i += 3) {
        net.triangles.push_back(d.triangles[i]);
        net.triangles.push_back(d.triangles[i+1]);
        net.triangles.push_back(d.triangles[i+2]);
        iNeighbours[d.triangles[i]].insert(d.triangles[i + 1]);
        iNeighbours[d.triangles[i]].insert(d.triangles[i + 2]);
        iNeighbours[d.triangles[i + 1]].insert(d.triangles[i]);
        iNeighbours[d.triangles[i + 1]].insert(d.triangles[i + 2]);
        iNeighbours[d.triangles[i + 2]].insert(d.triangles[i]);
        iNeighbours[d.triangles[i + 2]].insert(d.triangles[i + 1]);
    }
    for (int i = 0; i < net.nVerts; ++i) {
        net.verts[i].nNeighbours = iNeighbours[i].size();
        std::vector<std::pair<int, double>> rads(net.verts[i].nNeighbours);
        int idx = 0;
        for (auto &iNeighbour: iNeighbours[i]) {
            rads[idx].first = iNeighbour;
            float dx = net.verts[iNeighbour].position.x - net.verts[i].position.x;
            float dy = net.verts[iNeighbour].position.y - net.verts[i].position.y;
            float norm = sqrt(dx * dx + dy * dy);
            rads[idx].second = acos(dx / norm);
            if (dy < 0) {
                rads[idx].second = 2 * M_PI - rads[idx].second;
            }
            idx++;
        }
        std::sort(rads.begin(), rads.end(), [](const auto &a, const auto &b) {
            return a.second < b.second;
        });
//        float originLen = 0;
        for (int j = 0; j < net.verts[i].nNeighbours; ++j) {
            net.verts[i].iNeighbours[j] = rads[j].first;
            float dx = net.verts[rads[j].first].position.x - net.verts[i].position.x;
            float dy = net.verts[rads[j].first].position.y - net.verts[i].position.y;
//            originLen += sqrt(dx*dx+dy*dy) / net.verts[i].nNeighbours;
            net.verts[i].originLen[j] = sqrt(dx*dx+dy*dy);
        }
//        for (int j = 0; j < net.verts[i].nNeighbours; ++j) {
//            net.verts[i].originLen[j] = originLen;
//        }
    }
    return net;
} // TriangleNet Triangulate
std::vector<Polygon> CalculateVoronoi(TriangleNet const& net, float margin) {
    std::vector<Polygon> voronoi;
    voronoi.reserve(net.nVerts);
    for (int i = 0; i < net.nVerts; ++i) {
        const Node& node0 = net.verts[i];
        voronoi.push_back({ 0 });
        voronoi[i].nVerts = node0.nNeighbours;
        for (int j = 0, k = node0.nNeighbours-1; j < node0.nNeighbours; k = j++) {
            const Node& node1 = net.verts[node0.iNeighbours[k]];
            const Node& node2 = net.verts[node0.iNeighbours[j]];
            Vector2d p0 = { (node0.position.x + node1.position.x)/2,
                            (node0.position.y + node1.position.y)/2 };
            Vector2d p1 = { (node0.position.x + node2.position.x)/2,
                            (node0.position.y + node2.position.y)/2 };
            Vector2d r0 = { p0.x - node0.position.x, p0.y - node0.position.y};
            Vector2d r1 = { p1.x - node0.position.x, p1.y - node0.position.y};
            float norm0 = sqrt(r0.x*r0.x+r0.y*r0.y);
            if (norm0/2>margin) {
                p0.x -= margin * r0.x / norm0;
                p0.y -= margin * r0.y / norm0;
            }
            float norm1 = sqrt(r1.x*r1.x+r1.y*r1.y);
            if (norm1/2>margin) {
                p1.x -= margin * r1.x / norm1;
                p1.y -= margin * r1.y / norm1;
            }
            Vector2d d0 = { -r0.y/norm0, r0.x/norm0 };
            Vector2d d1 = { r1.y/norm1, -r1.x/norm1 };
            float det = d0.x * d1.y - d0.y * d1.x;
            const float epsilon = 1e-5;
            if (-epsilon < det && det < epsilon) {
                voronoi[i].verts[j] = { (p0.x+p1.x)/2, (p0.y+p1.y)/2 };
            } else {
                float px = - ((p0.x * d0.y - p0.y * d0.x) * d1.x - (p1.x * d1.y - p1.y * d1.x) * d0.x ) / det;
                float py = - ((p0.x * d0.y - p0.y * d0.x) * d1.y - (p1.x * d1.y - p1.y * d1.x) * d0.y ) / det;
//                if (false) {
                if ((px - p0.x) * d0.x < 0 && (px - p1.x) * d1.x < 0) {
                    // TODO: 处理后方交汇
//                    voronoi[i].verts[j] = node0.position;
                    voronoi[i].verts[j] = { 2*node0.position.x-px, 2*node0.position.y-py };
                } else {
                    voronoi[i].verts[j] = { px, py };
                }
            }
        } // for j,k
    } // for i
    return voronoi;
} // std::vector<Polygon> CalculateVoronoi
std::vector<Vector2d> BezierCurve(Polygon& polygon, int nSegments) {
    std::vector<Vector2d> curve;
    curve.reserve(polygon.nVerts * nSegments);
    for (int i = 0, j = polygon.nVerts - 1; i < polygon.nVerts; j=i++) {
        int k = (i + 1) % polygon.nVerts;
        Vector2d p0 = { (polygon.verts[j].x + polygon.verts[i].x) / 2,
                        (polygon.verts[j].y + polygon.verts[i].y) / 2 };
        Vector2d c = polygon.verts[i];
        Vector2d p1 = { (polygon.verts[i].x + polygon.verts[k].x) / 2,
                        (polygon.verts[i].y + polygon.verts[k].y) / 2 };
        for (int l = 0; l < nSegments; ++l) {
            float step = static_cast<float>(l) / nSegments;
            Vector2d c0 = { p0.x + step * (c.x - p0.x),
                            p0.y + step * (c.y - p0.y) };
            Vector2d c1 = { c.x + step * (p1.x - c.x),
                            c.y + step * (p1.y - c.y) };
            curve.push_back({ c0.x + step * (c1.x - c0.x),
                              c0.y + step * (c1.y - c0.y) });
        }
    } // for i,j,k
    return curve;
} // std::vector<Vector2d> BezierCurve

void UpdatePhysics(TriangleNet &net, float tau, float k) {
//    const float kBase = 10.f;
    for (int i = 0; i < net.nVerts; ++i) {
        Node& node0 = net.verts[i];
        if (node0.isFixed) continue;
        for (int j = 0; j < node0.nNeighbours; ++j) {
            Node& node1 = net.verts[node0.iNeighbours[j]];
            float dx = node1.position.x - node0.position.x;
            float dy = node1.position.y - node0.position.y;
            float norm = sqrt(dx*dx+dy*dy);
            if (norm == 0) {
                dx = node1.velocity.x - node0.velocity.x;
                dy = node1.velocity.y - node0.velocity.y;
                norm = sqrt(dx*dx+dy*dy);
                if (norm == 0) {
                    // TODO: 这种情况实在不知道如何处理了
                    continue;
                }
            }
            dx /= norm; dy /= norm;
//            float kMult = kBase / (kBase + node0.originLen[j]); // 越短弹力系数越大，防止破坏三角网拓扑结构
//            node0.velocity.x += tau * k * kMult * (norm - node0.originLen[j]) * dx;
//            node0.velocity.y += tau * k * kMult * (norm - node0.originLen[j]) * dy;
            node0.velocity.x += tau * k * (norm - node0.originLen[j]) * dx;
            node0.velocity.y += tau * k * (norm - node0.originLen[j]) * dy;
        } // for j
    } // for i
    for (int i = 0; i < net.nVerts; ++i) {
        Node& node0 = net.verts[i];
        node0.position.x += tau * node0.velocity.x;
        node0.position.y += tau * node0.velocity.y;
    } // for i
} // void UpdatePhysics

void FirstMove(TriangleNet &net, float energy) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> rad(0, 2*M_PI);
    for (int i = 0; i < net.nVerts; ++i) {
        Node& node0 = net.verts[i];
        if (node0.isFixed) continue;
        double theta = rad(gen);
        float velocity = node0.originLen[0];
        for (int j = 1; j < node0.nNeighbours; ++j) {
            if (node0.originLen[j] < velocity) {
                velocity = node0.originLen[j];
            }
        }
        velocity *= energy;
//        velocity = log(velocity+1);
        node0.velocity.x = velocity * cos(theta);
        node0.velocity.y = velocity * sin(theta);
    } // for i
} // void FirstMove
} // namespace WaterSurface