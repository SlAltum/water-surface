// Copyright (c) [2025] [Initial_Equationor]
// [WaterSurface] is licensed under Mulan PubL v2.
// You can use this software according to the terms and conditions of the Mulan PubL v2.
// You may obtain a copy of Mulan PubL v2 at:
// http://license.coscl.org.cn/MulanPubL-2.0
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PubL v2 for more details.
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <gtest/gtest.h>
#include <iostream>
#ifndef STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#endif
//#include "stb_image.h"
#include "water_surface.hpp"

TEST(WaterSurface, RandomPoints) {
    auto points = WaterSurface::RandomPoints({ 100, 100, 256, 256 });
    ASSERT_EQ(points.size(), 255);
    for (int i = 0; i < 255; ++i) {
        ASSERT_GE(points[i].x, 100); ASSERT_LE(points[i].x, 356);
        ASSERT_GE(points[i].y, 100); ASSERT_LE(points[i].y, 356);
    }
} // TEST(WaterSurface, RandomPoints)

TEST(WaterSurface, Triangulate) {
    auto points = WaterSurface::RandomPoints({ 100, 100, 256, 256 });
    ASSERT_EQ(points.size(), 255);
    for (int i = 0; i < 255; ++i) {
        ASSERT_GE(points[i].x, 100); ASSERT_LE(points[i].x, 356);
        ASSERT_GE(points[i].y, 100); ASSERT_LE(points[i].y, 356);
    }
    WaterSurface::TriangleNet net = WaterSurface::Triangulate(points);
    ASSERT_EQ(net.nVerts, 255);
    for (int i = 0; i < 255; ++i) {
        ASSERT_LE(net.verts[i].nNeighbours, MAX_NEIGHBOURS) << "i = " << i;
        ASSERT_GE(net.verts[i].position.x, 100); ASSERT_LE(net.verts[i].position.x, 356) << "i = " << i;
        ASSERT_GE(net.verts[i].position.y, 100); ASSERT_LE(net.verts[i].position.y, 356) << "i = " << i;
        ASSERT_EQ(net.verts[i].position.x, points[i].x) << "i = " << i;
        ASSERT_EQ(net.verts[i].position.y, points[i].y) << "i = " << i;
    }
    for (auto& node : net.verts) {
        ASSERT_GE(node.nNeighbours, 2);
        std::vector<double> rads(node.nNeighbours);
        for (int i = 0; i < node.nNeighbours; ++i) {
            double dx = net.verts[node.iNeighbours[i]].position.x - node.position.x;
            double dy = net.verts[node.iNeighbours[i]].position.y - node.position.y;
            double norm = sqrt(dx*dx+dy*dy);
            rads[i] = acos(dx/norm);
            if (dy<0) {
                rads[i] = 2*M_PI-rads[i];
            }
        }
        for (int i = 0; i < node.nNeighbours-1; ++i) {
            ASSERT_LE(rads[i], rads[i+1])
                << "Angle order wrong at node " << i
                << "\nrads[" << i << "] = " << rads[i]
                << "\nrads[" << i+1 << "] = " << rads[i+1]
                << "\nposition = (" << node.position.x << "," << node.position.y << ")"
                << "\niNeighbour1 = " << node.iNeighbours[i]
                << "\nneighbourPosition1 = (" << net.verts[node.iNeighbours[i]].position.x << "," << net.verts[node.iNeighbours[i]].position.y << ")"
                << "\niNeighbour2 = " << node.iNeighbours[i+1]
                << "\nneighbourPosition2 = (" << net.verts[node.iNeighbours[i+1]].position.x << "," << net.verts[node.iNeighbours[i+1]].position.y << ")"
                << "\nnNeighbours = " << node.nNeighbours;
        }
    }
} // TEST(WaterSurface, Triangulate)

TEST(WaterSurface, CalculateVoronoi) {
    auto points = WaterSurface::RandomPoints({ 100, 100, 256, 256 });
    ASSERT_EQ(points.size(), 255);
    for (int i = 0; i < 255; ++i) {
        ASSERT_GE(points[i].x, 100); ASSERT_LE(points[i].x, 356);
        ASSERT_GE(points[i].y, 100); ASSERT_LE(points[i].y, 356);
    }
    WaterSurface::TriangleNet net = WaterSurface::Triangulate(points);
    ASSERT_EQ(net.nVerts, 255);
    for (int i = 0; i < 255; ++i) {
        ASSERT_LE(net.verts[i].nNeighbours, MAX_NEIGHBOURS) << "i = " << i;
        ASSERT_GE(net.verts[i].position.x, 100); ASSERT_LE(net.verts[i].position.x, 356) << "i = " << i;
        ASSERT_GE(net.verts[i].position.y, 100); ASSERT_LE(net.verts[i].position.y, 356) << "i = " << i;
        ASSERT_EQ(net.verts[i].position.x, points[i].x) << "i = " << i;
        ASSERT_EQ(net.verts[i].position.y, points[i].y) << "i = " << i;
    }
    for (auto& node : net.verts) {
        ASSERT_GE(node.nNeighbours, 2);
        std::vector<double> rads(node.nNeighbours);
        for (int i = 0; i < node.nNeighbours; ++i) {
            double dx = net.verts[node.iNeighbours[i]].position.x - node.position.x;
            double dy = net.verts[node.iNeighbours[i]].position.y - node.position.y;
            double norm = sqrt(dx*dx+dy*dy);
            rads[i] = acos(dx/norm);
            if (dy<0) {
                rads[i] = 2*M_PI-rads[i];
            }
        }
        for (int i = 0; i < node.nNeighbours-1; ++i) {
            ASSERT_LE(rads[i], rads[i+1])
                                        << "Angle order wrong at node " << i
                                        << "\nrads[" << i << "] = " << rads[i]
                                        << "\nrads[" << i+1 << "] = " << rads[i+1]
                                        << "\nposition = (" << node.position.x << "," << node.position.y << ")"
                                        << "\niNeighbour1 = " << node.iNeighbours[i]
                                        << "\nneighbourPosition1 = (" << net.verts[node.iNeighbours[i]].position.x << "," << net.verts[node.iNeighbours[i]].position.y << ")"
                                        << "\niNeighbour2 = " << node.iNeighbours[i+1]
                                        << "\nneighbourPosition2 = (" << net.verts[node.iNeighbours[i+1]].position.x << "," << net.verts[node.iNeighbours[i+1]].position.y << ")"
                                        << "\nnNeighbours = " << node.nNeighbours;
        }
    }
    auto voronoi = WaterSurface::CalculateVoronoi(net);
    ASSERT_EQ(net.nVerts, voronoi.size());
    for (int i = 0; i < net.nVerts; ++i) {
        ASSERT_EQ(net.verts[i].nNeighbours, voronoi[i].nVerts);
    }
} // TEST(WaterSurface, Triangulate)

TEST(WaterSurface, BezierCurve) {
    WaterSurface::Polygon polygon;
    polygon.nVerts = 4;
    polygon.verts[0] = { 0, 0 };
    polygon.verts[1] = { 0, 1 };
    polygon.verts[2] = { -1, 1 };
    polygon.verts[3] = { -1, 0 };
    auto curve = BezierCurve(polygon);
    ASSERT_EQ(curve.size(), 80);
    ASSERT_EQ(curve[0].x, -0.5); ASSERT_EQ(curve[0].y, 0);
    ASSERT_EQ(curve[20].x, 0); ASSERT_EQ(curve[20].y, 0.5);
    ASSERT_EQ(curve[40].x, -0.5); ASSERT_EQ(curve[40].y, 1);
    ASSERT_EQ(curve[60].x, -1); ASSERT_EQ(curve[60].y, 0.5);
} // TEST(WaterSurface, BezierCurve)

TEST(WaterSurface, UpdatePhysics) {
    WaterSurface::TriangleNet net = { 0 };
    net.nVerts = 2;
    net.verts.reserve(2);
    WaterSurface::Node p0 = { 0 };
    p0.position = { 0, 0 };
    p0.nNeighbours = 1;
    p0.iNeighbours[0] = 1;
    p0.originLen[0] = 2;
    p0.velocity = { 1, 0 };
    p0.isFixed = false;
    net.verts.push_back(p0);
    WaterSurface::Node p1 = { 0 };
    p1.position = { 2, 0 };
    p1.nNeighbours = 0;
    p1.iNeighbours[0] = 0;
    p1.originLen[0] = 2;
    p1.velocity = { 0, 0 };
    p1.isFixed = true;
    net.verts.push_back(p1);
    WaterSurface::UpdatePhysics(net, 1, 1);
    ASSERT_EQ(net.verts[0].position.x, 1);
    WaterSurface::UpdatePhysics(net, 1, 1);
    ASSERT_EQ(net.verts[0].velocity.x, 0);
    ASSERT_EQ(net.verts[0].position.x, 1);
} // TEST(WaterSurface, UpdatePhysics)

//int main() {
//    testing::InitGoogleTest();
//    return RUN_ALL_TESTS();
//}
