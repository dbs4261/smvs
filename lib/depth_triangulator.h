/*
 * Copyright (c) 2016, Fabian Langguth
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef SMVS_DEPTH_TRIANGULATOR_HEADER
#define SMVS_DEPTH_TRIANGULATOR_HEADER

#include <vector>
#include <map>

#include "mve/core/image.h"
#include "mve/core/camera.h"
#include "mve/core/mesh.h"

#include "defines.h"
#include "delaunay_2d.h"

namespace smvs {

class DepthTriangulator
{
public:
    DepthTriangulator (mve::core::FloatImage::ConstPtr depth_map,
        mve::core::CameraInfo const& camera,
        mve::core::ByteImage::ConstPtr color = nullptr);

    mve::core::TriangleMesh::Ptr full_triangulation (void);

    /**
     * Compute approximate triangulation of depth map using the greedy,
     * Delaunay based algorithm presented in:
     *
     * "Fast Triangular Approximation of Terrains and Height Fields"
     * Michael Garland and Paul S. Heckbert, 1997
     *
     * max_vertices: Maximal number of vertices in the final triangulation.
     *               This defaults to 2.5% of the number of pixels in the dm.
     *
     * max_error: Maximal error of the final triangulation compared to
     *            the ground truth depth map. This defaults to 0.05% of the
     *            full depth range [min, max].
     */
    mve::core::TriangleMesh::Ptr approximate_triangulation (int max_vertices = -1,
        double max_error = -1.0);

    static void pixels_for_triangle (mve::math::Vec3d const& a, mve::math::Vec3d const& b,
        mve::math::Vec3d const& c, std::vector<mve::math::Vec2i> * pixels);

private:
    struct Triangle
    {
        Triangle (std::size_t id, double * vertices);
        std::size_t id;
        int num_zero_depths;
        mve::math::Vec3d v1, v2, v3;
        mve::math::Vec3d candidate;
        std::multimap<double, std::size_t>::iterator heap_iterator;
    };

private:
    void scan_triangle (std::size_t id);

private:
    mve::core::FloatImage::ConstPtr depth_map;
    mve::core::CameraInfo const& camera;
    mve::core::ByteImage::ConstPtr color;

    std::vector<Triangle> triangles;
    std::multimap<double, std::size_t, std::greater<double>> triangle_heap;

};

/* ------------------------ Implementation ------------------------ */

inline
DepthTriangulator::DepthTriangulator (mve::core::FloatImage::ConstPtr depth_map,
    mve::core::CameraInfo const& camera, mve::core::ByteImage::ConstPtr color)
    : depth_map(depth_map), camera(camera), color(color)
{
}

inline
DepthTriangulator::Triangle::Triangle (std::size_t id, double * vertices)
    : id(id), v1(vertices), v2(vertices + 3), v3(vertices + 6)
{
}


} // namespace smvs

#endif /* SMVS_DEPTH_TRIANGULATOR_HEADER */
