/*
 * Copyright (c) 2016, Fabian Langguth
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef SMVS_DELAUNAY_2D_HEADER
#define SMVS_DELAUNAY_2D_HEADER

#include <memory>
#include <set>

#include "mve/math/vector.h"
#include "mve/core/mesh.h"

#include "defines.h"
#include "quad_edge.h"

namespace smvs {

/*
 * Incremental 2D Delaunay Triangulation based on:
 *   Leonidas J. Guibas and Jorge Stolfi,
 *   Primitives for the Manipulation of General Subdivisions and the
 *   Computation of Voronoi Diagrams, ACM ToG 4(2):74–123, April 1985
 */

class Delaunay2D
{
public:
    /// Initialize triangulation to rectangular domain
    Delaunay2D (mve::math::Vec2d min, mve::math::Vec2d max, double z);
    Delaunay2D (mve::math::Vec3d p1, mve::math::Vec3d p2, mve::math::Vec3d p3, mve::math::Vec3d p4);

    void insert_point (mve::math::Vec3d const& p3d, std::size_t triangle = -1);

    mve::core::TriangleMesh::Ptr get_mesh (void) const;

    void fill_recently_changed (std::vector<std::size_t> * triangles) const;
    void fill_triangle_vertices (std::size_t triangle,
        double * vertices) const;

private:
    void initialize (mve::math::Vec3d p1, mve::math::Vec3d p2, mve::math::Vec3d p3,
        mve::math::Vec3d p4);

    void flip_edge (Edge::Ptr e);
    Edge::Ptr connect_edges (Edge::Ptr a, Edge::Ptr b);
    void delete_edge (Edge::Ptr e);
    Edge::Ptr locate (mve::math::Vec2d const& p, Edge::Ptr start_edge);
    mve::math::Vec2d edge_orig (Edge::Ptr e);
    mve::math::Vec2d edge_dest (Edge::Ptr e);

    void debug_print_edge (Edge::Ptr e);

private:
    struct Triangle
    {
        Triangle (Edge::Ptr start) : start(start) { }
        Edge::Ptr start;
        mve::math::Vec3ui get_vertices (void) const;
    };

    Edge::Ptr start;
    std::vector<mve::math::Vec3d> vertices;
    std::vector<Triangle> triangles;
    std::vector<std::unique_ptr<QuadEdge>> q_edges;
    std::set<std::size_t> recently_changed;
};

/* ------------------------ Implementation ------------------------ */

inline mve::math::Vec2d
Delaunay2D::edge_orig (Edge::Ptr e)
{
    return mve::math::Vec2d(*this->vertices[e->orig()]);
}

inline mve::math::Vec2d
Delaunay2D::edge_dest (Edge::Ptr e)
{
    return mve::math::Vec2d(*this->vertices[e->dest()]);
}

inline void
Delaunay2D::fill_recently_changed(std::vector<std::size_t> * triangles) const
{
    triangles->clear();
    for (auto const& t : this->recently_changed)
        triangles->push_back(t);
}

} // namespace smvs

#endif /* SMVS_DELAUNAY_2D_HEADER */
