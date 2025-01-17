/*
 * Copyright (c) 2016, Fabian Langguth
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include "mve/core/depthmap.h"
#include "mve/core/image_tools.h"
#include "mve/core/mesh_io.h"
#include "mve/core/mesh_tools.h"

#include "depth_triangulator.h"

namespace smvs {

mve::core::TriangleMesh::Ptr
DepthTriangulator::full_triangulation (void)
{
    mve::core::TriangleMesh::Ptr mesh = mve::core::geom::depthmap_triangulate(
        this->depth_map, this->color, this->camera);
    return mesh;
}

mve::core::TriangleMesh::Ptr
DepthTriangulator::approximate_triangulation (int max_vertices,
    double max_error)
{
    if (max_vertices < 0)
        max_vertices = this->depth_map->get_pixel_amount() / 40;

    float dm_min, dm_max;
    mve::core::image::find_min_max_value(this->depth_map, &dm_min, &dm_max);
    float dm_avg = 0;
    float counter = 0;
    for (float const* ptr = this->depth_map->begin();
         ptr != this->depth_map->end(); ++ptr)
        if (*ptr > 0)
        {
            dm_avg += *ptr;
            counter += 1;
        }
    dm_avg /= counter;
    if (max_error < 0.0)
        max_error = (dm_max - dm_avg) * 1e-3;

    int const max_x = this->depth_map->width() - 1;
    int const max_y = this->depth_map->height() - 1;

    mve::math::Vec3d bot_left (-1, -1, dm_max);
    if (this->depth_map->at(0, 0, 0) > 0)
        bot_left[2] = this->depth_map->at(0, 0, 0);
    mve::math::Vec3d bot_right (max_x + 1, -1, dm_max);
    if (this->depth_map->at(max_x, 0, 0) > 0)
        bot_right[2] = this->depth_map->at(max_x, 0, 0);
    mve::math::Vec3d top_left (-1 , max_y + 1, dm_max);
    if (this->depth_map->at(0, max_y, 0) > 0)
        top_left[2] = this->depth_map->at(0, max_y, 0);
    mve::math::Vec3d top_right(max_x + 1, max_y + 1, dm_max);
    if (this->depth_map->at(max_x, max_y, 0) > 0)
        top_right[2] = this->depth_map->at(max_x, max_y, 0);

    Delaunay2D delaunay(bot_left, bot_right, top_left, top_right);

    /* create starting triangles */
    double triangle_vertices[9];
    delaunay.fill_triangle_vertices(0, triangle_vertices);
    this->triangles.emplace_back(0, triangle_vertices);
    this->triangles.back().heap_iterator = this->triangle_heap.emplace(1., 0);
    this->scan_triangle(0);
    delaunay.fill_triangle_vertices(1, triangle_vertices);
    this->triangles.emplace_back(1, triangle_vertices);
    this->triangles.back().heap_iterator = this->triangle_heap.emplace(1., 1);
    this->scan_triangle(1);

    std::vector<std::size_t> changed;
    for (int i = 0; i < max_vertices; ++i)
    {
        /* Exit if error is already small */
        if ((*this->triangle_heap.begin()).first < max_error)
            break;

        /* Insert new point */
        std::size_t tid = (*this->triangle_heap.begin()).second;
        Triangle t = this->triangles[tid];
        delaunay.insert_point(t.candidate, t.id);

        /* Rescan all changed triangles */
        delaunay.fill_recently_changed(&changed);
        for (std::size_t id: changed)
        {
            delaunay.fill_triangle_vertices(id, triangle_vertices);
            /* Create new triangle */
            if (id > this->triangles.size() - 1)
            {
                this->triangles.emplace_back(id, triangle_vertices);
                this->triangles.back().heap_iterator =
                    this->triangle_heap.emplace(
                        std::numeric_limits<double>::max(), id);
            } else
            /* Update existing triangle */
            {
                this->triangles[id].v1 = mve::math::Vec3d(triangle_vertices);
                this->triangles[id].v2 = mve::math::Vec3d(triangle_vertices + 3);
                this->triangles[id].v3 = mve::math::Vec3d(triangle_vertices + 6);
            }
            this->scan_triangle(id);
        }
    }

    /* Get Mesh, clean, transform, and return */
    mve::core::TriangleMesh::Ptr mesh = delaunay.get_mesh();
    std::vector<bool> delete_list(mesh->get_vertices().size(), false);
    delete_list[0] = delete_list[1] = delete_list[2] = delete_list[3] = true;
    mesh->delete_vertices_fix_faces(delete_list);
    mve::math::Matrix3f invproj;
    this->camera.fill_inverse_calibration(*invproj,
        this->depth_map->width(), this->depth_map->height());

    mve::core::TriangleMesh::VertexList & vertices = mesh->get_vertices();

    /* Add vertex colors if color image exists */
    if (this->color != nullptr)
    {
        mve::core::TriangleMesh::ColorList & vcolors = mesh->get_vertex_colors();
        vcolors.reserve(vertices.size());
        for (auto & vert : vertices)
        {
            mve::math::Vec4f vcolor(this->color->at(vert[0], vert[1], 0),
                0.0f, 0.0f, 255.0f);
            if (this->color->channels() >= 3)
            {
                vcolor[1] = this->color->at(vert[0], vert[1], 1);
                vcolor[2] = this->color->at(vert[0], vert[1], 2);
            } else
                vcolor[1] = vcolor[2] = vcolor[0];
            vcolors.push_back(vcolor / 255.0f);
        }
    }

    /* Project vertices along viewing ray */
    for (auto & vert : vertices)
    {
        mve::math::Vec3f ray = invproj * mve::math::Vec3f(vert[0] + 0.5f,
            vert[1] + 0.5f, 1.0f);
        vert = ray.normalized() * vert[2];
    }
    /* Transform into world coordinates */
    mve::math::Matrix4f ctw;
    this->camera.fill_cam_to_world(*ctw);
    mve::core::geom::mesh_transform(mesh, ctw);

    /* Remove bad faces */
    mve::core::TriangleMesh::FaceList & faces = mesh->get_faces();
    for (std::size_t f = 0; f < faces.size(); f += 3)
    {
        float edge1 = (vertices[faces[f]] - vertices[faces[f + 1]]).norm();
        float edge2 = (vertices[faces[f]] - vertices[faces[f + 2]]).norm();
        float edge3 = (vertices[faces[f + 1]] - vertices[faces[f + 2]]).norm();
        float min_edge = std::min(edge1, std::min(edge2, edge3));
        float max_edge = std::max(edge1, std::max(edge2, edge3));
        if (this->triangles[f/3].num_zero_depths > 4
            || min_edge / max_edge < 0.1)
            faces[f] = faces[f + 1] = faces[f + 2] =  0;
    }
    mesh->delete_invalid_faces();
    mve::core::geom::mesh_delete_unreferenced(mesh);

    mesh->recalc_normals();
    return mesh;
}

namespace
{
    struct Plane
    {
        Plane (mve::math::Vec3d p1, mve::math::Vec3d p2, mve::math::Vec3d p3)
        {
            mve::math::Vec3d normal = (p2 - p1).cross(p3 - p1).normalized();
            a = normal[0];
            b = normal[1];
            c = normal[2];
            d = -(p1.dot(normal));
        }
        double distance_to_point (mve::math::Vec3d p)
        {
            return std::fabs(a * p[0] + b * p[1] + c * p[2] + d);
        }
        double a, b, c, d;
    };
}

void
DepthTriangulator::scan_triangle (std::size_t id)
{
    Triangle & triangle = this->triangles[id];
    std::vector<mve::math::Vec2i> pixels;
    pixels_for_triangle(triangle.v1, triangle.v2, triangle.v3, &pixels);

    double max_dist = 0;
    mve::math::Vec3d max_dist_point(0.0, 0.0, 0.0);
    triangle.num_zero_depths = 0;
    Plane plane(triangle.v1, triangle.v2, triangle.v3);
    for (auto const& pixel : pixels)
    {
        if (pixel[0] < 0 || pixel[0] > this->depth_map->width() - 1
            || pixel[1] < 0 || pixel[1] > this->depth_map->height() - 1)
            continue;
        if (this->depth_map->at(pixel[0], pixel[1], 0) == 0)
        {
            triangle.num_zero_depths += 1;
            continue;
        }

        mve::math::Vec3d point(pixel[0], pixel[1],
            this->depth_map->at(pixel[0], pixel[1], 0));
        double distance = plane.distance_to_point(point);
        if (distance > max_dist)
        {
            max_dist = distance;
            max_dist_point = point;
        }
    }
    triangle.candidate = max_dist_point;
    this->triangle_heap.erase(triangle.heap_iterator);
    triangle.heap_iterator = this->triangle_heap.emplace(max_dist, id);
}

void
pixels_for_bottom_flat_triangle (mve::math::Vec3d const& a,
    mve::math::Vec3d const& b, mve::math::Vec3d const& c,
    std::vector<mve::math::Vec2i> * pixels)
{
    double dx_1 = (c[0] - a[0]) / (c[1] - a[1]);
    double dx_2 = (c[0] - b[0]) / (c[1] - b[1]);

    double x1 = c[0];
    double x2 = c[0];

    /* Top to bottom */
    for (int y = c[1]; y > b[1]; y--)
    {
        for (int x = (int)std::ceil(std::min(x1, x2));
             x <= (int)std::floor(std::max(x1, x2)); ++x)
            pixels->emplace_back(x,y);
        x1 -= dx_1;
        x2 -= dx_2;
    }
}

void
pixels_for_top_flat_triangle (mve::math::Vec3d const& a,
    mve::math::Vec3d const& b, mve::math::Vec3d const& c,
    std::vector<mve::math::Vec2i> * pixels)
{
    double dx_1 = (a[0] - c[0]) / (a[1] - c[1]);
    double dx_2 = (a[0] - b[0]) / (a[1] - b[1]);

    double x1 = a[0];
    double x2 = a[0];

    /* Bottom to top */
    for (int y = a[1]; y <= b[1]; y++)
    {
        for (int x = (int)std::ceil(std::min(x1, x2));
             x <= (int)std::floor(std::max(x1, x2)); ++x)
            pixels->emplace_back(x,y);
        x1 += dx_1;
        x2 += dx_2;
    }
}

void
DepthTriangulator::pixels_for_triangle (mve::math::Vec3d const& a,
    mve::math::Vec3d const& b, mve::math::Vec3d const& c,
    std::vector<mve::math::Vec2i> * pixels)
{
    std::vector<mve::math::Vec3d> verts;
    verts.push_back(a);
    verts.push_back(b);
    verts.push_back(c);
    std::sort(verts.begin(), verts.end(), [](mve::math::Vec3d a, mve::math::Vec3d b) {
        return a[1] < b[1];});
    pixels->reserve(MATH_POW2(verts[2][1] - verts[1][1]) / 2);

    if (verts[1][1] == verts[2][1])
        pixels_for_top_flat_triangle(verts[0], verts[1], verts[2], pixels);
    else if (verts[0][1] == verts[1][1])
        pixels_for_bottom_flat_triangle(verts[0], verts[1], verts[2], pixels);
    else
    {
        mve::math::Vec3d v(verts[0][0] + ((verts[1][1] - verts[0][1])
            / (verts[2][1] - verts[0][1])) * (verts[2][0] - verts[0][0]),
            verts[1][1], 0.0);
        if (verts[0][0] < verts[1][0])
            v[0] = std::ceil(v[0]);
        else
            v[0] = std::floor(v[0]);
        pixels_for_top_flat_triangle(verts[0], verts[1], v, pixels);
        pixels_for_bottom_flat_triangle(verts[1], v, verts[2], pixels);
    }

}

} // namespace smvs
