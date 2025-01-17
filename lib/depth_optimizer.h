/*
 * Copyright (c) 2016, Fabian Langguth
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef SMVS_DEPTH_OPTIMIZER_HEADER
#define SMVS_DEPTH_OPTIMIZER_HEADER

#include "mve/core/depthmap.h"

#include "mve/math/matrix.h"

#include "stereo_view.h"
#include "surface.h"
#include "global_lighting.h"
#include "sse_vector.h"
#include "block_sparse_matrix.h"
#include "defines.h"
#include "correspondence.h"

namespace smvs {

class DepthOptimizer
{
public:
    struct Options
    {
        Options (void) = default;
        double regularization = 0.001;
        double light_surf_regularization = 0.0;
        int num_iterations = 10;
        int min_scale = 1;
        int debug_lvl = 0;
        bool use_shading = false;
        bool use_sgm = false;
        bool full_optimization = false;
        std::string output_name = "smvs";
    };

    typedef SSEVector DenseVector;
    typedef BlockSparseMatrix<4> SparseMatrix;

public:
    DepthOptimizer (StereoView::Ptr main_view,
        std::vector<StereoView::Ptr> const& sub_views,
        mve::core::Bundle::ConstPtr bundle,
        Options const& options);

    DepthOptimizer (StereoView::Ptr main_view,
        std::vector<StereoView::Ptr> const& sub_views,
        Surface::Ptr Surface,
        Options const& options);

    void optimize (void);

    mve::core::FloatImage::Ptr get_depth (void);
    mve::core::FloatImage::Ptr get_normals (void);

private:
    /* Initial preparations */
    void prepare_correspondences (void);
    void create_initial_surface (void);
    void create_subview_surfaces (void);

    /* Joint bilateral filter on initial depth map */
    mve::core::FloatImage::Ptr depthmap_bilateral_filter (
        mve::core::FloatImage::ConstPtr dm, mve::core::FloatImage::ConstPtr ci,
        float sigma = 5, int kernel_size = 5);

    /* Run Gauss-Newton Optimization */
    void run_newton_iterations (int num_iters);

    /* Householder operations */
    void get_non_converged_nodes(std::vector<mve::math::Vec2d> const& proj1,
        std::vector<mve::math::Vec2d> const& proj2,
        std::vector<std::size_t> * nodes);
    void fill_node_reprojections(std::vector<char> const& active_nodes,
        std::vector<std::pair<std::size_t, mve::math::Vec2d>> * proj);
    int cut_boundaries (void);

    /* Errors and values for patch */
    double mse_for_patch(std::size_t patch_id);
    double ncc_for_patch (std::size_t patch_id, std::size_t sub_id);
    double tex_score_for_patch (std::size_t patch_id);

    /* debug operations */
    void reproject_neighbor(std::size_t neighbor);
    void write_debug_depth (std::string const& postfix = "");

private:
    Options const& opts;

    mve::core::Bundle::ConstPtr bundle;

    StereoView::Ptr main_view;
    mve::core::FloatImage::ConstPtr main_gradients;
    mve::core::FloatImage::ConstPtr sgm_depth;

    std::vector<StereoView::Ptr> const& sub_views;
    std::vector<mve::math::Matrix3d> Mi;
    std::vector<mve::math::Vec3d> ti;

    Surface::Ptr surface;
    std::vector<std::vector<std::size_t>> subsurfaces;
    GlobalLighting::Ptr lighting;

    std::vector<mve::math::Vec2d> pixels;
    std::vector<std::size_t> pids;
    std::vector<double> depths;
    std::vector<mve::math::Vec2d> depth_derivatives;
    std::vector<mve::math::Vec3d> depth_2nd_derivatives;

    mve::math::Vec2d grad_main;
    mve::math::Vec2d grad_linear;
    mve::math::Vec2d grad_sub;
    mve::math::Vec2d proj;
    mve::math::Matrix2d jac;
};

/* ------------------------ Implementation ------------------------ */

inline
DepthOptimizer::DepthOptimizer (StereoView::Ptr main_view,
    std::vector<StereoView::Ptr> const& sub_views,
    Surface::Ptr surface, Options const& opts)
    : opts(opts), main_view(main_view), sub_views(sub_views), surface(surface)
{
    this->prepare_correspondences();
    this->bundle = nullptr;
}

inline
mve::core::FloatImage::Ptr
DepthOptimizer::get_depth (void)
{
    return this->surface->get_depth_map();
}

inline
mve::core::FloatImage::Ptr
DepthOptimizer::get_normals (void)
{
    return this->surface->get_normal_map(this->main_view->get_inverse_flen());
}

inline
void
DepthOptimizer::write_debug_depth (std::string const& postfix)
{
    if (this->opts.debug_lvl < 2)
        return;

    std::string name = "smvs-L" +
        mve::util::string::get(this->surface->get_scale()) + postfix;
    this->main_view->write_depth_to_view(this->surface->get_depth_map(), name);
}

} // namespace smvs

#endif /* SMVS_DEPTH_OPTIMIZER_HEADER */
