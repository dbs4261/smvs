/*
 * Copyright (c) 2016, Fabian Langguth
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef SMVS_GAUSS_NEWTON_STEP_HEADER
#define SMVS_GAUSS_NEWTON_STEP_HEADER

#include "mve/math/vector.h"
#include "mve/math/matrix.h"

#include "defines.h"
#include "sse_vector.h"
#include "block_sparse_matrix.h"
#include "surface.h"
#include "correspondence.h"
#include "global_lighting.h"

namespace smvs {

class GaussNewtonStep
{
public:
    struct Options
    {
        Options (void) = default;
        double regularization = 0.001;
        double light_surf_regularization = 0.0;
        double l1_min_factor = 1e-4;
    };

    typedef SSEVector DenseVector;
    typedef BlockSparseMatrix<4> SparseMatrix;

public:
    GaussNewtonStep (Options const& opts,
        StereoView::ConstPtr main_view,
        std::vector<StereoView::Ptr> const& sub_views,
        std::vector<mve::math::Matrix3d> const& Mi,
        std::vector<mve::math::Vec3d> const& ti);

    void construct (Surface::Ptr surface,
        std::vector<std::vector<std::size_t>> const& subsurfaces,
        std::vector<char> const& active_nodes,
        GlobalLighting::Ptr lighting,
        SparseMatrix * hessian, DenseVector * gradient, SparseMatrix * precond);

private:
    void jacobian_entries_for_patch (int const scale, Surface::Patch::Ptr patch,
        std::vector<std::size_t> const& patch_neighbors,
        std::vector<double> const& node_derivatives, double * gradient,
        double * hessian_entries);
    void fill_gradient_and_hessian_entries(std::size_t i, std::size_t num_subs,
        double * gradient, double * hessian_entries);


private:
    Options const& opts;
    StereoView::ConstPtr main_view;
    std::vector<StereoView::Ptr> const& sub_views;
    std::vector<mve::math::Matrix3d> const& Mi;
    std::vector<mve::math::Vec3d> const& ti;
    mve::core::FloatImage::ConstPtr main_gradients;
    mve::core::FloatImage::ConstPtr main_gradients_linear;

    GlobalLighting::Ptr lighting;

    std::vector<mve::math::Vec2d> pixels;
    std::vector<std::size_t> pids;
    std::vector<double> depths;
    std::vector<mve::math::Vec2d> depth_derivatives;
    std::vector<mve::math::Vec3d> depth_2nd_derivatives;

    mve::math::Vec2d c_dn[16];
    mve::math::Vec2d jac_dn[16];
    double full_surface_div[6];
    double full_surface_div_deriv[96];
    double normal_deriv[48];
    mve::util::AlignedMemory<mve::math::Vec2d, 16> j_grad_subs;
    mve::util::AlignedMemory<mve::math::Vec2d, 16> jac_entries;

    std::vector<double> p_diffs;
    std::vector<double> p_weights;
    mve::math::Vec2d grad_main;
    mve::math::Vec2d grad_linear;
    mve::math::Vec2d grad_sub;
    mve::math::Matrix2d hess_sub;
    mve::math::Vec2d proj;
    mve::math::Matrix2d jac;
    mve::math::Matrix2d jac_hess;
    mve::util::AlignedMemory<mve::math::Vec2d, 16> reg_grad_mem;
    mve::util::AlignedMemory<mve::math::Vec2d, 16> reg_hessian_mem;
    double basic_regularizer_weight;
    Correspondence C;

};

} // namespace smvs

#endif /* SMVS_GAUSS_NEWTON_STEP_HEADER */
