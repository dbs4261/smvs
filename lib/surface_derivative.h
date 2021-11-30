/*
 * Copyright (c) 2016, Fabian Langguth
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef SMVS_SURFACE_DERIVATIVE_HEADER
#define SMVS_SURFACE_DERIVATIVE_HEADER

#include "defines.h"

namespace smvs {
namespace surfderiv {

void
fill_normal(double x, double y, double inv_flen, double w,
    double dx, double dy, double * n);

void
normal_derivative(double const* d_node, double x, double y, double f,
    double w, double dx, double dy, // double dxy, double dxx, double dyy,
    double * deriv);

void
normal_divergence (double x, double y, double f, double w,
    double dx, double dy, double dxy, double dxx, double dyy,
    double * div);

void
normal_divergence_deriv (double const* d_node, double x, double y, double f,
    double w, double dx, double dy, double dxy, double dxx, double dyy,
    double * full_deriv);

double
mean_curvature (double dx, double dy, double dxy, double dxx, double dyy);

void
mean_curvature_derivative (double const* d_node, double dx,
    double dy, double dxy, double dxx, double dyy, double * deriv);

} // namespace surfderiv
} // namespace smvs

#endif /* SMVS_SURFACE_DERIVATIVE_HEADER */
