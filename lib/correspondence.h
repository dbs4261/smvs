/*
 * Copyright (c) 2016, Fabian Langguth
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef SMVS_CORRESPONDENCE_HEADER
#define SMVS_CORRESPONDENCE_HEADER

#include "mve/math/matrix.h"
#include "mve/math/vector.h"

#include "defines.h"

namespace smvs {

class Correspondence
{
public:
    Correspondence (void) { }
    Correspondence (mve::math::Matrix3d const& M, mve::math::Vec3d const& t,
        double u, double v, double w, double w_dx = 0, double w_dy = 0);

    void update (mve::math::Matrix3d const& M, mve::math::Vec3d const& t,
        double u, double v, double w, double w_dx = 0, double w_dy = 0);
    
    void fill (double * corr) const;
    void get_derivative (
        double const* dn00, double const* dn10,
        double const* dn01, double const* dn11,
        mve::math::Vec2d * c_dn00,
        mve::math::Vec2d * c_dn10,
        mve::math::Vec2d * c_dn01,
        mve::math::Vec2d * c_dn11) const;
    void fill_derivative (double const* dn, mve::math::Vec2d * c_dn) const;

    void fill_jacobian(double * jac) const;

    void fill_jacobian_derivative_grad(double const* grad,
        double const* dn, mve::math::Vec2d * jac_dn) const;

    double get_depth (void) const;

private:
    double p;
    double q;
    double r;

    mve::math::Vec3d t;

    double w;
    mve::math::Vec2d w_prime;

    double a;
    double b;
    double d;
    double d2;

    mve::math::Vec2d p_prime;
    mve::math::Vec2d q_prime;
    mve::math::Vec2d r_prime;
};

inline double
Correspondence::get_depth (void) const
{
    return this->d;
}

} // namespace smvs

#endif /* SMVS_CORRESPONDENCE_HEADER */
