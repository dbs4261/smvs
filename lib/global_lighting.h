/*
 * Copyright (c) 2016, Fabian Langguth
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef SMVS_GLOBAL_LIGHTING_HEADER
#define SMVS_GLOBAL_LIGHTING_HEADER

#include "mve/math/vector.h"
#include "mve/core/image.h"

#include "spherical_harmonics.h"
#include "defines.h"

namespace smvs {

class GlobalLighting
{
public:
    typedef mve::math::Vector<double, 16> Params;
    typedef mve::math::Vector<double, 16> SHBasis;
    typedef std::shared_ptr<GlobalLighting> Ptr;

public:
    static Ptr create(Params const& parameters);

    Params const& get_parameters (void);

    mve::core::FloatImage::Ptr get_rendered_sphere (int dim) const;
    mve::core::FloatImage::Ptr render_normal_map (
        mve::core::FloatImage::ConstPtr normals) const;
    double value_for_normal (mve::math::Vec3d const& normal) const;

private:
    GlobalLighting (Params const& parameters);

private:
    Params parameters;
};

inline GlobalLighting::Ptr
GlobalLighting::create (const Params &parameters)
{
    return Ptr(new GlobalLighting(parameters));
}

inline
GlobalLighting::GlobalLighting (Params const& parameters)
    : parameters(parameters)
{
}

inline GlobalLighting::Params const&
GlobalLighting::get_parameters (void)
{
    return this->parameters;
}

} // namespace smvs

#endif /* SMVS_GLOBAL_LIGHTING_HEADER */
