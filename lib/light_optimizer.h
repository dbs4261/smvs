/*
 * Copyright (c) 2016, Fabian Langguth
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef SMVS_LIGHT_OPTIMIZER_HEADER
#define SMVS_LIGHT_OPTIMIZER_HEADER

#include "surface.h"
#include "global_lighting.h"

#include "defines.h"

namespace smvs {

class LightOptimizer
{
public:
    LightOptimizer(Surface::Ptr surface, StereoView::Ptr view);

    GlobalLighting::Ptr fit_lighting_to_image (mve::core::FloatImage::ConstPtr image);

private:
    Surface::Ptr surface;
    StereoView::Ptr view;
};

} // namespace smvs

#endif /* SMVS_LIGHT_OPTIMIZER_HEADER */
