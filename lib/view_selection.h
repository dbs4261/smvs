/*
 * Copyright (c) 2016, Fabian Langguth
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef SMVS_VIEW_SELECTION_HEADER
#define SMVS_VIEW_SELECTION_HEADER

#include "mve/core/scene.h"

#include "defines.h"

namespace smvs {

class ViewSelection
{
public:
    struct Options
    {
        Options (void) = default;
        std::size_t num_neighbors = 6;
        std::string embedding = "undistorted";
    };

public:
    ViewSelection (Options const& opts, mve::core::Scene::ViewList const& views,
        mve::core::Bundle::ConstPtr bundle = nullptr);

    mve::core::Scene::ViewList get_neighbors_for_view(std::size_t const view) const;

private:
    mve::core::Scene::ViewList bundle_based_selection(std::size_t const view) const;
    mve::core::Scene::ViewList position_based_selection(std::size_t const view) const;
    mve::core::Scene::ViewList get_sorted_neighbors(std::size_t const view) const;

private:
    Options const& opts;
    mve::core::Scene::ViewList const& views;
    mve::core::Bundle::ConstPtr bundle;
};

inline
ViewSelection::ViewSelection (ViewSelection::Options const& opts,
    mve::core::Scene::ViewList const& views, mve::core::Bundle::ConstPtr bundle)
    : opts(opts), views(views), bundle(bundle)
{
}

} // namespace smvs

#endif /* SMVS_VIEW_SELECTION_HEADER */
