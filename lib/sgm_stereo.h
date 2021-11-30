/*
 * Copyright (c) 2016, Fabian Langguth
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef SMVS_SGM_STEREO_HEADER
#define SMVS_SGM_STEREO_HEADER

#include "mve/core/image.h"
#include "mve/util/aligned_memory.h"

#include "stereo_view.h"
#include "defines.h"

namespace smvs {

class SGMStereo
{
public:
    struct Options
    {
        Options (void) = default;
        int debug_lvl = 0;
        int scale = 1;
        int num_steps = 128;
        float min_depth = 0.0f;
        float max_depth = 0.0f;
        uint16_t penalty1 = 6;
        uint16_t penalty2 = 96;
    };

    SGMStereo (Options const& opts,
       StereoView::Ptr main, StereoView::Ptr neighbor);

    static mve::core::FloatImage::Ptr reconstruct (SGMStereo::Options sgm_opts,
        StereoView::Ptr main_view, StereoView::Ptr neighbor,
        mve::core::Bundle::ConstPtr bundle = nullptr);

    mve::core::FloatImage::Ptr run_sgm (float min_depth, float max_depth);

private:
    void warped_neighbors_for_depth (std::vector<float> const& depths,
        mve::core::ByteImage::Ptr image);

    void census_filter (mve::core::ByteImage::ConstPtr image,
        mve::core::Image<uint64_t>::Ptr filtered);

    void create_cost_volume (float min_depth, float max_depth, int num_steps);

    void aggregate_sgm_costs (void);

    void fill_path_cost (int x, int y, int px, int py,
        mve::core::RawImage::Ptr path);

    void fill_path_cost_sse (int base, int base_prev,
        mve::util::AlignedMemory<uint16_t> * path);
    void copy_cost_and_add_to_sgm (mve::util::AlignedMemory<uint16_t> * local_volume,
        int base);
    uint16_t sse_reduction_min (uint16_t * data, std::size_t size);

    mve::core::FloatImage::Ptr depth_from_cost_volume (void);
    mve::core::FloatImage::Ptr depth_from_sgm_volume (void);

    static void fill_depth_range_for_view (mve::core::Bundle::ConstPtr bundle,
        StereoView::Ptr view, float * range);

private:
    Options opts;

    StereoView::Ptr main;
    StereoView::Ptr neighbor;
    mve::core::ByteImage::ConstPtr main_image;
    mve::core::ByteImage::ConstPtr neighbor_image;

    mve::core::ByteImage::Ptr cost_volume;
    mve::core::RawImage::Ptr sgm_volume;
    std::vector<float> cost_volume_depths;

    mve::util::AlignedMemory<uint16_t> sse_cost_volume;
    mve::util::AlignedMemory<uint16_t> sse_sgm_volume;
    mve::util::AlignedMemory<uint16_t> min_cost_updates;
    mve::util::AlignedMemory<uint16_t> cost_updates;
    mve::util::AlignedMemory<uint16_t> mins;
};

} // namespace smvs

#endif /* SMVS_SGM_STEREO_HEADER */
