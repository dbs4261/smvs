/*
 * Copyright (c) 2016, Fabian Langguth
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef SMVS_STEREO_VIEW_HEADER
#define SMVS_STEREO_VIEW_HEADER

#include "mve/core/view.h"
#include "mve/core/bundle.h"
#include "mve/core/depthmap.h"

#include "defines.h"

namespace smvs {

class StereoView
{
public:
    typedef std::shared_ptr<StereoView> Ptr;
    typedef std::shared_ptr<const StereoView> ConstPtr;

public:
    static StereoView::Ptr create (mve::core::View::Ptr view,
        std::string const& image_embedding, bool initialize_linear = false,
        bool gamma_correction = false);

    void set_scale(int scale, bool debug = false);

    int get_width (void) const;
    int get_height (void) const;
    int get_view_id (void) const;
    mve::core::CameraInfo const& get_camera (void) const;
    float get_flen (void) const;
    float get_inverse_flen (void) const;
    mve::core::ByteImage::ConstPtr get_byte_image (void) const;
    mve::core::FloatImage::ConstPtr get_image (void) const;
    mve::core::FloatImage::ConstPtr get_scaleimage (void) const;
    mve::core::FloatImage::ConstPtr get_image_gradients (void) const;
    mve::core::FloatImage::ConstPtr get_image_hessian (void) const;
    mve::core::FloatImage::Ptr get_sgm_depth (void) const;
    mve::core::FloatImage::ConstPtr get_shading_image (void) const;
    mve::core::FloatImage::ConstPtr get_shading_gradients (void) const;
    mve::core::FloatImage::ConstPtr get_linear_image (void) const;

    void write_image_to_view (mve::core::ImageBase::Ptr image,
        std::string const& name);

    void write_depth_to_view (mve::core::FloatImage::Ptr depth,
        std::string const& name);

    mve::core::FloatImage::Ptr get_debug_image (void);
    void write_debug (int num = 0);

private:
    StereoView (mve::core::View::Ptr view, std::string const& image_embedding);

    void initialize_image_gradients (mve::core::FloatImage::ConstPtr image);
    void compute_gradients_and_hessian (mve::core::FloatImage::ConstPtr input,
        mve::core::FloatImage::Ptr gradient, mve::core::FloatImage::Ptr hessian = nullptr);

    void initialize_linear (bool gamma_correction);

private:
    mve::core::View::Ptr view;
    std::string const image_embedding;
    mve::core::FloatImage::ConstPtr image;
    mve::core::FloatImage::Ptr scaleimage;
    mve::core::FloatImage::Ptr image_grad;
    mve::core::FloatImage::Ptr image_hessian;
    mve::core::FloatImage::Ptr linear_image;
    mve::core::FloatImage::Ptr debug;
    mve::core::FloatImage::Ptr shading;
    mve::core::FloatImage::Ptr shading_grad;
};

/* ------------------------ Implementation ------------------------ */

inline
StereoView::Ptr
StereoView::create(mve::core::View::Ptr view, const std::string &image_embedding,
    bool initialize_linear, bool gamma_correction)
{
    Ptr stereo_view(new StereoView(view, image_embedding));
    if (initialize_linear)
        stereo_view->initialize_linear(gamma_correction);
    return stereo_view;
}

inline void
StereoView::write_image_to_view(mve::core::ImageBase::Ptr image,
    std::string const& name)
{
    this->view->set_image(image, name);
    this->view->save_view();
}

inline mve::core::CameraInfo const&
StereoView::get_camera (void) const
{
    return this->view->get_camera();
}

inline void
StereoView::write_depth_to_view(mve::core::FloatImage::Ptr depth,
    std::string const& name)
{
    mve::core::FloatImage::Ptr mve_depth = depth->duplicate();
    mve::math::Matrix3f invproj;
    this->get_camera().fill_inverse_calibration(
        *invproj, mve_depth->width(), mve_depth->height());
    mve::core::image::depthmap_convert_conventions<float>(mve_depth, invproj, true);
    this->view->set_image(mve_depth, name);
    this->view->save_view();
}

inline mve::core::FloatImage::Ptr
StereoView::get_sgm_depth (void) const
{
    mve::core::FloatImage::Ptr mve_depth = this->view->get_float_image("smvs-sgm");
    mve::math::Matrix3f invproj;
    this->get_camera().fill_inverse_calibration(
        *invproj, mve_depth->width(), mve_depth->height());
    mve::core::image::depthmap_convert_conventions<float>(mve_depth, invproj, false);
    return mve_depth;
}

inline float
StereoView::get_flen (void) const
{
    mve::math::Matrix3f proj;
    this->get_camera().fill_calibration(
        *proj, this->get_width(), this->get_height());
    return proj[0];
}

inline float
StereoView::get_inverse_flen (void) const
{
    mve::math::Matrix3f invproj;
    this->get_camera().fill_inverse_calibration(
       *invproj, this->get_width(), this->get_height());
    return invproj[0];
}


inline mve::core::FloatImage::ConstPtr
StereoView::get_image (void) const
{
    return this->image;
}

inline mve::core::FloatImage::ConstPtr
StereoView::get_scaleimage (void) const
{
    return this->scaleimage;
}

inline mve::core::FloatImage::ConstPtr
StereoView::get_image_gradients (void) const
{
    return this->image_grad;
}

inline mve::core::FloatImage::ConstPtr
StereoView::get_image_hessian (void) const
{
    return this->image_hessian;
}

inline mve::core::FloatImage::ConstPtr
StereoView::get_linear_image (void) const
{
    return this->linear_image;
}

inline mve::core::FloatImage::ConstPtr
StereoView::get_shading_image (void) const
{
    return this->shading;
}

inline mve::core::FloatImage::ConstPtr
StereoView::get_shading_gradients (void) const
{
    return this->shading_grad;
}

inline int
StereoView::get_width (void) const
{
    return this->image->width();
}

inline int
StereoView::get_height (void) const
{
    return this->image->height();
}

inline int
StereoView::get_view_id (void) const
{
    return this->view->get_id();
}

/********************************* Debug **************************************/

inline mve::core::FloatImage::Ptr
StereoView::get_debug_image (void)
{
    if (this->debug == nullptr)
        this->debug = mve::core::FloatImage::create(this->image->width(),
            this->image->height(), 3);
    return this->debug;
}

inline void
StereoView::write_debug (int num)
{
    std::string name = "smvs-debug-" + mve::util::string::get_filled(num, 2);
    this->view->set_image(this->debug, name);
    this->view->save_view();
    this->view->cache_cleanup();
}

} // namespace smvs

#endif /* SMVS_STEREO_VIEW_HEADER */
