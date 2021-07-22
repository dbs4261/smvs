set(SMVS_SOURCES
    bicubic_patch.cc
    light_optimizer.cc
    correspondence.cc
    mesh_simplifier.cc
    depth_optimizer.cc
    global_lighting.cc
    gauss_newton_step.cc
    delaunay_2d.cc
    depth_triangulator.cc
    mesh_generator.cc
    sgm_stereo.cc
    sse_vector.cc
    stereo_view.cc
    surface.cc
    surface_derivative.cc
    surface_patch.cc
    view_selection.cc
)
set(SMVS_PUBLIC_HEADERS
    bicubic_patch.h
    block_sparse_matrix.h
    conjugate_gradient.h
    correspondence.h
    defines.h
    delaunay_2d.h
    depth_optimizer.h
    depth_triangulator.h
    gauss_newton_step.h
    global_lighting.h
    ldl_decomposition.h
    light_optimizer.h
    mesh_generator.h
    mesh_simplifier.h
    quad_edge.h
    sgm_stereo.h
    spherical_harmonics.h
    sse_vector.h
    stereo_view.h
    surface.h
    surface_derivative.h
    surface_patch.h
    thread_pool.h
    view_selection.h
)

add_library(smvs ${SMVS_SOURCES})
add_library(smvs::lib ALIAS smvs)
set_target_properties(smvs PROPERTIES
    PUBLIC_HEADER "${SMVS_PUBLIC_HEADERS}"
    EXPORT_NAME lib
)

target_include_directories(smvs PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/smvs/>
)
target_link_libraries(smvs PUBLIC mve::core Threads::Threads PRIVATE smvs::settings)

install(TARGETS smvs EXPORT smvs-targets
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/smvs/
)
