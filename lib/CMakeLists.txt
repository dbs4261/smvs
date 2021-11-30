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
    EXPORT_NAME smvs
    DEBUG_POSTFIX "_debug"
    RELEASE_POSTFIX "_release"
    RELWITHDEBINFO_POSTFIX "_relwithdebinfo"
    VERSION "${ShadingAwareMVS_VERSION}"
    SOVERSION "${ShadingAwareMVS_VERSION_MAJOR}"
    INTERPROCEDURAL_OPTIMIZATION
    $<AND:$<BOOL:${CMAKE_CXX_LTO_SUPPORTED}>, $<BOOL:${ENABLE_IPO}>, $<NOT:$<CONFIG:Debug>>>
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)
target_link_libraries(smvs
    PUBLIC
        mve::core Threads::Threads
    PRIVATE
        smvs::settings
)
target_include_directories(smvs PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/smvs/>
)

target_compile_options(smvs PUBLIC
    $<$<BOOL:${HAS_SSE41}>:-msse4.1>
    $<$<BOOL:${HAS_POPCNT}>:-mpopcnt>
)

install(TARGETS smvs EXPORT smvs-targets
    RUNTIME COMPONENT SMVS_Runtime
    LIBRARY COMPONENT SMVS_Runtime NAMELINK_COMPONENT SMVS_Development
    ARCHIVE COMPONENT SMVS_Development
    )
install(FILES ${MVE_CORE_PUBLIC_HEADER}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/smvs/
    COMPONENT SMVS_Development
)
