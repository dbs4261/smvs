add_executable(simplify simplify.cc)
target_link_libraries(simplify smvs smvs::settings)
set_target_properties(simplify PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION
        $<AND:$<BOOL:${CMAKE_CXX_LTO_SUPPORTED}>, $<BOOL:${ENABLE_IPO}>, $<NOT:$<CONFIG:Debug>>>
)

install(TARGETS simplify RUNTIME COMPONENT SMVS_Tools)