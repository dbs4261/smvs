add_executable(smvsrecon smvsrecon.cc)
target_link_libraries(smvsrecon smvs::lib smvs::settings)
set_target_properties(smvsrecon PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION
        $<AND:$<BOOL:${CMAKE_CXX_LTO_SUPPORTED}>, $<BOOL:${ENABLE_IPO}>, $<NOT:$<CONFIG:Debug>>>
)

install(TARGETS smvsrecon RUNTIME COMPONENT SMVS_Applications)