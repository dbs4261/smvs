add_executable(simplify simplify.cc)
target_link_libraries(simplify smvs smvs::settings)

install(TARGETS simplify RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/smvs/)