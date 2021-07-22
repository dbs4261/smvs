add_executable(smvsrecon smvsrecon.cc)
target_link_libraries(smvsrecon smvs::lib smvs::settings)

install(TARGETS smvsrecon RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/smvs/)