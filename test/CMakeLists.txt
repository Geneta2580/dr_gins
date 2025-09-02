# test/test pipelines and nodes (similar pattern to reference project)
set(TEST_SOURCES
  test_pipeline
)

foreach(test_src ${TEST_SOURCES})
  # create test executable (do not install as package runtime by default)
  add_executable(${test_src} ${test_src}.cpp ${BACKWARD_ENABLE})
  target_link_libraries(${test_src}
    dr_gins_core
    ${THIRD_PARTY_LIBS}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
  )

  # if backward-cpp provides add_backward macro, use it to add debug symbols/backtrace support
  if(COMMAND add_backward)
    add_backward(${test_src})
  endif()

  # Optionally install a runtime node (uncomment if you want rosrun/roslaunch to use this test binary)
  install(TARGETS ${test_src}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endforeach()