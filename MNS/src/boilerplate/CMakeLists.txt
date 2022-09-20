add_library(boilerplate_loop_functions MODULE
   boilerplate_loop_functions.h
   boilerplate_loop_functions.cpp)

target_link_libraries(boilerplate_loop_functions
   ${ARGOS_CORE_LIBRARY}
   ${ARGOS_ENTITIES_LIBRARY}
   ${ARGOS_PROTOTYPE_LIBRARY})

install(TARGETS boilerplate_loop_functions
   RUNTIME DESTINATION bin
   LIBRARY DESTINATION lib/argos3
   ARCHIVE DESTINATION lib/argos3)

