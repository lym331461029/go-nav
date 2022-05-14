cmake_minimum_required(VERSION 3.0)

if ( NOT DEFINED BUILD_SHARED_LIBS)
    set(BUILD_SHARED_LIBS ON)
endif()

file(GLOB SOURCES ./*.cpp ../RecastDemo/Contrib/fastlz/fastlz.c)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")
#set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")

add_library(navigation ${SOURCES} Include/navigation.h)
set_property(TARGET navigation PROPERTY POSITION_INDEPENDENT_CODE ON)

#set(CMAKE_APPLE_SILICON_PROCESSOR arm64)
include_directories(SYSTEM ../RecastDemo/Contrib/fastlz)
include_directories(SYSTEM ../RecastDemo/Contrib)
include_directories(../DebugUtils/Include)
include_directories(../Detour/Include)
include_directories(../DetourCrowd/Include)
include_directories(../DetourTileCache/Include)
include_directories(../Recast/Include)
include_directories(./)


#set_target_properties(navigation PROPERTIES
#        SOVERSION ${SOVERSION}
#        VERSION ${LIB_VERSION}
#        COMPILE_PDB_OUTPUT_DIRECTORY .
#        COMPILE_PDB_NAME "Detour-d"
#        )



add_dependencies(navigation DebugUtils Detour DetourCrowd DetourTileCache Recast)
target_link_libraries(navigation DebugUtils Detour DetourCrowd DetourTileCache Recast)


file(GLOB INCLUDES Include/*.h)
install(FILES ${INCLUDES} DESTINATION
        ${CMAKE_INSTALL_INCLUDEDIR})

install(TARGETS navigation
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        COMPONENT library
        )
install(DIRECTORY ../RecastDemo/Bin/Meshes DESTINATION bin)
install(DIRECTORY ../RecastDemo/Bin/TestCases DESTINATION bin)
install(FILES ../RecastDemo/Bin/DroidSans.ttf DESTINATION bin)



