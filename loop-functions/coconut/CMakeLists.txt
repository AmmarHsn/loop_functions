# Set up ARGoS compilation information
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})
link_directories(
	${CMAKE_BINARY_DIR}/src
    )

add_library(aggregation_one_spot SHARED AggregationOneSpotLoopFunc.h AggregationOneSpotLoopFunc.cpp)
target_link_libraries(aggregation_one_spot argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(foraging_two_spots SHARED ForagingTwoSpotsLoopFunc.h ForagingTwoSpotsLoopFunc.cpp)
target_link_libraries(foraging_two_spots argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(fast_point_finding SHARED FastPointFindingLoopFunc.h FastPointFindingLoopFunc.cpp)
target_link_libraries(fast_point_finding argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(grid_exploration SHARED GridExplorationLoopFunc.h GridExplorationLoopFunc.cpp)
target_link_libraries(grid_exploration argos3plugin_${ARGOS_BUILD_FOR}_epuck)
