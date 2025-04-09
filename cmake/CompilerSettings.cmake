option(READ_EVENT_FROM_AESTREAM "read events from aestream instead of csv file" ON)
option(OUPUT_PCL_TO_USER "" OFF)
option(OUTPUT_DEPTH_MAPS_TO_USER "" OFF)
# Add more options here

if(READ_EVENT_FROM_AESTREAM)
    target_compile_definitions(my_executable PRIVATE READ_EVENT_FROM_AESTREAM)
endif()
if(OUPUT_PCL_TO_USER)
    target_compile_definitions(my_executable PRIVATE OUPUT_PCL_TO_USER)
endif()
if(OUTPUT_DEPTH_MAPS_TO_USER)
    target_compile_definitions(my_executable PRIVATE OUTPUT_DEPTH_MAPS_TO_USER)
endif()