add_executable(ba_my
        ba_my.cpp
)

include_directories(${CHOLMOD_INCLUDE_DIR})

target_link_libraries(ba_my core solver_cholmod types_sba)


