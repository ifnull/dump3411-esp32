# Native (non-ESP) build of the odid component plus the tools the parity
# tests use. No ESP-IDF needed:
#
#   cmake -S host -B build-host && cmake --build build-host
#   python3 tests/parity/run_parity.py
cmake_minimum_required(VERSION 3.16)
project(odid_host C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(ODID_DIR ${CMAKE_CURRENT_LIST_DIR}/../components/odid)
set(ODID_WARNINGS -Wall -Wextra -Werror -Wshadow -Wdouble-promotion)

# The parity fixtures include deliberately malformed frames; run them under
# ASan/UBSan so an out-of-bounds read fails the test instead of passing quietly.
option(ODID_SANITIZE "Build host tools with AddressSanitizer + UBSan" ON)
if(ODID_SANITIZE)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
    add_link_options(-fsanitize=address,undefined)
endif()

add_library(odid STATIC ${ODID_DIR}/odid_parser.c ${ODID_DIR}/odid_wifi.c ${ODID_DIR}/odid_tracker.c)
target_include_directories(odid PUBLIC ${ODID_DIR}/include)
target_compile_options(odid PRIVATE ${ODID_WARNINGS})
target_link_libraries(odid PUBLIC m)

# Hex in on stdin, JSON out: the C side of the parity comparison.
add_executable(odid_dump odid_dump.c)
target_compile_options(odid_dump PRIVATE ${ODID_WARNINGS})
target_link_libraries(odid_dump PRIVATE odid)

# Timed script in, snapshots out: the C side of the tracker comparison.
add_executable(tracker_sim tracker_sim.c)
target_compile_options(tracker_sim PRIVATE ${ODID_WARNINGS})
target_link_libraries(tracker_sim PRIVATE odid)

# Regenerates tests/parity/fixtures/encoder.txt from the OpenDroneID reference
# encoder (the library ArduRemoteID transmits with). Off by default because it
# downloads that library; the generated fixtures are committed.
option(ODID_FIXTURE_GEN "Build gen_fixtures (fetches opendroneid-core-c)" OFF)
if(ODID_FIXTURE_GEN)
    include(FetchContent)
    set(OPENDRONEID_CORE_C_COMMIT 6484f26545d4f012682524e2d843fab0fbdc0b34)
    FetchContent_Declare(opendroneid_core_c
        GIT_REPOSITORY https://github.com/opendroneid/opendroneid-core-c.git
        GIT_TAG        ${OPENDRONEID_CORE_C_COMMIT}
        # Only fetch; we compile the two sources we need below.
        SOURCE_SUBDIR  do-not-add-subdirectory
    )
    FetchContent_MakeAvailable(opendroneid_core_c)
    set(ODC ${opendroneid_core_c_SOURCE_DIR}/libopendroneid)

    add_executable(gen_fixtures gen_fixtures.c ${ODC}/opendroneid.c ${ODC}/wifi.c)
    target_include_directories(gen_fixtures PRIVATE ${ODC})
    target_compile_definitions(gen_fixtures PRIVATE
        OPENDRONEID_CORE_C_COMMIT="${OPENDRONEID_CORE_C_COMMIT}")
    target_link_libraries(gen_fixtures PRIVATE m)
endif()
