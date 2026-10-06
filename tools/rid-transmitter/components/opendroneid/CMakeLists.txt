# The OpenDroneID reference library (Apache-2.0), from the submodule at
# third_party/opendroneid-core-c. Only the encoder and Wi-Fi frame builders
# are needed.
set(ODC ${CMAKE_CURRENT_LIST_DIR}/../../../../third_party/opendroneid-core-c/libopendroneid)
idf_component_register(
    SRCS "${ODC}/opendroneid.c" "${ODC}/wifi.c"
    INCLUDE_DIRS "${ODC}"
)
# Third-party code: don't let its warnings fail our build.
target_compile_options(${COMPONENT_LIB} PRIVATE -w)
