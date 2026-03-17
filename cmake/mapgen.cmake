set(MAP_BIN "${CMAKE_CURRENT_SOURCE_DIR}/data/map_data.bin")
set(MAP_HDR "${CMAKE_CURRENT_BINARY_DIR}/MapData.h")

add_custom_command(
    OUTPUT "${MAP_HDR}"
    # Wrap the entire pipeline in 'sh -c'
    COMMAND sh -c "printf '#pragma once\\n#include <stdint.h>\\n' > \"${MAP_HDR}\" && xxd -i -n map_data_bin ${MAP_BIN} | sed -e 's/unsigned char/const uint8_t/g' -e 's/unsigned int/const unsigned int/g' >> \"${MAP_HDR}\""
    # Run from the data directory so xxd creates a clean variable name (finalMapData_bin)
    DEPENDS "${MAP_BIN}"
    COMMENT "Generating map_data.h using xxd..."
    VERBATIM
)