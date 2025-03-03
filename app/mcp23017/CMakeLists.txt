add_library(mcp23017 STATIC)

target_sources(mcp23017 PRIVATE 
    "mcp23017.cpp"
)

target_include_directories(mcp23017 PUBLIC 
    "."
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(mcp23017 PRIVATE
    utility
    stm32cubemx
)

target_compile_options(mcp23017 PUBLIC
    -std=c++23
    -Wall
    -Wextra
    -Wconversion
    -Wshadow
    -Wpedantic
    -Wnarrowing
    -Waddress
    -pedantic
    -Wdeprecated
    -Wsign-conversion
    -Wduplicated-cond
    -Wduplicated-branches
    -Wlogical-op
    -Wnull-dereference
    -Wdouble-promotion
    -Wimplicit-fallthrough
    -Wcast-align
    -fconcepts
)