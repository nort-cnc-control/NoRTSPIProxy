cmake_minimum_required(VERSION 3.13)
project(NoRTSPIProxy)


include(FindPkgConfig)
pkg_search_module(GPIOD REQUIRED libgpiod)

add_executable(nort_spi_proxy main.c)
target_link_libraries(nort_spi_proxy pthread)

target_include_directories(nort_spi_proxy PUBLIC ${GPIOD_INCLUDE_DIRS})
target_compile_options(nort_spi_proxy PUBLIC ${GPIOD_CFLAGS_OTHER})
target_link_libraries(nort_spi_proxy ${GPIOD_LIBRARIES})

install(TARGETS nort_spi_proxy DESTINATION bin)

