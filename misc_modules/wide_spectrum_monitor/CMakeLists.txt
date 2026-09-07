cmake_minimum_required(VERSION 3.13)
project(wide_spectrum_monitor)

file(GLOB SRC "src/*.cpp")

include(${SDRPP_MODULE_CMAKE})

target_include_directories(wide_spectrum_monitor PRIVATE "../../decoder_modules/radio/src")
