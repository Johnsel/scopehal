cmake_minimum_required(VERSION 3.16.3)

project(liblitepcie)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(WDK REQUIRED)


#find_package(SimpleITK)

add_library ( liblitepcie SHARED  litepcie_flash.c  litepcie_helpers.c litepcie_dma.c )
target_link_libraries(liblitepcie setupapi)
target_include_directories(liblitepcie PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

#target_link_libraries ( sitk_example ${SimpleITK_LIBRARIES} )