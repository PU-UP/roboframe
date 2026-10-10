# Source identity is automatic and includes uncommitted changes; no component version file.
set_property(GLOBAL PROPERTY ROBOFRAME_METADATA_TEMPLATE "${CMAKE_CURRENT_LIST_DIR}/component_version.hpp.in")
function(roboframe_component_metadata target namespace)
    get_property(ROBOFRAME_METADATA_TEMPLATE GLOBAL PROPERTY ROBOFRAME_METADATA_TEMPLATE)
    get_target_property(built_with_sdk Roboframe::sdk ROBOFRAME_SDK_VERSION)
    string(REPLACE "_" "-" component_name "${target}")
    file(GLOB_RECURSE source_files CONFIGURE_DEPENDS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
        "src/*" "include/*" "cmake/*")
    list(APPEND source_files CMakeLists.txt)
    list(SORT source_files)
    set(fingerprint "")
    foreach(path IN LISTS source_files)
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${path}")
        file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/${path}" hash)
        string(APPEND fingerprint "${path}:${hash}\n")
    endforeach()
    string(SHA256 source_id "${fingerprint}")
    string(REPLACE "." ", " sdk_numbers "${built_with_sdk}")
    configure_file("${ROBOFRAME_METADATA_TEMPLATE}"
        "${CMAKE_CURRENT_BINARY_DIR}/generated/${namespace}/version.hpp" @ONLY)
    target_include_directories(${target} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>)
    set_target_properties(${target} PROPERTIES
        ROBOFRAME_BUILT_WITH_SDK "${built_with_sdk}" ROBOFRAME_SOURCE_ID "${source_id}"
        EXPORT_PROPERTIES "ROBOFRAME_BUILT_WITH_SDK;ROBOFRAME_SOURCE_ID")
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/generated/${namespace}/version.hpp"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${namespace}")
    file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/VERSION"
        "# Generated build facts; do not edit.\n"
        "set(component_name \"${component_name}\")\n"
        "set(component_source_id \"${source_id}\")\n"
        "set(component_built_with_sdk \"${built_with_sdk}\")\n"
        "set(component_compiler \"${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}\")\n"
        "set(component_target \"${CMAKE_SYSTEM_NAME}/${CMAKE_SYSTEM_PROCESSOR}\")\n")
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/VERSION"
        DESTINATION "${CMAKE_INSTALL_DATADIR}/roboframe/${component_name}")
endfunction()
