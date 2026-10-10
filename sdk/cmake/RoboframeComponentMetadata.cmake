# Called after a component includes its VERSION and links Roboframe::sdk.
set_property(GLOBAL PROPERTY ROBOFRAME_METADATA_TEMPLATE "${CMAKE_CURRENT_LIST_DIR}/component_version.hpp.in")
function(roboframe_component_metadata target namespace)
    get_property(ROBOFRAME_METADATA_TEMPLATE GLOBAL PROPERTY ROBOFRAME_METADATA_TEMPLATE)
    if(NOT component_name MATCHES "^[a-z][a-z0-9-]*$")
        message(FATAL_ERROR "VERSION: invalid component_name")
    endif()
    foreach(field component_version component_contract_version component_sdk_min_version)
        if(NOT "${${field}}" MATCHES "^[0-9]+[.][0-9]+[.][0-9]+$")
            message(FATAL_ERROR "VERSION: ${field} must be major.minor.patch")
        endif()
    endforeach()
    get_target_property(built_with_sdk Roboframe::sdk ROBOFRAME_SDK_VERSION)
    string(REGEX MATCH "^[0-9]+" required_major "${component_sdk_min_version}")
    string(REGEX MATCH "^[0-9]+" built_major "${built_with_sdk}")
    if(NOT built_major STREQUAL required_major OR built_with_sdk VERSION_LESS component_sdk_min_version)
        message(FATAL_ERROR "${component_name}: SDK ${built_with_sdk} does not satisfy >=${component_sdk_min_version}, same major")
    endif()
    string(REPLACE "." ", " contract_numbers "${component_contract_version}")
    string(REPLACE "." ", " sdk_min_numbers "${component_sdk_min_version}")
    configure_file("${ROBOFRAME_METADATA_TEMPLATE}"
        "${CMAKE_CURRENT_BINARY_DIR}/generated/${namespace}/version.hpp" @ONLY)
    target_include_directories(${target} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>)
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/generated/${namespace}/version.hpp"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${namespace}")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" manifest)
    file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/VERSION"
        "${manifest}\n# Generated build fact; do not edit.\nset(component_built_with_sdk \"${built_with_sdk}\")\n")
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/VERSION"
        DESTINATION "${CMAKE_INSTALL_DATADIR}/roboframe/${component_name}")
endfunction()
