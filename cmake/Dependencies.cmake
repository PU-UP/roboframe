# Product maintainer selects the SDK and component packages here.
set(ROBOFRAME_REQUIRED_SDK_VERSION "2.0.0")
function(roboframe_load_components)
    roboframe_component(odom Odom)
    roboframe_component(map-updater MapUpdater)
    roboframe_component(map-server MapServer)
endfunction()
