# Product maintainer selects the SDK and component packages here.
set(ROBOFRAME_REQUIRED_SDK_VERSION "4.1.0")
function(roboframe_load_components)
    roboframe_component(odom Odom)
    roboframe_component(lidar-map LidarMap)
    roboframe_component(map-updater MapUpdater)
    roboframe_component(map-server MapServer)
    roboframe_component(planner Planner)
endfunction()
