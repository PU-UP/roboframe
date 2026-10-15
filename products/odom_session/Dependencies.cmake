set(ROBOFRAME_REQUIRED_SDK_VERSION "10.0.0")
# Product main owns log ingress. A normal variable shadows the cache only for this configure,
# so a later component-only build of odom can still enable its receiver.
set(ODOM_BUILD_RECEIVER OFF)
function(roboframe_load_components)
    roboframe_component(odom Odom)
    roboframe_component(path_distance PathDistance)
    roboframe_component(distance_goal DistanceGoal)
endfunction()
