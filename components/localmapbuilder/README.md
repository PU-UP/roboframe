# LocalMapBuilder

LocalMapComponent 将 odom 与同一时刻 scan 堆成局部子图，默认 16 帧一组。
调用者可以要求开始新锚点；当前产品在定位修正变化时使用它，组件自身不理解 SLAM 或全局地图。
子图局部坐标以首帧 base 为原点。每个发布快照不可变，复制的只是小型 scan 引用表，不复制点数组。
窗口上限 64 帧、单帧最多 2048 点；只保留当前子图。历史由实际消费者按预算保留。
同步单 owner，使用既有 ExecutionChannel 运行；没有内部线程。独立测试 main 在 tests/local_map_component_test.cpp。
