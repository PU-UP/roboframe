# OdomBuffer

C++14 / SDK >=1.1、相同 major；公开 target 为 OdomBuffer::component。
只依赖 SDK，不依赖 integration、通讯或其他组件。

输入是外部算法/传感器提供的 TimedPose2D：采集微秒时间、米、弧度、T_odom_base。
它不是新的里程计估计器；当前用于真实录制 ODOM 的校验、有界历史与查询。

- 构造 Config：capacity 默认 512，max_gap_us 默认 250000。
- Activate 后 PushOdometry；同一实例激活期间时间必须严格递增。
- At(t, pose) 返回 ready / waiting / unavailable；仅 ready 才写入输出。
- 两侧时间间隔不超过上限时线性插值位置、最短圆弧插值 yaw；不外推、不取最新代替。
- Deactivate 清空缓存。单线程调用，无后台任务或回调。

~~~sh
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/sdk
cmake --build build
ctest --test-dir build --output-on-failure
cmake --install build --prefix /path/to/odom-package
~~~

tests/check.cpp 本身就是直接连接 component 的 main，可作为算法作者自测入口。
配置由 integration 选择，组件只校验自身参数，不判断产品是否处于建图阶段。
