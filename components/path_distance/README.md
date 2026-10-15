# path_distance：异步累计路程

输入为 TimedPose2D（米、弧度、微秒）；按接纳顺序累计相邻位姿二维距离，忽略 yaw，不重排时间。
输出 Sample 包含样本数、最新位姿、累计米数。非法位姿在 worker 校验，Take 返回 exception_ptr；
该样本不改变累计状态。组件不认识 odom、产品或 BT。

组件持有 ExecutionChannel，默认容量 32，包含排队、计算和已完成未领取的结果。
full 必须由宿主保留同一输入并重试；不可丢帧，否则会改变路径长度。
全部公开端口由同一个控制 owner 调用，算法在一个内部 worker 串行运行。
Activate 清零；RequestDeactivate 关闭输入并请求取消；Deactivate 等待并丢弃旧结果。
重新 Activate 前必须完成 Deactivate。停止不承诺保留尚未领取的输出；正常结束先 Take 完所有输入。
析构 join worker，回调/内部状态不会跨激活周期使用。

公开头：include/path_distance/path_distance_component.hpp；链接 PathDistance::component，只依赖 SDK。

~~~cpp
path_distance::PathDistanceComponent distance;
distance.Activate();
distance.Push({1, {0, 0, 0}});
distance.Push({2, {3, 4, 0}});
// 在自己的循环中调用 Take；第二份成功输出的 meters 为 5。
// 完整的有截止时间等待与错误检查见 tests/path_distance_component_test.cpp。
distance.Deactivate();
~~~

日常开发在仓库根目录，固定 build-dev：

~~~sh
cmake -S . -B build-dev -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON -DROBOFRAME_SDK_SOURCE=SOURCE -DROBOFRAME_PRODUCT= -DROBOFRAME_COMPONENT=path_distance
cmake --build build-dev -j4
ctest --test-dir build-dev --output-on-failure
~~~

可使用已安装 SDK 独立交付，不需要产品、interface 或其他组件：

~~~sh
cmake -S components/path_distance -B /tmp/path_distance-build -DCMAKE_BUILD_TYPE=Debug -DCMAKE_PREFIX_PATH=/path/to/sdk-prefix
cmake --build /tmp/path_distance-build -j4
ctest --test-dir /tmp/path_distance-build --output-on-failure
cmake --install /tmp/path_distance-build --prefix /path/to/component-prefix
~~~

实际组件独立安装、搬迁消费者及产品包模式验证由
products/odom_session/tests/delivery_test.py 在临时目录完成；不替代日常 clangd 索引。
本组件没有额外的外部接入 main；组件测试就是普通宿主的直接调用示例。
