# distance_goal：周期里程目标

UpdateDistance 接收非负、有限、单调不减的累计米数；Activate 将它清零。
Goal 的 meters 是本次激活以来的绝对阈值，timeout 从 worker start 起算，默认 2 秒。
组件持有 SDK Server，以 10ms 周期检查阈值并输出 Progress；达到目标成功，超时失败。
不是速度控制器，不驱动实机。它不认识路径组件、odom、integration 或 BT。

生命周期只有一个 owner；UpdateDistance 和 server() 的目标/状态/取消端口支持并发。
进度回调在 Server worker 执行，必须短、不阻塞、不重入生命周期，接收者需存活到组件停用。
finish 尝试输出 Progress{}（inactive，数值清零），包括 start 前取消、非法目标和 step 异常。
若输出回调本身抛异常，Server 返回 failed；不能声称发送成功。
RequestDeactivate 关闭输入并请求取消，Deactivate 等待收尾；取消是协作式。
Activate 后可执行新目标；server() 仅供操作目标，不让宿主管理内部执行器生命周期。

公开头：include/distance_goal/distance_goal_component.hpp；链接 DistanceGoal::component，只依赖 SDK。

~~~cpp
distance_goal::DistanceGoalComponent goal([](distance_goal::Progress p) { /* 短回调入队 */ });
goal.Activate();
auto handle = goal.server().Submit({5, std::chrono::seconds(1)});
goal.UpdateDistance(5);
// 查询 GetStatus / TakeResult，或 RequestCancel(handle) 后等待结果。
// 完整直接测试见 tests/distance_goal_component_test.cpp；不需要 Integration 或 Poll。
goal.Deactivate();
~~~

日常开发在仓库根目录，固定 build-dev：

~~~sh
cmake -S . -B build-dev -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON -DROBOFRAME_SDK_SOURCE=SOURCE -DROBOFRAME_PRODUCT= -DROBOFRAME_COMPONENT=distance_goal
cmake --build build-dev -j4
ctest --test-dir build-dev --output-on-failure
~~~

可使用已安装 SDK 独立交付，不需要产品、interface 或其他组件：

~~~sh
cmake -S components/distance_goal -B /tmp/distance_goal-build -DCMAKE_BUILD_TYPE=Debug -DCMAKE_PREFIX_PATH=/path/to/sdk-prefix
cmake --build /tmp/distance_goal-build -j4
ctest --test-dir /tmp/distance_goal-build --output-on-failure
cmake --install /tmp/distance_goal-build --prefix /path/to/component-prefix
~~~

实际组件独立安装、搬迁消费者及产品包模式验证由
products/odom_session/tests/delivery_test.py 在临时目录完成；不替代日常 clangd 索引。
本组件没有额外的外部接入 main；组件测试就是普通宿主的直接调用示例。
