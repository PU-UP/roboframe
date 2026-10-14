# 二维坐标契约

T_parent_child 将 child 坐标转换到 parent。长度米、角度弧度、时间为采集时钟微秒。
当前只使用 base_link、odom、map；没有预设未来 global 层。

- SDK TimedPose2D：T_odom_base。
- SLAM 输出 MapPose2D：T_map_base，同 scan 采集时间。
- PoseGraph：T_map_odom = T_map_base × inverse(T_odom_base)。
- 当前全局 pose：T_map_odom × 最新 T_odom_base；修正必须来自过去且未过期。
- 子图首帧为 local 原点；LocalMapBuilder 计算 T_local_base。
- PoseNode2D 保存锚点的 T_odom_node 和 T_map_node，MapUpdater 使用 T_map_node × T_local_base 放置每帧射线。

SDK submap2d.hpp 是组件之间使用的简单数据，不包含算法。节点和子图 ID 只在 generation 内有效，节点 revision 与栅格 revision 各有语义。
子图只能追加；已发布前缀不能换 scan 或修改局部 pose。历史节点更新必须明确增加版本，不把历史更新当当前定位。

SensorFlow 对齐原始 odom 与 scan；SLAM 仅在参考匹配时输出修正后的 pose，PoseGraph 才推导变换。
当前 PG 是录制修正传播与节点记录实现，不是融合优化器。未来不同后端可替换或组合，算法职责留 component。

输入 Scan2D 已是 base_link 中的有限命中点，射线起点为 base 原点；外部回放负责数据集外参处理。
没有返回的束不伪造为障碍。非零射线原点/逐束时间/去畸变需要扩展契约再接入，不能仅旋转端点冒充标定完整。

对外 TF 查询表示最近匹配 scan 的变换；TF 事件也可以携带 odom 更新后传播的当前变换，消费者必须检查时间戳和有效位。
加载栅格不会建立有效定位。数据来源和匹配算法见 data/intel-lab/README.md；运行链路见 mapping.md。
