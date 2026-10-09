# MapStore

C++14 / SDK >=1.1、相同 major；公开 target 为 MapStore::component。
只依赖 SDK 与 Linux/POSIX 文件 API。输出目录由 integration/宿主配置，必须预先存在。

Activate → Save(name, GridSnapshot) / Load(name) → Deactivate。
name 只接受最多 80 个字母、数字、下划线和连字符；实际文件为 name.rfgrid。
Save 采用同目录临时文件，完成写入并 fsync 文件之后，以硬链接原子发布。
已有文件不会覆盖，失败清理临时文件；integration 内的地图不受保存失败影响。
当前要求支持硬链接的本地文件系统，不承诺掉电持久性（没有目录 fsync）。

RFGRID1 文件格式：
1. 第一行固定 RFGRID1。
2. 第二行为 width height resolution origin_x origin_y revision stamp_us。
3. 第三行为 metadata 原始字节与全部 cells 拼接后的 FNV-1a 64 位十进制校验值。
4. 后续恰好 width*height 字节，y 从下向上，0=occupied、254=free、205=unknown。

Load 校验格式、数值/尺寸上限、载荷长度、合法栅格和完整性，返回只读快照。
地图内容无本机指针或二进制 C++ 对象布局。校验值不是身份认证或加密签名。
组件只管理地图文件，不决定何时结束建图，也不负责产品地图选择流程。

~~~sh
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/sdk
cmake --build build
ctest --test-dir build --output-on-failure
cmake --install build --prefix /path/to/store-package
~~~

tests/check.cpp 是独立 main，覆盖读回一致、不覆盖、路径拒绝、损坏和尺寸校验。
真实产品若需要掉电保证、多进程目录管理或 Windows，单独扩展这个组件即可。
