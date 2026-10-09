# 可复制、可运行的开发模板

component/ 是独立组件目录：具体配置、输入输出、实现和 main 都在这里。
integration/ 是产品装配示例：配置选择、接线、启停在这里，不反向进入组件。
模板依赖已安装的 RoboframeSDK 1.x；不依赖 roboframe 应用。

~~~sh
# 在这个 templates 目录运行，替换 prefix 为自己的 SDK 安装位置。
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/sdk-prefix
cmake --build build
ctest --test-dir build --output-on-failure

# 算法作者也可只构建 component/，不接触 integration/。
cmake -S component -B build-component -DCMAKE_PREFIX_PATH=/path/to/sdk-prefix
cmake --build build-component
./build-component/value_scale_demo
~~~

复制整个目录作为起点，或仅复制 component/；替换 value_scale / ValueScale 名称及业务类型。
不要修改 SDK 安装目录里的模板作为正式业务代码。模板不参与默认主框架编译，
隔离检查会验证安装后的模板能独立构建。这里没有代码生成器、注册表或通用数据容器。

Config 是构造时校验、实例存续期间不变的算法参数。Input/Output 是模块自己的契约。
这里只做连续缩放，没有 Goal、Cancel 或 Task；需要任务的模块参考
components/sample-accumulator。不要把产品工作模式或 IPC 参数解析放进组件。

公开接口继承 SDK 的 Component，仅统一 Activate / Deactivate / lifecycle。
产品 integration 的端口由其职责决定，模板采用组合；没有强制 Integration 基类。
当一个 integration 的装配、数据路由、任务编排变复杂时，再参考主框架 integration/
按实现文件拆分。目前没有必要先创建三套管理器。

模板 CMake 仅提供源码链接。若需导出自己的安装包，可参考正式 sample-accumulator
的 install/export/config 配置；不要为一个尚未发布的模板再复制整套包发布文件。
