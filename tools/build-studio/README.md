# 构建工作台
从 roboframe 根目录启动静态服务器，打开 tools/build-studio/index.html。
页面只生成命令，不执行构建、下载依赖或检查本机架构。

catalog.js 声明真实组件与产品组合；新增组件无需逐个编写 UI。
planner.js 将 SOURCE/PACKAGE 转成逐组件 CMake 选项，支持混合构建。
全源码使用源码 SDK；包含安装包时使用统一 SDK 安装包。
ARM 仅预览，需要工具链、sysroot 和目标架构开发包后再接入。
所有来源选项显式写出，重用构建目录也不会遗留旧选择。
配置 JSON 仅用于记录，CMake 暂不直接读取。
校验：node tools/build-studio/check.cjs。

将来组件清单稳定后再让 CMake 和页面共享清单；当前八个组件手工登记，
不增加插件发现、依赖下载器或后台服务。

产品目录由 catalog 的 product 字段选择；生成 ROBOFRAME_PRODUCT，并按产品隔离构建目录。
