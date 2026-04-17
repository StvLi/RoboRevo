# virtuose_ros2 踩坑与排错记录

本文记录从 ROS1 迁移与联调过程中实际遇到的问题及原因，便于后续维护与排查。

## 1. `virtOpen` 始终返回 NULL（与 IP/端口格式无关）

**现象**：日志出现 `virtOpen("127.0.0.1#5001") 返回 NULL`，但同机 ROS1 `virtuose_node` 正常。

**根因**：`libvirtuose.a` 并非完整自包含库，内部通过 **`dlopen` 加载 `virtuoseAPI.so`**。若安装包中未携带该 `.so`，且 `LD_LIBRARY_PATH` / `RUNPATH` 也找不到它，则动态库加载失败，`virtOpen` 失败。

**ROS1 为何“开箱能用”**：catkin 生成的 `virtuose_node` 通常带有指向源码树  
`.../virtuose/external/VirtuoseAPI`  
的 **RUNPATH**，运行时可从该目录加载 `virtuoseAPI.so`。

**ROS2 侧修复**（已实现）：

- CMake **`install(FILES ... virtuoseAPI.so DESTINATION lib)`**，随包安装到 `install/<pkg>/lib/`；
- 为 `virtuose_node` 设置 **`INSTALL_RPATH=$ORIGIN/..`**（可执行文件在 `lib/<pkg>/`，上一级即 `lib/`）；
- **`BUILD_RPATH`** 指向构建时的 `VIRTUOSE_SDK_ROOT`，便于在 `build` 目录直接运行未 install 的二进制。

**自检**：

```bash
ls install/virtuose_ros2/lib/virtuoseAPI.so
readelf -d install/virtuose_ros2/lib/virtuose_ros2/virtuose_node | grep RUNPATH
```

## 2. colcon 配置阶段 Python 报错（如 `No module named 'pkg_resources'`）

**现象**：CMake 调用 ament 模板脚本失败。

**根因**：`colcon`/`cmake` 使用了 **conda** 等环境中的 Python，缺少 `setuptools` 提供的 `pkg_resources`。

**处理**：构建时指定系统 Python，例如：

```bash
PYTHON_EXECUTABLE=/usr/bin/python3 colcon build --packages-select virtuose_ros2 \
  --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
```

或在 conda 环境中安装 `setuptools`。

## 3. 测试节点报“服务不可用”，但 `ros2 service list` 能看到服务

**现象**：`virtuose_reset` 等在列表中可见，客户端仍长时间等不到。

**根因之一**：**`ROS_DOMAIN_ID` 不一致**，不同进程不在同一 DDS 域。

**处理**：所有终端在 `source` 后执行相同 `export ROS_DOMAIN_ID=...`（或不导出，统一默认 0）。

**根因之二（已修复写法）**：循环里调用 **`wait_for_service(较长时间)`** 会阻塞当前线程且**不执行** `rclcpp::spin_some`，图与发现更新卡住。

**处理**：在循环中 **`spin_some(node)` + `client->service_is_ready()`**，短间隔 `sleep`（测试节点中已按此实现）。

## 4. `VirtuoseAdmittance` 增益字段与 ROS1 不一致

**现象**：自定义客户端按 ROS1 字段名填参，导纳行为异常或编译失败。

**说明**：ROS 2 中请求字段为 **`ktrans` / `btrans` / `krot` / `brot`**（全小写），对应 ROS1 的 `Ktrans` / `Btrans` / `Krot` / `Brot`。

## 5. CMake 仍指向错误的 SDK 路径

**现象**：换机器或拷贝工作区后，链接/安装的仍是旧路径下的 `.so`。

**根因**：`build/<pkg>/CMakeCache.txt` 中 **`VIRTUOSE_SDK_ROOT` 被缓存**。

**处理**：删除该包 build 缓存或整个 `build/virtuose_ros2` 后重新 `colcon build`；或显式传入 `-DVIRTUOSE_SDK_ROOT=...`。

**说明**：若存在可用的 `haption_ros1_ws/.../external/VirtuoseAPI`，本包 **默认优先**使用该路径作为 `VIRTUOSE_SDK_ROOT`，与已验证的 catkin 环境保持一致。

## 6. `virtSetPeriodicFunction` 的周期参数指针

**说明**：若实现库在周期线程中长期保存传入的 **`float*` 周期**，则不应使用服务回调栈上的局部变量。当前实现使用 **文件作用域静态变量** 保存周期（秒），避免悬空指针。

## 7. 执行器与 Virtuose 调用线程

**说明**：曾对 `virtuose_node` 使用 **`SingleThreadedExecutor`**，使 ROS 侧服务/订阅回调串行化，降低与设备 API 交互时的竞态风险；设备周期回调仍在 Virtuose 内部线程执行。

## 8. 仅拷贝 `libvirtuose.a` 未拷贝 `virtuoseAPI.so`

**现象**：配置或运行阶段才暴露问题。

**处理**：本包 CMake 在 **`virtuoseAPI.so` 缺失时直接 FATAL**，强制 SDK 目录完整；安装规则会把 `.so` 一并安装进工作空间。

## 9. 多实例 / 重复节点名

**现象**：`ros2 node list` 警告多个同名 `/virtuose`。

**处理**：关闭多余的 `virtuose_node` 进程，保证域内只有一个主节点，避免状态与服务混乱。

---

若出现新的问题，建议保留 **`virtuose_node` 终端中带 `[virtuose]` 的日志**（含失败步骤与 `virtGetErrorCode` 对应说明），便于对照 Virtuose API 文档继续分析。
