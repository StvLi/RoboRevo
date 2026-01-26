# FR3 ROS1 Infrastructure 文档

本目录包含FR3 ROS1系统的架构文档和使用说明。

## 文档列表

### [Cartesian Impedance Controller 架构文档](./cartesian_impedance_controller_architecture.md)

详细描述了 `cartesian_impedance_example_controller.launch` 启动后的完整系统架构，包括：

- **节点列表**: 所有运行节点的功能说明
- **话题关系**: 完整的发布-订阅关系图
- **服务关系**: 所有服务的服务器-客户端关系
- **Action关系**: Action服务器和客户端的关系
- **数据流图**: 控制数据流和状态反馈数据流
- **使用建议**: 如何启动、控制和监控系统

**适用场景**:
- 理解系统架构
- 开发新的控制器
- 调试系统问题
- 集成其他ROS节点

**生成时间**: 2026-01-22

---

## 如何查看文档

文档使用Markdown格式编写，可以使用以下方式查看：

1. **在IDE中直接打开**: 大多数现代IDE都支持Markdown预览
2. **使用命令行工具**: 
   ```bash
   cat docs/cartesian_impedance_controller_architecture.md
   ```
3. **使用Markdown查看器**: 
   ```bash
   # 如果安装了grip
   grip docs/cartesian_impedance_controller_architecture.md
   ```

---

## 更新文档

当系统配置发生变化时，需要重新生成文档：

1. 启动相应的launch文件
2. 使用 `rostopic list`, `rosservice list`, `rosnode info` 等命令收集信息
3. 更新对应的Markdown文档

---

**最后更新**: 2026-01-22



