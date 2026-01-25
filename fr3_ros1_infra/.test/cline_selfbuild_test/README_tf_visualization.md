# TF可视化脚本使用说明

## 脚本功能
从h5文件读取位姿数据并通过ROS TF发布，实现坐标可视化。

## 要求满足情况
1. ✅ **30帧/s**：默认发布频率为30Hz，可通过`--rate`参数调整
2. ✅ **tf关系父系为fr3_link0，子系为umi_target**：默认TF关系为`fr3_link0` → `umi_target`，可通过`--parent`和`--child`参数调整
3. ✅ **直接获得h5文件中actions的位姿不需要diff处理**：直接从h5文件读取原始位姿数据（位置+四元数），不进行差分处理
4. ✅ **只需要将四元数转为tf**：将四元数直接用于TF变换发布
5. ✅ **循环播放**：默认循环播放，可通过`--no-loop`参数禁用
6. ✅ **不需要mock等功能，最小实现**：脚本仅包含核心功能，无额外mock功能

## 使用方法

### 基本使用
```bash
# 激活ROS环境后运行
cd /home/alan/fr3_ros1_infra
source devel/setup.bash
python .test/cline_selfbuild_test/tf_visualization.py
```

### 命令行参数
```bash
# 使用默认参数（banana1.h5，30Hz，循环播放）
python tf_visualization.py

# 指定h5文件
python tf_visualization.py --file /path/to/your/file.h5

# 调整发布频率（例如10Hz）
python tf_visualization.py --rate 10

# 不循环播放
python tf_visualization.py --no-loop

# 自定义TF坐标系
python tf_visualization.py --parent base_link --child target_frame

# 组合使用
python tf_visualization.py --file /home/alan/桌面/UMI_replay_数据/正常/banana2.h5 --rate 20 --no-loop
```

### 参数说明
- `--file`: h5文件路径（默认：`/home/alan/桌面/UMI_replay_数据/正常/banana1.h5`）
- `--rate`: 发布频率，单位Hz（默认：30）
- `--no-loop`: 禁用循环播放（默认：启用循环播放）
- `--parent`: 父坐标系（默认：`fr3_link0`）
- `--child`: 子坐标系（默认：`umi_target`）

## 数据格式要求
h5文件应包含`action`数据集，形状为`(n, 8)`，其中：
- 第0-2列：位置 (x, y, z)
- 第3-6列：四元数 (x, y, z, w)
- 第7列：夹爪状态（可选，脚本会忽略）

如果数据维度不同，脚本会自动处理：
- 7维数据：补零到8维
- 大于8维：取前8维
- 小于7维：报错退出

## 验证方法

### 1. 检查TF变换
在RViz中：
1. 添加TF显示
2. 查看`fr3_link0`到`umi_target`的变换
3. 确认变换随时间更新（30Hz）

### 2. 命令行测试
```bash
# 测试数据读取
cd .test/cline_selfbuild_test
python3 -c "import tf_visualization as tfv; poses = tfv.read_from_file('/home/alan/桌面/UMI_replay_数据/正常/banana1.h5'); print(f'读取到{len(poses)}个位姿')"

# 查看帮助
python tf_visualization.py --help
```

### 3. 查看日志
脚本运行时输出日志包括：
- 读取的位姿数量
- 发布频率
- 循环播放状态
- TF坐标系关系
- 每100帧的调试信息

## 脚本结构
```
tf_visualization.py
├── read_from_file()          # 读取h5文件
├── TFVisualizationNode类     # 主节点类
│   ├── __init__()           # 初始化
│   ├── publish_tf()         # 发布TF变换
│   └── run()                # 主循环
└── main()                   # 命令行入口
```

## 注意事项
1. 需要ROS环境（roscore运行中）
2. 需要安装h5py、numpy、rospy等依赖
3. 脚本会自动归一化四元数
4. 如果位姿数据无效，会使用单位四元数
5. 按Ctrl+C停止脚本

## 示例h5文件
默认使用：`/home/alan/桌面/UMI_replay_数据/正常/banana1.h5`
该文件包含385个位姿，每个位姿8维数据。