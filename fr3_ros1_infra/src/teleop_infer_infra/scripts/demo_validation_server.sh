#!/bin/bash
# 验证服务器演示脚本

echo "================================================================"
echo "H5验证HTTP服务器演示"
echo "================================================================"
echo ""
echo "这个演示将展示："
echo "1. 启动H5验证HTTP服务器"
echo "2. 发送测试请求"
echo "3. 验证功能：等待用户输入(y/n)"
echo "4. 检查CSV记录"
echo ""

# 设置变量
SERVER_HOST="127.0.0.1"
SERVER_PORT="5006"
H5_DIR="/home/alan/桌面/UMI_replay_数据/正常"
CSV_FILE="output_data/validation_results.csv"

echo "配置："
echo "  服务器地址: http://${SERVER_HOST}:${SERVER_PORT}"
echo "  H5目录: ${H5_DIR}"
echo "  CSV文件: ${CSV_FILE}"
echo ""

# 检查H5目录
if [ ! -d "$H5_DIR" ]; then
    echo "错误：H5目录不存在: $H5_DIR"
    exit 1
fi

echo "✓ H5目录存在"
echo ""

# 清理之前的CSV文件（可选）
echo "清理之前的测试数据..."
if [ -f "$CSV_FILE" ]; then
    BACKUP_FILE="${CSV_FILE}.backup.$(date +%Y%m%d%H%M%S)"
    cp "$CSV_FILE" "$BACKUP_FILE"
    echo "  已备份CSV文件: $BACKUP_FILE"
fi

echo "创建新的CSV文件..."
echo "filename,timestamp,validation_result,request_time" > "$CSV_FILE"
echo "✓ CSV文件已初始化"
echo ""

# 启动服务器（在后台）
echo "启动H5验证HTTP服务器..."
cd /home/alan/fr3_ros1_infra
python3 src/teleop_infer_infra/scripts/h5_validation_server.py \
    --host "$SERVER_HOST" \
    --port "$SERVER_PORT" \
    --h5-dir "$H5_DIR" \
    --mode sequential \
    --no-diff-postprocess &
    
SERVER_PID=$!
echo "  服务器PID: $SERVER_PID"

# 等待服务器启动
echo "等待服务器启动..."
sleep 3

# 检查服务器是否运行
if ! curl -s "http://${SERVER_HOST}:${SERVER_PORT}/health" > /dev/null; then
    echo "错误：服务器启动失败"
    kill $SERVER_PID 2>/dev/null
    exit 1
fi

echo "✓ 服务器已启动并运行"
echo ""

# 显示服务器信息
echo "服务器信息："
curl -s "http://${SERVER_HOST}:${SERVER_PORT}/info" | python3 -m json.tool
echo ""

# 发送测试请求
echo "发送测试请求 #1..."
REQUEST1_RESPONSE=$(curl -s -X POST "http://${SERVER_HOST}:${SERVER_PORT}/predict_action" \
    -H "Content-Type: application/json" \
    -d '{
        "examples": [
            {
                "image": ["base64_image_data_1", "base64_image_data_2"],
                "lang": "Pick up the banana",
                "state": null
            }
        ]
    }')

echo "请求 #1 响应："
echo "$REQUEST1_RESPONSE" | python3 -m json.tool
echo ""

echo "================================================================"
echo "验证阶段"
echo "================================================================"
echo ""
echo "现在服务器正在等待验证输入..."
echo "请查看服务器终端，输入 'y' 或 'n' 来完成验证。"
echo ""
echo "在另一个终端中，您可以："
echo "1. 查看服务器状态: curl http://${SERVER_HOST}:${SERVER_PORT}/health"
echo "2. 查看验证状态: curl http://${SERVER_HOST}:${SERVER_HOST}:${SERVER_PORT}/validation/status"
echo "3. 跳过验证: curl -X POST http://${SERVER_HOST}:${SERVER_PORT}/validation/skip"
echo ""
echo "等待10秒让您有机会输入验证结果..."
sleep 10

# 检查验证状态
echo "检查验证状态："
VALIDATION_STATUS=$(curl -s "http://${SERVER_HOST}:${SERVER_PORT}/validation/status")
echo "$VALIDATION_STATUS" | python3 -m json.tool
echo ""

# 检查CSV文件
echo "检查CSV文件内容："
if [ -f "$CSV_FILE" ]; then
    echo "CSV文件内容："
    cat "$CSV_FILE"
    echo ""
    
    LINE_COUNT=$(wc -l < "$CSV_FILE")
    echo "CSV文件行数: $LINE_COUNT"
else
    echo "错误：CSV文件不存在"
fi

echo ""

# 发送第二个请求（如果验证已完成）
echo "发送测试请求 #2..."
REQUEST2_RESPONSE=$(curl -s -X POST "http://${SERVER_HOST}:${SERVER_PORT}/predict_action" \
    -H "Content-Type: application/json" \
    -d '{
        "examples": [
            {
                "image": ["base64_image_data_3", "base64_image_data_4"],
                "lang": "Place the banana on the table",
                "state": null
            }
        ]
    }')

if echo "$REQUEST2_RESPONSE" | grep -q "error"; then
    echo "请求 #2 被拒绝（预期行为 - 服务器正在等待验证）："
    echo "$REQUEST2_RESPONSE" | python3 -m json.tool
else
    echo "请求 #2 响应："
    echo "$REQUEST2_RESPONSE" | python3 -m json.tool
fi

echo ""

# 最终状态
echo "最终服务器状态："
curl -s "http://${SERVER_HOST}:${SERVER_PORT}/health" | python3 -m json.tool
echo ""

# 清理
echo "清理：停止服务器..."
kill $SERVER_PID 2>/dev/null
wait $SERVER_PID 2>/dev/null
echo "✓ 服务器已停止"
echo ""

echo "================================================================"
echo "演示完成"
echo "================================================================"
echo ""
echo "总结："
echo "1. H5验证HTTP服务器成功启动"
echo "2. 服务器在每次响应后等待用户验证输入(y/n)"
echo "3. 验证结果自动记录到CSV文件: $CSV_FILE"
echo "4. 在等待验证时，新请求被拒绝（HTTP 503）"
echo ""
echo "要使用完整功能，请运行："
echo "  python3 src/teleop_infer_infra/scripts/h5_validation_server.py \\"
echo "    --host 127.0.0.1 --port 5003 --mode sequential"
echo ""
echo "CSV文件格式："
echo "  filename,timestamp,validation_result,request_time"
echo "  示例: banana1.h5,20260126160420,YES,20260126160415"