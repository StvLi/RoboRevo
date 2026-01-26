#!/usr/bin/env python3
"""
验证管理器

这个模块负责管理具身数据验证replay的验证过程：
1. 在每次HTTP响应后暂停，等待用户输入
2. 记录验证结果到CSV文件
3. 管理验证状态
"""

import os
import csv
import threading
import time
from datetime import datetime
from typing import Optional, Dict, List

class ValidationManager:
    """
    验证管理器类
    
    管理具身数据验证replay的验证过程，包括：
    - 等待用户输入(y/n)
    - 记录验证结果到CSV文件
    - 管理验证状态
    """
    
    def __init__(self, output_dir: str = "output_data"):
        """
        初始化验证管理器
        
        Args:
            output_dir: 输出目录路径
        """
        self.output_dir = output_dir
        self.lock = threading.Lock()
        self.waiting_for_input = False
        self.current_validation_data: Optional[Dict] = None
        self.validation_queue: List[Dict] = []
        
        # 确保输出目录存在
        os.makedirs(output_dir, exist_ok=True)
        
        # CSV文件路径
        self.csv_file = os.path.join(output_dir, "validation_results.csv")
        
        # 初始化CSV文件（如果不存在）
        self._init_csv_file()
        
        print(f"[验证管理器] 初始化完成")
        print(f"[验证管理器] 输出目录: {output_dir}")
        print(f"[验证管理器] CSV文件: {self.csv_file}")
    
    def _init_csv_file(self):
        """初始化CSV文件，添加表头（如果文件不存在）"""
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(['filename', 'timestamp', 'validation_result', 'request_time'])
            print(f"[验证管理器] 创建新的CSV文件: {self.csv_file}")
    
    def start_validation(self, filename: str, request_time: Optional[str] = None) -> bool:
        """
        开始验证过程
        
        Args:
            filename: 正在验证的文件名
            request_time: 请求时间（可选）
            
        Returns:
            bool: 是否成功开始验证
        """
        with self.lock:
            if self.waiting_for_input:
                print(f"[验证管理器] 警告：已有验证在进行中，将新验证加入队列")
                self.validation_queue.append({
                    'filename': filename,
                    'request_time': request_time or self._get_current_timestamp()
                })
                return False
            
            # 设置验证数据
            self.current_validation_data = {
                'filename': filename,
                'request_time': request_time or self._get_current_timestamp(),
                'validation_time': None,
                'validation_result': None
            }
            self.waiting_for_input = True
            
            print(f"\n" + "=" * 70)
            print(f"[验证管理器] 开始验证")
            print(f"[验证管理器] 文件: {filename}")
            print(f"[验证管理器] 请求时间: {self.current_validation_data['request_time']}")
            print(f"[验证管理器] 请输入验证结果 [Yes(y)/No(n)]: ", end='', flush=True)
            
            return True
    
    def process_user_input(self, user_input: str) -> bool:
        """
        处理用户输入
        
        Args:
            user_input: 用户输入（y/n）
            
        Returns:
            bool: 是否成功处理输入
        """
        with self.lock:
            if not self.waiting_for_input or not self.current_validation_data:
                print(f"[验证管理器] 警告：没有等待验证的请求")
                return False
            
            # 清理用户输入
            user_input = user_input.strip().lower()
            
            # 验证输入
            if user_input not in ['y', 'n', 'yes', 'no']:
                print(f"[验证管理器] 错误：无效输入 '{user_input}'，请输入 y 或 n")
                return False
            
            # 确定验证结果
            validation_result = "YES" if user_input in ['y', 'yes'] else "NO"
            
            # 更新验证数据
            self.current_validation_data['validation_time'] = self._get_current_timestamp()
            self.current_validation_data['validation_result'] = validation_result
            
            # 记录到CSV
            self._record_to_csv(self.current_validation_data)
            
            print(f"[验证管理器] 验证结果: {validation_result}")
            print(f"[验证管理器] 验证时间: {self.current_validation_data['validation_time']}")
            print(f"[验证管理器] 验证完成")
            print("=" * 70 + "\n")
            
            # 重置状态
            self.waiting_for_input = False
            completed_data = self.current_validation_data
            self.current_validation_data = None
            
            # 处理队列中的下一个验证
            if self.validation_queue:
                next_validation = self.validation_queue.pop(0)
                print(f"[验证管理器] 处理队列中的下一个验证: {next_validation['filename']}")
                self.start_validation(next_validation['filename'], next_validation['request_time'])
            
            return True
    
    def _record_to_csv(self, validation_data: Dict):
        """
        记录验证结果到CSV文件
        
        Args:
            validation_data: 验证数据字典
        """
        try:
            with open(self.csv_file, 'a', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    validation_data['filename'],
                    validation_data['validation_time'],
                    validation_data['validation_result'],
                    validation_data['request_time']
                ])
            
            print(f"[验证管理器] 验证结果已记录到CSV文件")
        except Exception as e:
            print(f"[验证管理器] 错误：记录到CSV文件失败: {e}")
    
    def _get_current_timestamp(self) -> str:
        """
        获取当前时间戳，格式为YYYYMMDDHHmmss
        
        Returns:
            str: 当前时间戳
        """
        return datetime.now().strftime("%Y%m%d%H%M%S")
    
    def is_waiting_for_input(self) -> bool:
        """
        检查是否正在等待用户输入
        
        Returns:
            bool: 是否正在等待用户输入
        """
        with self.lock:
            return self.waiting_for_input
    
    def get_current_filename(self) -> Optional[str]:
        """
        获取当前正在验证的文件名
        
        Returns:
            Optional[str]: 文件名，如果没有正在验证的文件则返回None
        """
        with self.lock:
            if self.current_validation_data:
                return self.current_validation_data['filename']
            return None
    
    def get_status(self) -> Dict:
        """
        获取验证管理器状态
        
        Returns:
            Dict: 状态信息
        """
        with self.lock:
            return {
                'waiting_for_input': self.waiting_for_input,
                'current_filename': self.get_current_filename(),
                'queue_length': len(self.validation_queue),
                'csv_file': self.csv_file,
                'output_dir': self.output_dir
            }
    
    def skip_current_validation(self, result: str = "SKIPPED") -> bool:
        """
        跳过当前验证
        
        Args:
            result: 跳过原因
            
        Returns:
            bool: 是否成功跳过
        """
        with self.lock:
            if not self.waiting_for_input or not self.current_validation_data:
                return False
            
            # 更新验证数据
            self.current_validation_data['validation_time'] = self._get_current_timestamp()
            self.current_validation_data['validation_result'] = result
            
            # 记录到CSV
            self._record_to_csv(self.current_validation_data)
            
            print(f"[验证管理器] 验证已跳过: {result}")
            
            # 重置状态
            self.waiting_for_input = False
            self.current_validation_data = None
            
            # 处理队列中的下一个验证
            if self.validation_queue:
                next_validation = self.validation_queue.pop(0)
                print(f"[验证管理器] 处理队列中的下一个验证: {next_validation['filename']}")
                self.start_validation(next_validation['filename'], next_validation['request_time'])
            
            return True