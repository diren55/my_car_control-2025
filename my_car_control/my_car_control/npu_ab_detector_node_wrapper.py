#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-
"""
NPU AB检测节点包装器
使用Python 3.9运行，以确保NPU库兼容性
"""

import sys
import os

# 设置昇腾NPU库路径
ascend_lib_paths = [
    '/usr/local/Ascend/ascend-toolkit/7.0.RC1/aarch64-linux/devlib',
    '/usr/local/Ascend/ascend-toolkit/7.0.RC1/atc/lib64',
    '/usr/local/Ascend/driver/lib64',
    '/usr/local/Ascend/add-ons',
]

current_ld_path = os.environ.get('LD_LIBRARY_PATH', '')
new_paths = [p for p in ascend_lib_paths if os.path.exists(p)]
if new_paths:
    new_ld_path = ':'.join(new_paths)
    if current_ld_path:
        os.environ['LD_LIBRARY_PATH'] = new_ld_path + ':' + current_ld_path
    else:
        os.environ['LD_LIBRARY_PATH'] = new_ld_path

# 导入实际节点代码
from npu_ab_detector_node import main

if __name__ == '__main__':
    main()

