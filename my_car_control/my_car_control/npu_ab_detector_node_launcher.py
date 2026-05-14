#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NPU AB检测节点启动器
自动检测Python版本并使用兼容的Python解释器
"""

import sys
import os
import subprocess

def find_python39():
    """查找Python 3.9解释器"""
    candidates = [
        '/usr/local/miniconda3/bin/python3.9',
        '/usr/bin/python3.9',
        'python3.9',
    ]
    for python_cmd in candidates:
        try:
            result = subprocess.run(
                [python_cmd, '--version'],
                capture_output=True,
                timeout=2
            )
            if result.returncode == 0 and '3.9' in result.stdout.decode():
                return python_cmd
        except:
            continue
    return None

def test_npu_import(python_cmd):
    """测试NPU库是否可以导入"""
    try:
        test_script = """
import sys
import os
sys.path.insert(0, '/home/davinci-mini/infer_project')
os.environ['LD_LIBRARY_PATH'] = '/usr/local/Ascend/ascend-toolkit/7.0.RC1/aarch64-linux/devlib:/usr/local/Ascend/ascend-toolkit/7.0.RC1/atc/lib64:' + os.environ.get('LD_LIBRARY_PATH', '')
try:
    from ais_bench.infer.interface import InferSession
    sys.exit(0)
except Exception as e:
    print(f'NPU import failed: {e}', file=sys.stderr)
    sys.exit(1)
"""
        result = subprocess.run(
            [python_cmd, '-c', test_script],
            capture_output=True,
            timeout=5,
            env=dict(os.environ, LD_LIBRARY_PATH='/usr/local/Ascend/ascend-toolkit/7.0.RC1/aarch64-linux/devlib:/usr/local/Ascend/ascend-toolkit/7.0.RC1/atc/lib64:' + os.environ.get('LD_LIBRARY_PATH', ''))
        )
        return result.returncode == 0
    except:
        return False

def main():
    """主函数：选择正确的Python版本运行节点"""
    current_python = sys.executable
    current_version = sys.version_info
    
    # 设置LD_LIBRARY_PATH（必须在导入NPU库之前）
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
    
    # 如果当前是Python 3.9，直接运行
    if current_version.major == 3 and current_version.minor == 9:
        from npu_ab_detector_node import main as npu_main
        npu_main()
        return
    
    # 否则，查找并使用Python 3.9
    python39 = find_python39()
    if python39:
        if test_npu_import(python39):
            # 获取节点脚本路径
            node_script = os.path.join(os.path.dirname(__file__), 'npu_ab_detector_node.py')
            # 使用Python 3.9运行
            os.execv(python39, [python39, node_script] + sys.argv[1:])
        else:
            print("⚠️  Python 3.9无法导入NPU库，尝试当前Python...", file=sys.stderr)
    else:
        print("⚠️  未找到Python 3.9，尝试当前Python...", file=sys.stderr)
    
    # 使用当前Python尝试运行
    try:
        from npu_ab_detector_node import main as npu_main
        npu_main()
    except ImportError as e:
        if 'aclruntime' in str(e) or 'ais_bench' in str(e):
            print("❌ NPU库导入失败！", file=sys.stderr)
            print("当前Python版本不兼容NPU库（需要Python 3.9）", file=sys.stderr)
            print("解决方案:", file=sys.stderr)
            print("  1. 在root环境下运行（已配置Python 3.9）", file=sys.stderr)
            print("  2. 或安装Python 3.9并配置NPU库", file=sys.stderr)
            sys.exit(1)
        else:
            raise

if __name__ == '__main__':
    main()

