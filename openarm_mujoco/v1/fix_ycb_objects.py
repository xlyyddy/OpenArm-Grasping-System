#!/usr/bin/env python3
"""
修复 MuJoCo XML 中 YCB 物体的配置
根据 object2urdf 工具和 YCB 数据集规范进行修复
"""

import os
import re
from pathlib import Path

def check_mesh_files(mesh_dir):
    """检查 mesh 文件是否存在"""
    apple_visual = mesh_dir / "apple" / "textured.obj"
    apple_collision = mesh_dir / "apple" / "textured_vhacd.obj"
    banana_visual = mesh_dir / "banana" / "textured.obj"
    banana_collision = mesh_dir / "banana" / "textured_vhacd.obj"
    
    results = {
        'apple_visual': apple_visual.exists(),
        'apple_collision': apple_collision.exists(),
        'banana_visual': banana_visual.exists(),
        'banana_collision': banana_collision.exists()
    }
    
    return results

def fix_xml_scale(xml_file, scale=10.0):
    """
    修复 XML 文件中的 mesh scale
    根据 object2urdf 的转换，YCB 原始模型单位是 0.1m，需要 scale=10 转换为米
    """
    with open(xml_file, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 替换苹果和香蕉的 mesh scale
    # 查找所有 mesh 标签中的 scale 属性
    patterns = [
        (r'(<mesh name="apple_visual"[^>]*scale=")1 1 1(")', f'\\g<1>{scale} {scale} {scale}\\g<2>'),
        (r'(<mesh name="apple_collision"[^>]*scale=")1 1 1(")', f'\\g<1>{scale} {scale} {scale}\\g<2>'),
        (r'(<mesh name="banana_visual"[^>]*scale=")1 1 1(")', f'\\g<1>{scale} {scale} {scale}\\g<2>'),
        (r'(<mesh name="banana_collision"[^>]*scale=")1 1 1(")', f'\\g<1>{scale} {scale} {scale}\\g<2>'),
    ]
    
    modified = False
    for pattern, replacement in patterns:
        if re.search(pattern, content):
            content = re.sub(pattern, replacement, content)
            modified = True
            print(f"✓ 已修复: {pattern}")
    
    if modified:
        # 备份原文件
        backup_file = xml_file.with_suffix('.xml.bak')
        with open(backup_file, 'w', encoding='utf-8') as f:
            with open(xml_file, 'r', encoding='utf-8') as orig:
                f.write(orig.read())
        print(f"✓ 已创建备份: {backup_file}")
        
        # 写入修改后的内容
        with open(xml_file, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"✓ 已更新: {xml_file}")
        return True
    else:
        print("没有需要修复的内容")
        return False

def main():
    """主函数"""
    script_dir = Path(__file__).parent
    mesh_dir = script_dir / "meshes" / "environment"
    xml_file = script_dir / "openarm_bimanual.xml"
    
    print("=" * 60)
    print("YCB 物体配置检查和修复工具")
    print("=" * 60)
    
    # 检查 mesh 文件
    print("\n1. 检查 mesh 文件...")
    mesh_status = check_mesh_files(mesh_dir)
    
    for name, exists in mesh_status.items():
        status = "✓ 存在" if exists else "✗ 缺失"
        print(f"   {name}: {status}")
    
    if not all(mesh_status.values()):
        print("\n警告: 部分 mesh 文件缺失!")
        print("请确保以下文件存在:")
        print(f"  - {mesh_dir / 'apple' / 'textured.obj'}")
        print(f"  - {mesh_dir / 'apple' / 'textured_vhacd.obj'}")
        print(f"  - {mesh_dir / 'banana' / 'textured.obj'}")
        print(f"  - {mesh_dir / 'banana' / 'textured_vhacd.obj'}")
        print("\n提示: 可以使用 convert_ycb_to_mujoco.py 脚本从 YCB 数据集转换")
    
    # 检查并修复 scale
    print("\n2. 检查 mesh scale 配置...")
    print("   根据 object2urdf 和 YCB 数据集规范:")
    print("   - YCB 原始模型单位是 0.1m")
    print("   - 需要 scale='10 10 10' 转换为米单位")
    print("   - 如果 mesh 文件已经预处理为米单位，则使用 scale='1 1 1'")
    
    # 读取当前配置
    with open(xml_file, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 检查当前的 scale 值
    scale_pattern = r'<mesh name="(apple|banana)_(visual|collision)"[^>]*scale="([^"]+)"'
    matches = re.findall(scale_pattern, content)
    
    if matches:
        print("\n   当前配置:")
        for obj, geom_type, scale in matches:
            print(f"   {obj}_{geom_type}: scale={scale}")
        
        # 如果都是 1 1 1，建议修复
        if all(scale == "1 1 1" for _, _, scale in matches):
            print("\n   检测到 scale='1 1 1'，这可能导致物体太小")
            print("   建议: 如果物体显示太小，请将 scale 改为 '10 10 10'")
            
            response = input("\n   是否将 scale 修改为 '10 10 10'? (y/n): ")
            if response.lower() == 'y':
                fix_xml_scale(xml_file, scale=10.0)
            else:
                print("   保持当前配置")
        elif all(scale == "10 10 10" for _, _, scale in matches):
            print("\n   ✓ scale 配置正确 (10 10 10)")
        else:
            print("\n   警告: scale 配置不一致，请手动检查")
    
    print("\n" + "=" * 60)
    print("完成!")
    print("=" * 60)

if __name__ == "__main__":
    main()

