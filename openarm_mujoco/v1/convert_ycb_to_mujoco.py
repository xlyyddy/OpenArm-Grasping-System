#!/usr/bin/env python3
"""
将 YCB 数据集的物体转换为 MuJoCo XML 格式
根据 object2urdf 工具和 YCB 数据集规范进行转换
支持从 URDF/Xacro 文件转换到 MuJoCo XML 格式
"""

import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import re
import math

def convert_ycb_to_mujoco_xml(obj_dir, object_name, output_mesh_dir, scale=1.0):
    """
    将 YCB 物体的 .obj 文件转换为 MuJoCo XML 片段
    
    Args:
        obj_dir: YCB 物体目录（包含 google_16k 子目录）
        object_name: 物体名称（如 'apple', 'banana'）
        output_mesh_dir: 输出 mesh 文件的目录
        scale: 缩放因子（YCB 原始单位是 0.1m，需要 scale=10 转换为米）
    
    Returns:
        dict: 包含 mesh 定义和 body 定义的 XML 字符串
    """
    # 查找 mesh 文件
    google_16k_dir = os.path.join(obj_dir, "google_16k")
    if not os.path.exists(google_16k_dir):
        # 尝试直接在当前目录查找
        google_16k_dir = obj_dir
    
    visual_obj = None
    collision_obj = None
    
    # 查找 visual 和 collision mesh
    for root, dirs, files in os.walk(google_16k_dir):
        for file in files:
            if file == "textured.obj":
                visual_obj = os.path.join(root, file)
            elif file == "textured_vhacd.obj":
                collision_obj = os.path.join(root, file)
    
    if not visual_obj:
        # 尝试查找其他可能的文件
        for root, dirs, files in os.walk(google_16k_dir):
            for file in files:
                if file.endswith(".obj") and "textured" in file and "vhacd" not in file:
                    visual_obj = os.path.join(root, file)
                    break
            if visual_obj:
                break
    
    if not collision_obj:
        # 如果没有 vhacd 文件，使用 visual 文件作为 collision
        collision_obj = visual_obj
    
    if not visual_obj:
        raise FileNotFoundError(f"找不到 {object_name} 的 mesh 文件")
    
    # 复制 mesh 文件到输出目录
    output_obj_dir = os.path.join(output_mesh_dir, object_name)
    os.makedirs(output_obj_dir, exist_ok=True)
    
    import shutil
    visual_dest = os.path.join(output_obj_dir, "textured.obj")
    collision_dest = os.path.join(output_obj_dir, "textured_vhacd.obj")
    
    shutil.copy2(visual_obj, visual_dest)
    if collision_obj != visual_obj:
        shutil.copy2(collision_obj, collision_dest)
    else:
        # 如果没有单独的 collision 文件，复制 visual 文件
        shutil.copy2(visual_obj, collision_dest)
    
    # 复制相关的 .mtl 和纹理文件
    for root, dirs, files in os.walk(google_16k_dir):
        for file in files:
            if file.endswith(('.mtl', '.png', '.jpg', '.jpeg')):
                src_file = os.path.join(root, file)
                dst_file = os.path.join(output_obj_dir, file)
                shutil.copy2(src_file, dst_file)
    
    # 生成 MuJoCo XML 片段
    mesh_relative_path = f"environment/{object_name}/textured.obj"
    collision_relative_path = f"environment/{object_name}/textured_vhacd.obj"
    
    # Mesh 定义
    mesh_xml = f"""    <mesh name="{object_name}_visual" file="{mesh_relative_path}" scale="{scale} {scale} {scale}" />
    <mesh name="{object_name}_collision" file="{collision_relative_path}" scale="{scale} {scale} {scale}" />"""
    
    return {
        'mesh_xml': mesh_xml,
        'visual_mesh': mesh_relative_path,
        'collision_mesh': collision_relative_path,
        'scale': scale
    }


def update_mujoco_xml(xml_file, ycb_objects_info):
    """
    更新 MuJoCo XML 文件，添加 YCB 物体的 mesh 和 body 定义
    
    Args:
        xml_file: MuJoCo XML 文件路径
        ycb_objects_info: 物体信息列表，每个元素包含 name, pos, quat, mass 等
    """
    tree = ET.parse(xml_file)
    root = tree.getroot()
    
    # 找到 asset 标签
    asset = root.find('asset')
    if asset is None:
        asset = ET.SubElement(root, 'asset')
    
    # 找到 worldbody 标签
    worldbody = root.find('worldbody')
    if worldbody is None:
        worldbody = ET.SubElement(root, 'worldbody')
    
    # 添加 mesh 定义（在 asset 标签的末尾，但在 material 之后）
    # 找到最后一个 material 或 mesh
    last_mesh_idx = -1
    for i, elem in enumerate(asset):
        if elem.tag in ['mesh', 'material']:
            last_mesh_idx = i
    
    # 插入 mesh 定义
    for obj_info in ycb_objects_info:
        mesh_elem_visual = ET.Element('mesh')
        mesh_elem_visual.set('name', f"{obj_info['name']}_visual")
        mesh_elem_visual.set('file', obj_info['visual_mesh'])
        mesh_elem_visual.set('scale', f"{obj_info['scale']} {obj_info['scale']} {obj_info['scale']}")
        
        mesh_elem_collision = ET.Element('mesh')
        mesh_elem_collision.set('name', f"{obj_info['name']}_collision")
        mesh_elem_collision.set('file', obj_info['collision_mesh'])
        mesh_elem_collision.set('scale', f"{obj_info['scale']} {obj_info['scale']} {obj_info['scale']}")
        
        asset.insert(last_mesh_idx + 1, mesh_elem_visual)
        asset.insert(last_mesh_idx + 2, mesh_elem_collision)
        last_mesh_idx += 2
    
    # 添加 body 定义（在 worldbody 中）
    for obj_info in ycb_objects_info:
        body = ET.SubElement(worldbody, 'body')
        body.set('name', obj_info['name'])
        body.set('pos', f"{obj_info['pos'][0]} {obj_info['pos'][1]} {obj_info['pos'][2]}")
        body.set('quat', f"{obj_info['quat'][0]} {obj_info['quat'][1]} {obj_info['quat'][2]} {obj_info['quat'][3]}")
        
        # Inertial
        inertial = ET.SubElement(body, 'inertial')
        inertial.set('pos', '0 0 0')
        inertial.set('quat', '1 0 0 0')
        inertial.set('mass', str(obj_info['mass']))
        inertial.set('diaginertia', f"{obj_info['inertia']} {obj_info['inertia']} {obj_info['inertia']}")
        
        # Visual geom
        geom_visual = ET.SubElement(body, 'geom')
        geom_visual.set('name', f"{obj_info['name']}_visual")
        geom_visual.set('type', 'mesh')
        geom_visual.set('mesh', f"{obj_info['name']}_visual")
        geom_visual.set('pos', f"{obj_info['mesh_offset'][0]} {obj_info['mesh_offset'][1]} {obj_info['mesh_offset'][2]}")
        geom_visual.set('quat', '1 0 0 0')
        geom_visual.set('contype', '0')
        geom_visual.set('conaffinity', '0')
        geom_visual.set('group', '0')
        
        # Collision geom
        geom_collision = ET.SubElement(body, 'geom')
        geom_collision.set('name', f"{obj_info['name']}_collision")
        geom_collision.set('type', 'mesh')
        geom_collision.set('mesh', f"{obj_info['name']}_collision")
        geom_collision.set('pos', f"{obj_info['mesh_offset'][0]} {obj_info['mesh_offset'][1]} {obj_info['mesh_offset'][2]}")
        geom_collision.set('quat', '1 0 0 0')
        geom_collision.set('contype', '1')
        geom_collision.set('conaffinity', '1')
        geom_collision.set('group', '3')
    
    # 保存文件
    tree.write(xml_file, encoding='utf-8', xml_declaration=True)
    print(f"✓ 已更新 {xml_file}")


def parse_urdf_link(urdf_xml, link_name, base_mesh_dir):
    """
    从 URDF XML 中解析指定 link 的信息并转换为 MuJoCo 格式
    
    Args:
        urdf_xml: 解析后的 URDF XML ElementTree
        link_name: 要解析的 link 名称（如 'apple_link', 'banana_link'）
        base_mesh_dir: mesh 文件的基础目录（用于解析 package:// 路径）
    
    Returns:
        dict: 包含 mesh、位置、质量等信息的字典，格式与 convert_ycb_to_mujoco_xml 返回的相同
    """
    # 查找 link
    link = urdf_xml.find(f".//link[@name='{link_name}']")
    if link is None:
        raise ValueError(f"找不到 link: {link_name}")
    
    # 提取 visual mesh
    visual_mesh = None
    visual_scale = [1.0, 1.0, 1.0]
    visual_origin = {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}
    
    visual = link.find('visual')
    if visual is not None:
        origin = visual.find('origin')
        if origin is not None:
            xyz = origin.get('xyz', '0 0 0').split()
            rpy = origin.get('rpy', '0 0 0').split()
            visual_origin = {
                'xyz': [float(x) for x in xyz],
                'rpy': [float(r) for r in rpy]
            }
        
        geometry = visual.find('geometry')
        if geometry is not None:
            mesh_elem = geometry.find('mesh')
            if mesh_elem is not None:
                filename = mesh_elem.get('filename', '')
                # 处理 package:// 路径
                if filename.startswith('package://'):
                    # 提取包名和相对路径
                    match = re.match(r'package://([^/]+)/(.+)', filename)
                    if match:
                        pkg_name, rel_path = match.groups()
                        # 转换为相对路径（从 MuJoCo XML 文件的角度）
                        # 假设 MuJoCo XML 在 v1 目录，mesh 在 openarm_description/meshes
                        visual_mesh = f"../../openarm_description/meshes/{rel_path}"
                    else:
                        # 简单替换
                        visual_mesh = filename.replace('package://openarm_description/meshes/', 'environment/')
                else:
                    visual_mesh = filename
                
                scale_str = mesh_elem.get('scale', '1 1 1')
                visual_scale = [float(s) for s in scale_str.split()]
    
    # 提取 collision mesh
    collision_mesh = None
    collision_scale = [1.0, 1.0, 1.0]
    collision_origin = {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}
    
    collision = link.find('collision')
    if collision is not None:
        origin = collision.find('origin')
        if origin is not None:
            xyz = origin.get('xyz', '0 0 0').split()
            rpy = origin.get('rpy', '0 0 0').split()
            collision_origin = {
                'xyz': [float(x) for x in xyz],
                'rpy': [float(r) for r in rpy]
            }
        
        geometry = collision.find('geometry')
        if geometry is not None:
            mesh_elem = geometry.find('mesh')
            if mesh_elem is not None:
                filename = mesh_elem.get('filename', '')
                if filename.startswith('package://'):
                    match = re.match(r'package://([^/]+)/(.+)', filename)
                    if match:
                        pkg_name, rel_path = match.groups()
                        collision_mesh = f"../../openarm_description/meshes/{rel_path}"
                    else:
                        collision_mesh = filename.replace('package://openarm_description/meshes/', 'environment/')
                else:
                    collision_mesh = filename
                
                scale_str = mesh_elem.get('scale', '1 1 1')
                collision_scale = [float(s) for s in scale_str.split()]
    
    # 如果没有 collision mesh，使用 visual mesh
    if not collision_mesh and visual_mesh:
        collision_mesh = visual_mesh.replace('textured.dae', 'textured_vhacd.obj')
        if not os.path.exists(os.path.join(base_mesh_dir, collision_mesh)):
            collision_mesh = visual_mesh
    
    # 提取 inertial 信息
    mass = 0.1
    inertia = 0.0001
    inertial = link.find('inertial')
    if inertial is not None:
        mass_elem = inertial.find('mass')
        if mass_elem is not None:
            mass = float(mass_elem.get('value', '0.1'))
        
        inertia_elem = inertial.find('inertia')
        if inertia_elem is not None:
            # 使用平均惯性
            ixx = float(inertia_elem.get('ixx', '0.0001'))
            iyy = float(inertia_elem.get('iyy', '0.0001'))
            izz = float(inertia_elem.get('izz', '0.0001'))
            inertia = (ixx + iyy + izz) / 3.0
    
    # 提取 joint 信息（位置和姿态）
    joint = urdf_xml.find(f".//joint[child[@link='{link_name}']]")
    pos = [0, 0, 0]
    quat = [1, 0, 0, 0]
    
    if joint is not None:
        origin = joint.find('origin')
        if origin is not None:
            xyz = origin.get('xyz', '0 0 0').split()
            rpy = origin.get('rpy', '0 0 0').split()
            pos = [float(x) for x in xyz]
            
            # 将 RPY 转换为四元数
            roll, pitch, yaw = [float(r) for r in rpy]
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)
            
            quat = [
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy
            ]
    
    # 确定 mesh offset（使用 visual origin）
    mesh_offset = visual_origin['xyz']
    
    # 确定 scale（使用 visual scale 的第一个值，假设是均匀缩放）
    scale = visual_scale[0] if visual_scale else 1.0
    
    # 生成相对路径（用于 MuJoCo XML）
    object_name = link_name.replace('_link', '')
    if visual_mesh:
        # 从完整路径提取相对路径
        if 'environment' in visual_mesh:
            visual_relative = visual_mesh.split('environment/')[-1]
        else:
            visual_relative = f"environment/{object_name}/textured.obj"
    else:
        visual_relative = f"environment/{object_name}/textured.obj"
    
    if collision_mesh:
        if 'environment' in collision_mesh:
            collision_relative = collision_mesh.split('environment/')[-1]
        else:
            collision_relative = f"environment/{object_name}/textured_vhacd.obj"
    else:
        collision_relative = f"environment/{object_name}/textured_vhacd.obj"
    
    return {
        'name': object_name,
        'visual_mesh': visual_relative,
        'collision_mesh': collision_relative,
        'pos': pos,
        'quat': quat,
        'mass': mass,
        'inertia': inertia,
        'mesh_offset': mesh_offset,
        'scale': scale
    }


def convert_urdf_to_mujoco(urdf_file, xacro_args=None, output_mesh_dir=None, mujoco_xml_file=None):
    """
    从 URDF/Xacro 文件转换物体到 MuJoCo XML 格式
    
    Args:
        urdf_file: URDF 或 Xacro 文件路径
        xacro_args: Xacro 参数字典（如果输入是 Xacro 文件）
        output_mesh_dir: 输出 mesh 文件的目录（可选，用于复制文件）
        mujoco_xml_file: 要更新的 MuJoCo XML 文件路径（可选）
    
    Returns:
        list: 转换后的物体信息列表
    """
    try:
        # 尝试使用 xacro 处理文件
        try:
            import xacro
            # 如果是 xacro 文件，先处理
            if str(urdf_file).endswith('.xacro'):
                if xacro_args is None:
                    xacro_args = {}
                urdf_string = xacro.process_file(str(urdf_file), mappings=xacro_args).toxml()
            else:
                with open(urdf_file, 'r', encoding='utf-8') as f:
                    urdf_string = f.read()
        except ImportError:
            print("警告: 未安装 xacro，尝试直接解析 URDF")
            with open(urdf_file, 'r', encoding='utf-8') as f:
                urdf_string = f.read()
        
        # 解析 URDF XML
        root = ET.fromstring(urdf_string)
        
        # 查找所有包含 mesh 的 link（环境物体）
        converted_objects = []
        base_mesh_dir = Path(urdf_file).parent.parent / "meshes" if output_mesh_dir is None else Path(output_mesh_dir).parent
        
        for link in root.findall('.//link'):
            link_name = link.get('name', '')
            # 只处理环境物体的 link（apple_link, banana_link 等）
            if link_name.endswith('_link') and any(obj in link_name for obj in ['apple', 'banana', 'table']):
                try:
                    obj_info = parse_urdf_link(root, link_name, str(base_mesh_dir))
                    converted_objects.append(obj_info)
                    print(f"✓ 已解析 URDF link: {link_name} -> {obj_info['name']}")
                except Exception as e:
                    print(f"✗ 解析 {link_name} 失败: {e}")
        
        return converted_objects
        
    except Exception as e:
        print(f"✗ 转换 URDF 失败: {e}")
        import traceback
        traceback.print_exc()
        return []


def main():
    """主函数：转换 YCB 物体或 URDF 模型并更新 MuJoCo XML"""
    import argparse
    
    parser = argparse.ArgumentParser(description='将 YCB 物体或 URDF 模型转换为 MuJoCo XML 格式')
    parser.add_argument('--urdf', type=str, help='从 URDF/Xacro 文件转换（指定文件路径）')
    parser.add_argument('--ycb', action='store_true', help='从 YCB 数据集转换（默认模式）')
    parser.add_argument('--no-prompt', action='store_true', help='不询问直接更新 XML 文件')
    
    args = parser.parse_args()
    
    # 配置路径
    script_dir = Path(__file__).parent
    mujoco_dir = script_dir
    mesh_dir = mujoco_dir / "meshes" / "environment"
    xml_file = mujoco_dir / "openarm_bimanual.xml"
    
    converted_objects = []
    
    # 如果指定了 URDF 文件，从 URDF 转换
    if args.urdf:
        urdf_path = Path(args.urdf)
        if not urdf_path.exists():
            print(f"错误: 找不到 URDF 文件: {urdf_path}")
            return
        
        print(f"从 URDF 文件转换: {urdf_path}")
        converted_objects = convert_urdf_to_mujoco(
            str(urdf_path),
            xacro_args={'use_local_mesh': 'true'},
            output_mesh_dir=str(mesh_dir),
            mujoco_xml_file=str(xml_file)
        )
    
    # 否则使用 YCB 数据集转换（原有功能）
    else:
        # YCB 数据集路径（需要用户指定）
        ycb_base_path = Path("D:/code/ros2_ws/ycb_urdfs/ycb_assets")
        
        # 如果 ycb_urdfs 不存在，尝试其他路径
        if not ycb_base_path.exists():
            ycb_examples_path = Path("D:/code/ros2_ws/ycb_tools/object2urdf/examples/ycb")
            if ycb_examples_path.exists():
                ycb_base_path = ycb_examples_path
            else:
                print("错误: 找不到 YCB 数据集路径")
                print("请设置 ycb_base_path 变量指向 YCB 数据集目录")
                return
        
        # 定义要转换的物体
        objects_to_convert = [
            {
                'name': 'apple',
                'ycb_id': '013_apple',
                'pos': [0.4, -0.12, 0.663],
                'quat': [1, 0, 0, 0],
                'mass': 0.2,
                'inertia': 0.0002,
                'mesh_offset': [-0.013105802251075094, 0.03861242636253375, -0.3633673271131293],
                'scale': 1.0
            },
            {
                'name': 'banana',
                'ycb_id': '011_banana',
                'pos': [0.3, 0.02, 0.471],
                'quat': [0.648, 0, 0, 0.761],
                'mass': 0.12,
                'inertia': 0.00015,
                'mesh_offset': [0.09936102440002159, -0.04072679186328214, -0.1712688923116843],
                'scale': 1.0
            }
        ]
        
        print("开始转换 YCB 物体...")
        
        # 转换每个物体
        for obj_config in objects_to_convert:
            obj_dir = ycb_base_path / obj_config['ycb_id']
            
            if not obj_dir.exists():
                print(f"警告: 找不到 {obj_config['ycb_id']} 目录，跳过")
                continue
            
            try:
                result = convert_ycb_to_mujoco_xml(
                    str(obj_dir),
                    obj_config['name'],
                    str(mesh_dir),
                    scale=obj_config['scale']
                )
                
                # 合并配置信息
                obj_info = {**obj_config, **result}
                converted_objects.append(obj_info)
                
                print(f"✓ 已转换 {obj_config['name']}")
            except Exception as e:
                print(f"✗ 转换 {obj_config['name']} 失败: {e}")
    
    if converted_objects:
        print(f"\n已成功转换 {len(converted_objects)} 个物体")
        print("注意: 此脚本会直接修改 XML 文件，建议先备份")
        
        # 询问是否更新 XML（除非指定了 --no-prompt）
        if args.no_prompt:
            response = 'y'
        else:
            response = input("\n是否更新 MuJoCo XML 文件? (y/n): ")
        
        if response.lower() == 'y':
            update_mujoco_xml(str(xml_file), converted_objects)
            print("完成!")
        else:
            print("已取消更新")
    else:
        print("没有成功转换任何物体")


if __name__ == "__main__":
    main()

