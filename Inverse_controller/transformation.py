'''
说明：
    该文件提供坐标变换相关工具函数

'''






import numpy as np





def fix_quat_continuity(quat_xyzw, prev_quat, jump_count):
    """
    修复四元数符号翻转导致的不连续跳变。
    
    Args:
        quat_xyzw: 当前帧四元数 [x, y, z, w]
        prev_quat: 上一帧四元数（首帧传 None）
        jump_count: 累计跳变次数
    Returns:
        (修正后的四元数, 更新后的跳变计数)
    """
    if prev_quat is None:
        return quat_xyzw, jump_count
    
    # 点积 < 0 → q 和 -q 表示同一旋转，但符号翻了
    if np.dot(quat_xyzw, prev_quat) < 0.0:
        quat_xyzw = -quat_xyzw
        jump_count += 1
    
    return quat_xyzw, jump_count

def quat_to_axes_xyzw(quat_xyzw):
    """
    将四元数从 [x, y, z, w] 格式转换为 [w, x, y, z] 格式。
    
    Args:
        quat_xyzw: 四元数 [x, y, z, w]
    Returns:
        四元数 [w, x, y, z]
    """
    x, y, z, w = quat_xyzw
    return np.array([w, x, y, z])