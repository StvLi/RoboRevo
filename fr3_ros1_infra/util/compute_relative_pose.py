"""
BAAI 具身大模型组
李佩泽 TEL:138 1085 0696 E-mail: 3190102290@zju.edu.cn
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

class Pose:
    def __init__(self, translation = [0,0,0], rotation = [0,0,0,1], rot_form = 'quat'):
        """
        初始化一个对象，设置平移和旋转参数。

        参数:
        translation (list): 平移向量，默认为 [0, 0, 0]。
        rotation (list): 旋转参数，默认为 [0, 0, 0, 1]。
        rot_form (str): 旋转形式，可选值包括 'quat', 'rotvec', 'matrix', 'matrix_2r' 或其他欧拉角序列，默认为 'quat'。

        返回值:
        无返回值。
        """

        # 将平移向量转换为NumPy数组并存储在实例变量中
        self.translation = np.array(translation)
        
        # 根据指定的旋转形式初始化旋转对象
        if rot_form == 'quat':
            # 使用四元数初始化旋转
            self.rotation = R.from_quat(rotation)
        elif rot_form == 'rotvec':
            # 使用旋转向量初始化旋转
            self.rotation = R.from_rotvec(rotation)
        elif rot_form == 'matrix' :
            # 使用方向余弦矩阵初始化旋转
            self.rotation = R.from_dcm(rotation)
        elif rot_form == 'matrix_2r':
            # 从两个行向量构造完整的旋转矩阵并初始化旋转
            row1 = np.array(rotation[0:3])
            row2 = np.array(rotation[3:6])
            row3 = np.cross(row1, row2)
            row2 = np.cross(row3,row1)
            matrix = np.vstack([row1, row2, row3])
            self.rotation = R.from_dcm(matrix)
        else:
            # 使用欧拉角初始化旋转，其中seq参数被错误地设置为'rot_form'，应为有效的欧拉角序列
            self.rotation = R.from_euler(
                seq = 'rot_form',
                angles = rotation,
                degrees = False
            )

def compute_relative_pose(target_pose: Pose, refer_pose: Pose):
    """
    计算一个姿态相对于另一个姿态的相对位姿。

    参数:
        target_pose (Pose): 目标姿态，表示需要计算相对位姿的姿态。
        refer_pose (Pose): 参考姿态，表示作为参考坐标系的姿态。

    返回:
        Pose: 返回目标姿态相对于参考姿态的相对位姿，包含平移和旋转信息。
              旋转形式为四元数（quat）。
    """

    # 计算相对平移：将目标姿态的平移向量转换到参考姿态的坐标系中
    relative_translation = refer_pose.rotation.as_dcm() @ (target_pose.translation - refer_pose.translation)

    # 计算相对旋转：通过参考姿态的逆旋转与目标姿态的旋转相乘得到相对旋转
    relative_rotation = (refer_pose.rotation.inv() * target_pose.rotation).as_quat()

    # 构造并返回相对位姿对象
    return Pose(translation=relative_translation, rotation=relative_rotation, rot_form='quat')

def relative_actions_from_init(target_states, init_states):
    """
    计算从初始状态到目标状态的相对动作序列。

    参数:
        target_states (list of list): 目标状态列表，每个状态包含平移（前3个元素）和四元数旋转（后4个元素）。
        init_states (list): 初始状态，包含平移（前3个元素）和四元数旋转（后4个元素）。

    返回:
        list of list: 相对动作序列，每个动作包含相对于初始姿态的平移和旋转向量表示的旋转。
    """
    # 将初始状态转换为Pose对象，便于后续计算
    init_pose = Pose(translation=init_states[0:3], rotation=init_states[3:7], rot_form='quat')
    actions = []
    
    # 遍历所有目标状态，计算相对于初始状态的相对动作
    for target_state in target_states:
        # 将当前目标状态转换为Pose对象
        target_pose = Pose(translation=target_state[0:3], rotation=target_state[3:7], rot_form='quat')
        
        # 计算目标姿态相对于初始姿态的相对姿态
        relative_pose = compute_relative_pose(target_pose, init_pose)
        
        # 将相对姿态的平移和旋转（以旋转向量形式表示）合并为一个动作，并添加到结果列表中
        actions.append(np.hstack([relative_pose.translation, relative_pose.rotation.as_rotvec()]))
    
    return actions

def differential_actions_between_step(states):
    """
    计算连续状态之间的差分动作。

    该函数接收一个状态序列，计算每两个相邻状态之间的相对位姿变化，
    并将这些变化表示为平移和旋转（以旋转向量形式）的组合。

    参数:
        states (list of lists): 状态序列，每个状态是一个列表，前3个元素表示平移，
                                后4个元素表示四元数形式的旋转。

    返回:
        list of numpy arrays: 差分动作序列，每个动作是一个numpy数组，
                              前3个元素表示平移差分，后3个元素表示旋转差分（旋转向量形式）。
    """
    actions = []
    # 遍历状态序列，从第二个状态开始
    for i in range(1, len(states)):
        # 构造前一个状态的位姿对象
        prev_pose = Pose(translation=states[i-1][0:3], rotation=states[i-1][3:7], rot_form='quat')
        # 构造下一个状态的位姿对象
        next_pose = Pose(translation=states[i][0:3], rotation=states[i][3:7], rot_form='quat')
        # 计算从prev_pose到next_pose的相对位姿
        diff_pose = compute_relative_pose(next_pose, prev_pose)
        # 将相对位姿的平移和旋转（转换为旋转向量）合并为一个动作向量
        actions.append(np.hstack([diff_pose.translation, diff_pose.rotation.as_rotvec()]))
    return actions