"""scipy.spatial.transform.Rotation 与旧版 scipy 的兼容层。"""

from __future__ import annotations

from typing import Any

from scipy.spatial.transform import Rotation as R


def rotation_3x3(rot: Any):
    """返回 3x3 旋转矩阵；scipy<1.4 使用 as_dcm()。"""
    fn = getattr(rot, 'as_matrix', None)
    if callable(fn):
        return fn()
    return rot.as_dcm()


def rotation_from_3x3(matrix: Any) -> R:
    """从 3x3 旋转矩阵构建 Rotation；scipy<1.4 使用 from_dcm()."""
    fn = getattr(R, 'from_matrix', None)
    if callable(fn):
        return fn(matrix)
    return R.from_dcm(matrix)
