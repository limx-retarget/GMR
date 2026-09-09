"""Per-channel rotation offsets for Xsens BVH, stored as JSON.

Kept apart from ``CurveEditor`` so that offline retargeting does not import PyQt6: the editor is
what produces the offsets, but reading them back needs nothing but json.
"""

import json
import os

channel_names = ["X", "Y", "Z"]


class OffsetManager:
    """类用于读取、保存和解析 JSON 文件中的 offset 数据。
    数据格式：{ "joint_name": { "X": offset, "Y": offset, "Z": offset }, ... }
    """

    def __init__(self, default_path="offsets.json"):
        self.default_path = default_path
        self.offsets = self.load_offsets()

    def load_offsets(self, path=None):
        """从指定路径加载 offset。如果路径不存在，则初始化为全 0。"""
        path = path or self.default_path
        if os.path.exists(path):
            try:
                with open(path, "r") as f:
                    data = json.load(f)
                return data
            except (json.JSONDecodeError, IOError) as e:
                print(f"加载 JSON 时出错: {e}. 初始化为全 0。")
        else:
            print(f"路径 {path} 不存在. 初始化为全 0。")
        return {}  # 返回空字典，后续在窗口中填充全 0

    def save_offsets(self, offsets, path):
        """保存 offset 到指定路径。"""
        try:
            with open(path, "w") as f:
                json.dump(offsets, f, indent=4)
            print(f"Offset 已保存至 {path}。")
        except IOError as e:
            print(f"保存 JSON 时出错: {e}。")

    def parse_to_window_format(self, joint_names, offsets_dict):
        """解析 JSON 数据到窗口的 offsets 字典格式 {(joint_idx, channel_idx): offset}。"""
        offsets = {}
        for j, joint in enumerate(joint_names):
            joint_data = offsets_dict.get(joint, {"X": 0.0, "Y": 0.0, "Z": 0.0})
            for c, channel in enumerate(channel_names):
                offsets[(j, c)] = joint_data.get(channel, 0.0)
        return offsets

    def format_for_save(self, offsets, joint_names):
        """将窗口 offsets 格式化为 JSON 保存格式。"""
        save_data = {}
        for j, joint in enumerate(joint_names):
            save_data[joint] = {
                channel_names[c]: offsets.get((j, c), 0.0) for c in range(3)
            }
        return save_data
