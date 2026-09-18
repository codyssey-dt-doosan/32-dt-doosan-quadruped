"""dataclass 필드를 ROS 파라미터로 선언하는 보조 함수."""

from __future__ import annotations

from dataclasses import MISSING, fields


def declare_dataclass_params(node, cls):
    """dataclass 필드를 ROS 파라미터로 선언하고 채운 dataclass를 돌려준다."""
    values = {}
    for f in fields(cls):
        default = f.default_factory() if f.default_factory is not MISSING else f.default
        if isinstance(default, list) and not default:
            # 빈 리스트는 타입을 추론할 수 없다(BYTE_ARRAY로 잡힘): 실수 배열로 선언
            from rclpy.parameter import Parameter
            v = node.declare_parameter(f.name, Parameter.Type.DOUBLE_ARRAY).value
            values[f.name] = list(v) if v is not None else []
        else:
            values[f.name] = node.declare_parameter(f.name, default).value
    return cls(**values)
