"""Joint target validation shared by the activation probe and offline tests."""

import math


def joint_positions(message, side):
    expected = [f"{side}_fr3v2_joint{i}" for i in range(1, 8)]
    if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
        raise RuntimeError(f"{side}: malformed JointState")
    values = dict(zip(message.name, message.position))
    if not all(name in values for name in expected):
        raise RuntimeError(f"{side}: required FR3v2 joints are missing")
    result = [values[name] for name in expected]
    if not all(math.isfinite(v) for v in result):
        raise RuntimeError(f"{side}: non-finite joint position")
    return result


def alignment_error(target, measured, side):
    expected = [f"{side}_fr3v2_joint{i}" for i in range(1, 8)]
    # The site controller consumes the first seven positions by index, not by name.
    if list(target.name) != expected:
        raise RuntimeError(f"{side}: controller target order does not match the hardware")
    error = max(abs(a - b) for a, b in zip(joint_positions(target, side), joint_positions(measured, side)))
    if error > 0.003:
        raise RuntimeError(f"{side}: target differs from measured position by {error:.6f} rad")
    return error
