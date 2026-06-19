import rclpy


def _spin_until_complete(node, future, timeout):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.done()


def _value_or_nan(values, index: int) -> float:
    if index >= len(values):
        return float("nan")
    return float(values[index])


def _approach(value: float, target: float, step: float) -> float:
    if value < target:
        return min(value + step, target)
    if value > target:
        return max(value - step, target)
    return value
