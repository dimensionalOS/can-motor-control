import json
import resource
import time

import numpy as np

import can_motor_control as c
from can_motor_control.robstride import MotorType, RobstrideCodec


def main():
    motors = [
        c.MotorSpec(
            f"joint{i}",
            MotorType.RS06 if i <= 3 else MotorType.RS00,
            send_id=i,
            recv_id=i,
        )
        for i in range(1, 8)
    ]
    robot = (
        c.RobotBuilder()
        .add_bus("main", c.MockCanBus("mock"), RobstrideCodec())
        .add_arm("arm", bus="main", motors=motors)
        .build()
    )
    robot.connect()
    robot.enable()
    arm = robot["arm"]
    commands = np.zeros((7, 5), dtype=np.float64)
    commands[:, 0] = 2.0
    commands[:, 1] = 0.1
    samples = []
    started = time.perf_counter()
    cpu_started = time.process_time()
    try:
        for _ in range(10000):
            tick_started = time.perf_counter_ns()
            arm.mit_control(commands)
            robot.tick(0)
            samples.append(time.perf_counter_ns() - tick_started)
    finally:
        robot.disable()
    samples.sort()
    print(
        json.dumps(
            {
                "transport": "MockCanBus",
                "motor_count": 7,
                "iterations": len(samples),
                "p50_ns": samples[len(samples) // 2],
                "p99_ns": samples[len(samples) * 99 // 100],
                "maximum_ns": samples[-1],
                "elapsed_s": time.perf_counter() - started,
                "process_cpu_s": time.process_time() - cpu_started,
                "max_rss_kib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                "physical_can_access": False,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
