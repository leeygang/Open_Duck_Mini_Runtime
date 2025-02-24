import rustypot
import time
import numpy as np
from mini_bdx_runtime.onnx_infer import OnnxInfer


ids = [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33]

io = rustypot.feetech("/dev/ttyACM0", 1000000)

policy = OnnxInfer("/home/bdxv2/ONNX_MUJOCO.onnx", awd=True)

goal_times = list(np.zeros(14, dtype=np.uint16))
times = []
start = time.time()
obs = np.zeros(98)
while True:
    if time.time() - start > 5:
        break
    s = time.time()


    io.read_present_position(ids)
    io.set_goal_time(ids, goal_times)

    s_policy = time.time()
    action = policy.infer(obs)
    policy_took = time.time() - s_policy
    print("policy_took", policy_took)


    took = time.time() - s
    print("took", took)
    times.append(took)
    time.sleep(0.01)

print("avg", np.mean(times))
