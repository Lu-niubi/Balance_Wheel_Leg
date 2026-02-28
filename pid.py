"""
串级PID自动调参脚本 (IAR 半自动手动烧录版)
目标：最小化超调量 + ITAE
算法：贝叶斯优化 (高斯过程 + EI采集函数)
"""

import serial
import subprocess
import re
import time
import numpy as np
from skopt import gp_minimize
from skopt.space import Real
from skopt.utils import use_named_args
import argparse

# ============================================================
# 用户配置区
# ============================================================

SERIAL_PORT   = "COM19"       # 改成你的串口号
BAUD_RATE     = 115200
COLLECT_SECS  = 15.0          # 修改：每次阶跃响应采集时长改为 20 秒
STEP_CHAR     = b'S'          # 触发阶跃的串口字符


# 要修改的源文件路径（含 #define GYRO_KP 等宏的那个 .c 文件）
SOURCE_FILE = r"D:\ZNC\LT\LT\Seekfree_CYT4BB_Opensource_Library\project\code\Moudle\Motor\Motor.c"

# ============================================================
# 参数搜索范围  [min, max]
# ============================================================
PARAM_BOUNDS = {
    # 角速度环
    "GYRO_KP": (100.0,  600.0),
    "GYRO_KI": (0.0,    0.0),    # 固定为0，不参与搜索
    "GYRO_KD": (0.0,    0.0),
    # 角度环
    "ANG_KP":  (0.2,    1.8),
    "ANG_KI":  (0.0,    0.1),
    "ANG_KD":  (0.0,    0.15),
    # 速度环（固定，不搜索）
    "SPD_KP":  (0.10,   0.10),
    "SPD_KI":  (0.001,  0.001), 
    "SPD_KD":  (0.0,    0.0),
}

# 超调量权重 vs ITAE权重（两者归一化后加权）
W_OVERSHOOT = 0.6
W_ITAE      = 0.4

# ============================================================
# 工具函数
# ============================================================

def patch_source(params: dict):
    """用正则替换源文件中的 #define 宏值"""
    try:
        with open(SOURCE_FILE, "r", encoding="utf-8") as f:
            src = f.read()

        for name, val in params.items():
            pattern = rf"(#define\s+{name}\s+)[0-9eE+\-\.]+f?"
            replacement = rf"\g<1>{val:.6f}f"
            src, n = re.subn(pattern, replacement, src)
            if n == 0:
                print(f"  [WARN] 未找到宏 {name}，跳过")

        with open(SOURCE_FILE, "w", encoding="utf-8") as f:
            f.write(src)
        print(f"  [SUCCESS] 成功将参数写入 {SOURCE_FILE}")
    except Exception as e:
        print(f"  [ERROR] 文件读写失败: {e}")

def collect_step_response(port: str, baud: int,
                           duration: float) -> tuple[list, list, list]:
    """
    触发阶跃，采集 duration 秒的 CSV 数据
    """
    ser = serial.Serial(port, baud, timeout=0.1)
    time.sleep(0.3)
    ser.reset_input_buffer()

    # 触发阶跃
    ser.write(STEP_CHAR)

    t_list, actual_list, setpoint_list = [], [], []
    t0 = time.time()

    while time.time() - t0 < duration:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue
        
        parts = line.split(",")
        if len(parts) < 2:
            continue
        try:
            sp  = float(parts[0])
            act = float(parts[1])
            t   = time.time() - t0
            setpoint_list.append(sp)
            actual_list.append(act)
            t_list.append(t)
        except ValueError:
            continue

    ser.close()
    return t_list, setpoint_list, actual_list

def compute_cost(t_list, sp_list, actual_list) -> float:
    """计算代价函数：超调量 + ITAE"""
    if len(t_list) < 10:
        print("  [WARN] 接收到的数据太少，给予高惩罚！请检查串口连接和触发逻辑。")
        return 1e6   # 数据太少，惩罚

    t   = np.array(t_list)
    sp  = np.array(sp_list)
    act = np.array(actual_list)
    err = sp - act

    # 稳态值（取最后20%数据均值）
    n = len(act)
    steady = np.mean(act[int(n * 0.8):])
    target = np.mean(sp[int(n * 0.8):])

    # 超调量（归一化，0~1）
    if abs(target) > 1e-6:
        peak = np.max(act) if target > 0 else np.min(act)
        overshoot = max(0.0, (peak - target) / abs(target))
    else:
        overshoot = 0.0

    # ITAE = integral(t * |e(t)| dt)
    dt = np.diff(t, prepend=t[0])
    itae = float(np.sum(t * np.abs(err) * dt))

    # 归一化 ITAE（除以时间窗口平方，量纲无关）
    T = t[-1] - t[0] + 1e-6
    itae_norm = itae / (T * T)

    cost = W_OVERSHOOT * overshoot + W_ITAE * itae_norm
    print(f"    overshoot={overshoot*100:.1f}%  ITAE_norm={itae_norm:.4f}  cost={cost:.4f}")
    return cost

# ============================================================
# 搜索变量提取
# ============================================================

SEARCH_PARAMS = [(k, v) for k, v in PARAM_BOUNDS.items() if v[0] != v[1]]
FIXED_PARAMS  = {k: v[0] for k, v in PARAM_BOUNDS.items() if v[0] == v[1]}
SPACE = [Real(lo, hi, name=name) for name, (lo, hi) in PARAM_BOUNDS.items() if lo != hi]

eval_count = 0

@use_named_args(SPACE)
def objective(**kwargs):
    global eval_count
    eval_count += 1

    params = {**FIXED_PARAMS, **kwargs}

    print(f"\n" + "="*50)
    print(f"[Eval #{eval_count}] AI建议的新参数：")
    for k in kwargs:
        print(f"  #define {k:<10} {kwargs[k]:.6f}f")
    print("="*50)

    # 修改：注销掉自动修改代码的操作
    # patch_source(params) 

    # 修改：加入手动阻塞确认机制
    input("\n👉 请在 IAR 中手动更新上述参数，完成编译、下载、复位后，【按回车键】开始测试...")

    print(f"  [SERIAL] 正在发送触发指令并采集 {COLLECT_SECS}s 阶跃响应，请稍候...")
    t_list, sp_list, act_list = collect_step_response(
        SERIAL_PORT, BAUD_RATE, COLLECT_SECS)
    
    return compute_cost(t_list, sp_list, act_list)

# ============================================================
# 主程序
# ============================================================

def main():
    global SERIAL_PORT

    parser = argparse.ArgumentParser(description="串级PID自动调参 (IAR手动模式)")
    parser.add_argument("--port",    default=SERIAL_PORT, help="串口号，如 COM3")
    parser.add_argument("--n-calls", type=int, default=30, help="总评估次数（含初始随机点）")
    parser.add_argument("--n-init",  type=int, default=5,  help="初始随机探索次数")
    args = parser.parse_args()

    SERIAL_PORT  = args.port
    search_names = [s.name for s in SPACE]

    print("=" * 60)
    print("串级PID半自动调参  (贝叶斯优化 / 高斯过程)")
    print(f"搜索参数: {search_names}")
    print(f"总次数: {args.n_calls}  初始随机: {args.n_init}")
    print(f"串口: {SERIAL_PORT}  采集时长: {COLLECT_SECS}s")
    print("注意: 脚本运行期间不会修改你的源文件，请根据控制台提示手动填入 IAR。")
    print("=" * 60)

    result = gp_minimize(
        objective,
        SPACE,
        n_calls=args.n_calls,
        n_initial_points=args.n_init,
        acq_func="EI",
        noise=1e-4,
        random_state=42,
        verbose=False, # 关掉底层的 verbose，我们已经自己写了打印逻辑
    )

    print("\n" + "=" * 60)
    print("🎉 调参结束！最优参数如下:")
    best_params = dict(FIXED_PARAMS)
    for name, val in zip(search_names, result.x):
        best_params[name] = val
        print(f"  {name} = {val:.6f}f")
    print(f"最优代价: {result.fun:.4f}")
    print("=" * 60)

    # 修改：只在所有测试结束后，把最终的最优结果写入文档
    print("\n正在将最终最优参数自动写入您的源文件...")
    patch_source(best_params)
    print("写入完成！您可以回到 IAR 进行最终的编译和烧录了。")

if __name__ == "__main__":
    main()