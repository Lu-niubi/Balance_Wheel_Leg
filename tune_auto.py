"""
tune_auto.py  --  自动调参脚本（静止采集版）
数据格式：printf("D:%.2f,%.2f,%.4f\n", -8.0, imu_sys.pitch, imu_sys.gx);
         前缀 D: 后接 字段0=目标pitch, 字段1=当前pitch, 字段2=角速度

用法：
    python tune_auto.py --port COM19 --baud 115200
    python tune_auto.py --port COM19 --diag     # 诊断模式
    python tune_auto.py --port COM19 --rounds 20 --settle 3
"""

import serial
import time
import argparse
import sys
from dataclasses import dataclass
from typing import List

DATA_PREFIX = "D:"

# ─────────────────────────────────────────────
#  调参搜索空间（按历史参数范围展开）
#  (ANG_KP, ANG_KI, ANG_KD, GYRO_KP)
# ─────────────────────────────────────────────
PARAM_GRID = [
    # --- GYRO_KP=240 组 (当前在用) ---
    (1.20, 0.02, 0.09, 240),
    (1.20, 0.04, 0.09, 240),
    (1.40, 0.02, 0.05, 240),
    (1.40, 0.04, 0.09, 240),
    (1.50, 0.02, 0.05, 240),
    # --- GYRO_KP=380 组 ---
    (1.20, 0.04, 0.09, 380),
    (1.40, 0.02, 0.09, 380),
    (1.40, 0.04, 0.09, 380),
    (1.50, 0.02, 0.05, 380),
    (1.50, 0.04, 0.09, 380),
    # --- GYRO_KP=480 组 ---
    (1.00, 0.02, 0.09, 480),
    (1.20, 0.02, 0.09, 480),
    (1.20, 0.04, 0.09, 480),
    (1.40, 0.02, 0.09, 480),
    (1.40, 0.04, 0.09, 480),
    # --- GYRO_KP=200 组 (小值探索) ---
    (1.40, 0.02, 0.09, 200),
    (1.50, 0.04, 0.09, 200),
    (0.80, 0.02, 0.10, 200),
    # --- 历史最优附近精调 ---
    (1.24, 0.006, 0.108, 171),
    (1.32, 0.006, 0.051, 191),
]

SETTLE_SECS  = 3    # 发完参数后等待控制器稳定（秒）
COLLECT_SECS = 10   # 采集数据时长（秒）


@dataclass
class TrialResult:
    idx:      int
    kp:       float
    ki:       float
    kd:       float
    gyro_kp:  float
    rmse:     float
    max_err:  float
    std_err:  float
    osc_count: int   # 过零次数 (振荡指标)
    sample_count: int
    score:    float = 0.0


def compute_score(r: TrialResult) -> float:
    """
    评分逻辑（越低越好）：
    - RMSE: 稳态误差，权重最高
    - std: 抖动/振荡程度
    - osc_count: 过零振荡次数
    - max_err: 最大瞬时偏差
    """
    if r.sample_count < 50:
        return 9999.0
    s  = r.rmse * 3.0
    s += r.std_err * 2.0
    s += r.osc_count * 0.1
    s += r.max_err * 0.3
    return s


def parse_data_line(line: str):
    """解析 D: 前缀数据行，返回 (target, pitch, gyro) 或 None"""
    if not line.startswith(DATA_PREFIX):
        return None
    body = line[len(DATA_PREFIX):]
    parts = body.split(",")
    if len(parts) < 3:
        return None
    try:
        return float(parts[0]), float(parts[1]), float(parts[2])
    except ValueError:
        return None


def send_params(ser: serial.Serial, kp, ki, kd, gyro_kp):
    """发送参数命令，等待MCU回显[Tune]行"""
    ser.reset_input_buffer()
    cmd = "p={:.4f};i={:.4f};d={:.4f};g={:.2f}\n".format(kp, ki, kd, gyro_kp)
    ser.write(cmd.encode("ascii"))
    print("  >>> 发送: {}".format(cmd.strip()))

    tune_lines = []
    deadline = time.time() + 2.0
    while time.time() < deadline and len(tune_lines) < 4:
        if ser.in_waiting == 0:
            time.sleep(0.01)
            continue
        try:
            line = ser.readline().decode("ascii", errors="replace").strip()
        except Exception:
            continue
        if line.startswith("[Tune]"):
            tune_lines.append(line)
            print("  MCU: {}".format(line))

    if not tune_lines:
        print("  [警告] 未收到[Tune]回显，请检查串口连接")
    else:
        print("  [OK] 收到 {} 条[Tune]回显".format(len(tune_lines)))
    return len(tune_lines)


def wait_settle(ser: serial.Serial, secs: int):
    """等待控制器稳定，持续排空缓冲区"""
    print("  ... 等待 {}秒 控制器稳定 ...".format(secs))
    t0 = time.time()
    while time.time() - t0 < secs:
        if ser.in_waiting > 0:
            ser.read(ser.in_waiting)
        time.sleep(0.02)


def collect_data(ser: serial.Serial, duration: float):
    """采集 duration 秒的 D: 前缀数据，实时显示"""
    ser.reset_input_buffer()
    timestamps, errors, gyros = [], [], []
    t0 = time.time()
    while time.time() - t0 < duration:
        if ser.in_waiting == 0:
            time.sleep(0.001)
            continue
        try:
            line = ser.readline().decode("ascii", errors="replace").strip()
        except Exception:
            continue
        parsed = parse_data_line(line)
        if parsed is None:
            continue
        target, pitch, gyro = parsed
        err = pitch - target
        t   = time.time() - t0
        timestamps.append(t)
        errors.append(err)
        gyros.append(gyro)

        bar_len = int(min(abs(err) / 5.0 * 20, 20))
        bar = ("|" * bar_len).ljust(20)
        print("\r  t={:5.1f}s  pitch={:7.3f}  err={:+7.3f}  gyro={:+8.4f}  [{}]".format(
            t, pitch, err, gyro, bar), end="", flush=True)
    print()
    return timestamps, errors, gyros


def count_zero_crossings(errors: list) -> int:
    """统计误差过零次数（振荡指标）"""
    count = 0
    for i in range(1, len(errors)):
        if errors[i - 1] * errors[i] < 0:
            count += 1
    return count


def std_dev(vals: list) -> float:
    if len(vals) < 2:
        return 0.0
    mean = sum(vals) / len(vals)
    return (sum((v - mean) ** 2 for v in vals) / len(vals)) ** 0.5


def analyze(idx, kp, ki, kd, gyro_kp, timestamps, errors, gyros) -> TrialResult:
    if not errors:
        return TrialResult(idx, kp, ki, kd, gyro_kp,
                           rmse=999, max_err=999, std_err=999,
                           osc_count=0, sample_count=0)

    rmse    = (sum(e**2 for e in errors) / len(errors)) ** 0.5
    max_err = max(abs(e) for e in errors)
    std     = std_dev(errors)
    osc     = count_zero_crossings(errors)

    return TrialResult(idx, kp, ki, kd, gyro_kp,
                       rmse=rmse, max_err=max_err, std_err=std,
                       osc_count=osc, sample_count=len(errors))


def print_summary(results: List[TrialResult]):
    print()
    print("=" * 105)
    print("{:>3}  {:>6}  {:>6}  {:>6}  {:>6}  {:>7}  {:>7}  {:>7}  {:>5}  {:>6}  {:>7}".format(
        "#", "KP", "KI", "KD", "GKP", "RMSE", "MaxErr", "Std", "Osc", "Smpls", "Score"))
    print("-" * 105)
    for r in sorted(results, key=lambda x: x.score):
        print("{:>3}  {:>6.3f}  {:>6.4f}  {:>6.4f}  {:>6.1f}  {:>7.4f}  {:>7.4f}  {:>7.4f}  {:>5d}  {:>6d}  {:>7.4f}".format(
            r.idx + 1, r.kp, r.ki, r.kd, r.gyro_kp,
            r.rmse, r.max_err, r.std_err, r.osc_count, r.sample_count, r.score))
    print("=" * 105)

    valid = [r for r in results if r.sample_count >= 50]
    if valid:
        best = min(valid, key=lambda x: x.score)
        print("\n最佳参数（第 {} 轮，score={:.4f}）：".format(best.idx + 1, best.score))
        print("  p={:.4f};i={:.4f};d={:.4f};g={:.2f}".format(
            best.kp, best.ki, best.kd, best.gyro_kp))
        print("\n  固件中设置：")
        print("  float GYRO_KP = {:.4f}f;".format(best.gyro_kp))
        print("  float ANG_KP  = {:.4f}f;".format(best.kp))
        print("  float ANG_KI  = {:.4f}f;".format(best.ki))
        print("  float ANG_KD  = {:.4f}f;".format(best.kd))
    print()


def run_diag(ser: serial.Serial, secs=8):
    print("=== 诊断模式：打印 {}秒 原始串口数据 ===".format(secs))
    t0 = time.time()
    data_count = 0
    tune_count = 0
    while time.time() - t0 < secs:
        if ser.in_waiting == 0:
            time.sleep(0.005)
            continue
        try:
            line = ser.readline().decode("ascii", errors="replace").strip()
        except Exception:
            continue
        print("  RAW: [{}]".format(line))
        if line.startswith(DATA_PREFIX):
            data_count += 1
        if line.startswith("[Tune]"):
            tune_count += 1
    print("=== 诊断结束：D:行={}, [Tune]行={} ===".format(data_count, tune_count))
    if data_count == 0:
        print("[问题] 未收到任何 D: 数据行！检查串口号/波特率/固件版本")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port",    default="COM19",   help="串口号")
    parser.add_argument("--baud",    default=115200, type=int, help="波特率")
    parser.add_argument("--settle",  default=SETTLE_SECS,  type=int, help="稳定等待(秒)")
    parser.add_argument("--collect", default=COLLECT_SECS, type=int, help="采集时长(秒)")
    parser.add_argument("--rounds",  default=20, type=int, help="调参轮数")
    parser.add_argument("--diag",    action="store_true", help="诊断模式")
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print("串口打开失败: {}".format(e))
        sys.exit(1)

    time.sleep(0.5)
    ser.reset_input_buffer()

    if args.diag:
        run_diag(ser, secs=8)
        ser.close()
        return

    rounds = min(args.rounds, len(PARAM_GRID))
    results: List[TrialResult] = []

    print("全自动模式：发参数 -> 等{}s稳定 -> 采集{}s -> 自动下一组".format(
        args.settle, args.collect))
    print("请保持小车直立静止，脚本自动完成全部 {} 轮\n".format(rounds))

    for idx in range(rounds):
        kp, ki, kd, gyro_kp = PARAM_GRID[idx]
        print()
        print("=" * 60)
        print("  第 {}/{} 轮".format(idx + 1, rounds))
        print("  ANG_KP={:.4f}  ANG_KI={:.4f}  ANG_KD={:.4f}  GYRO_KP={:.2f}".format(
            kp, ki, kd, gyro_kp))
        print("=" * 60)

        # 1. 发送参数
        send_params(ser, kp, ki, kd, gyro_kp)

        # 2. 等控制器稳定
        wait_settle(ser, args.settle)

        # 3. 采集数据
        print("  >>> 采集 {} 秒静止数据...".format(args.collect))
        timestamps, errors, gyros = collect_data(ser, args.collect)

        # 4. 分析
        result = analyze(idx, kp, ki, kd, gyro_kp, timestamps, errors, gyros)
        result.score = compute_score(result)
        results.append(result)

        print("  本轮: RMSE={:.4f}°  Max={:.4f}°  Std={:.4f}°  Osc={}  采样={}  Score={:.4f}".format(
            result.rmse, result.max_err, result.std_err,
            result.osc_count, result.sample_count, result.score))

        # 实时排名
        ranked = sorted([r for r in results if r.sample_count >= 50], key=lambda x: x.score)
        if ranked:
            b = ranked[0]
            print("  当前最优: 第{}轮 KP={:.3f} KD={:.4f} GKP={:.0f} Score={:.4f}".format(
                b.idx + 1, b.kp, b.kd, b.gyro_kp, b.score))

    ser.close()
    print_summary(results)


if __name__ == "__main__":
    main()
