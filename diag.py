"""
diag.py -- 串口诊断脚本
用法: python diag.py --port COM19
"""
import serial
import time
import sys
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--port", default="COM19")
parser.add_argument("--baud", default=115200, type=int)
args = parser.parse_args()

try:
    ser = serial.Serial(args.port, args.baud, timeout=0.05)
except Exception as e:
    print("打开串口失败:", e)
    sys.exit(1)

time.sleep(0.5)
ser.reset_input_buffer()

# 先监听2秒，确认D:数据正常到达
print("=== 步骤1: 监听原始数据 2秒 ===")
t0 = time.time()
count = 0
while time.time() - t0 < 2:
    if ser.in_waiting:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if line:
            print("  RX:", repr(line))
            count += 1
            if count >= 5:
                break
if count == 0:
    print("  [错误] 没有收到任何数据！检查串口号和波特率")
    ser.close()
    sys.exit(1)

# 发送单个简短命令，看回显
print()
print("=== 步骤2: 发送 'save\\n'，看MCU回显 ===")
ser.reset_input_buffer()
cmd = b"save\n"
print("  TX bytes:", list(cmd))
ser.write(cmd)

t0 = time.time()
while time.time() - t0 < 2:
    if ser.in_waiting:
        line = ser.readline().decode("ascii", errors="replace").strip()
        print("  RX:", repr(line))

# 发送分解命令，一个一个
print()
print("=== 步骤3: 分别发送单参数命令 ===")
for cmd_str in ["g=290\n", "p=1.62\n", "i=0.02\n", "d=0.08\n"]:
    ser.reset_input_buffer()
    time.sleep(0.1)
    print("  TX:", repr(cmd_str), "  bytes:", list(cmd_str.encode("ascii")))
    ser.write(cmd_str.encode("ascii"))
    t0 = time.time()
    got = []
    while time.time() - t0 < 1.0:
        if ser.in_waiting:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if line:
                got.append(line)
                print("  RX:", repr(line))
    if not got:
        print("  [无回显]")

# 发送完整联合命令
print()
print("=== 步骤4: 发送联合命令 ===")
ser.reset_input_buffer()
time.sleep(0.1)
full = "p=1.6200;i=0.0200;d=0.0800;g=290.00\n"
print("  TX:", repr(full))
print("  TX长度:", len(full), "字节")
ser.write(full.encode("ascii"))
t0 = time.time()
got = []
while time.time() - t0 < 2.0:
    if ser.in_waiting:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if line:
            got.append(line)
            print("  RX:", repr(line))
if not got:
    print("  [无回显] MCU未响应联合命令")

# 再次发save确认参数
print()
print("=== 步骤5: 发 'save\\n' 确认当前参数 ===")
ser.reset_input_buffer()
time.sleep(0.1)
ser.write(b"save\n")
t0 = time.time()
while time.time() - t0 < 2:
    if ser.in_waiting:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if line:
            print("  RX:", repr(line))

ser.close()
print()
print("=== 诊断完成 ===")
