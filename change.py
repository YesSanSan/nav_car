import tkinter as tk
from tkinter import ttk, messagebox
import serial
import threading
import time

# ----------------串口配置----------------
SERIAL_PORT = "COM17"   # 根据实际修改
BAUD_RATE = 921600

# ----------------全局变量----------------
ser = None
running = False

# ----------------串口发送函数----------------
def send_command(cmd: str):
    if ser and ser.is_open:
        ser.write(("\n" + cmd + "\n").encode())
        print(f"Sent: {cmd}")

# ----------------启动/停止功能----------------
def start():
    global running
    running = True
    send_pid_speed()

def stop():
    global running
    running = False
    send_command("pause 1")  # 暂停STM32运动

def safe_float(entry, default=0.0):
    """安全转换输入框内容为浮点数"""
    try:
        val_str = entry.get().strip()
        if val_str == "":
            return default
        return float(val_str)
    except ValueError:
        return default

# ----------------读取输入并发送----------------
def send_pid_speed():
    if not running:
        return
    # 获取PID参数和速度
    pid_kp = [safe_float(entry_kp[i]) for i in range(4)]
    pid_ki = [safe_float(entry_ki[i]) for i in range(4)]
    pid_kd = [safe_float(entry_kd[i]) for i in range(4)]
    speed = safe_float(entry_speed)

    # 构建命令（自定义协议）
    # 格式：PID,<轮号>,<kp>,<ki>,<kd>
    for i in range(4):
        cmd = f"PID,{i},{pid_kp[i]},{pid_ki[i]},{pid_kd[i]}"
        send_command(cmd)

    # 速度命令
    send_command(f"SPEED,{speed}")
    send_command("pause 0")  # 取消暂停

    # 每200ms发送一次
    root.after(200, send_pid_speed)

# ----------------GUI布局----------------
root = tk.Tk()
root.title("四轮PID调参工具")

mainframe = ttk.Frame(root, padding="10")
mainframe.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

entry_kp = []
entry_ki = []
entry_kd = []

for i in range(4):
    ttk.Label(mainframe, text=f"轮{i} Kp:").grid(row=i, column=0)
    kp = ttk.Entry(mainframe, width=7)
    kp.grid(row=i, column=1)
    kp.insert(0, "0.0")
    entry_kp.append(kp)

    ttk.Label(mainframe, text="Ki:").grid(row=i, column=2)
    ki = ttk.Entry(mainframe, width=7)
    ki.grid(row=i, column=3)
    ki.insert(0, "0.0")
    entry_ki.append(ki)

    ttk.Label(mainframe, text="Kd:").grid(row=i, column=4)
    kd = ttk.Entry(mainframe, width=7)
    kd.grid(row=i, column=5)
    kd.insert(0, "0.0")
    entry_kd.append(kd)

# 速度输入
ttk.Label(mainframe, text="速度期望值 (m/s):").grid(row=4, column=0, columnspan=2)
entry_speed = ttk.Entry(mainframe, width=7)
entry_speed.grid(row=4, column=2)
entry_speed.insert(0, "0.0")

# 启动/停止按钮
ttk.Button(mainframe, text="启动", command=start).grid(row=5, column=0, columnspan=2, sticky=(tk.W, tk.E))
ttk.Button(mainframe, text="停止", command=stop).grid(row=5, column=2, columnspan=2, sticky=(tk.W, tk.E))

# ----------------串口初始化----------------
def init_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print("串口已打开")
    except Exception as e:
        messagebox.showerror("串口错误", str(e))
        root.quit()

# 启动串口
init_serial()
root.mainloop()
