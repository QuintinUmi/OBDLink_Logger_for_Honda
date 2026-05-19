import matplotlib.pyplot as plt
import pandas as pd
from pathlib import Path

SIGNALS_DIR = Path("captures\\20260514_235604\\signals")

# 自动查找所有轮速信号
wheel_speed_files = sorted(SIGNALS_DIR.glob("WHEEL_SPEEDS_WHEEL_SPEED_*.csv"))


def sanitize(name: str) -> str:
    return "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in name)

# 读取并绘制单个信号
def plot_signal(signal_name, ax=None):
    file = SIGNALS_DIR / f"{sanitize(signal_name)}.csv"
    if not file.exists():
        print(f"File not found: {file}")
        return
    df = pd.read_csv(file)
    ax = ax or plt.gca()
    ax.plot(df["timestamp"], df["value"], label=signal_name)
    ax.set_title(signal_name)
    ax.set_xlabel("timestamp")
    ax.set_ylabel("value")
    ax.legend()

# 1. ENGINE_DATA.XMISSION_SPEED2
plt.figure(figsize=(10, 4))
plot_signal("ENGINE_DATA.XMISSION_SPEED2")
plt.tight_layout()

# 2. GAS_PEDAL_2.CAR_GAS
plt.figure(figsize=(10, 4))
plot_signal("GAS_PEDAL_2.CAR_GAS")
plt.tight_layout()

# 3. STEERING_SENSORS.STEER_ANGLE
plt.figure(figsize=(10, 4))
plot_signal("STEERING_SENSORS.STEER_ANGLE")
plt.tight_layout()

# 4. STEERING_SENSORS.STEER_ANGLE_RATE
plt.figure(figsize=(10, 4))
plot_signal("STEERING_SENSORS.STEER_ANGLE_RATE")
plt.tight_layout()

# 5. POWERTRAIN_DATA.BRAKE_SWITCH
plt.figure(figsize=(10, 4))
plot_signal("POWERTRAIN_DATA.BRAKE_SWITCH")
plt.tight_layout()

# 6. WHEEL_SPEEDS.WHEEL_SPEED_* (same window)
if wheel_speed_files:
    plt.figure(figsize=(10, 4))
    for file in wheel_speed_files:
        df = pd.read_csv(file)
        plt.plot(df["timestamp"], df["value"], label=file.stem)
    plt.title("WHEEL_SPEEDS.WHEEL_SPEED_*")
    plt.xlabel("timestamp")
    plt.ylabel("value")
    plt.legend()
    plt.tight_layout()
else:
    print("No wheel speed files found.")

plt.show()
