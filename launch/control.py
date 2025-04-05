import sys
import select
import subprocess
import time

NODE_NAME = "/pcd_publisher_single"  # 根据实际节点名修改

def get_current_rate():
    result = subprocess.run(
        ["ros2", "param", "get", NODE_NAME, "playRate"],
        capture_output=True,
        text=True
    )
    try:
        return float(result.stdout.split()[-1])
    except:
        return 1.0

def set_play_rate(rate):
    subprocess.run(
        ["ros2", "param", "set", NODE_NAME, "playRate", str(rate)],
        capture_output=True,
        text=True
    )
    print(f"Set playRate to {rate:.2f}")

def main():
    current_rate = 1.0
    while True:
        # 显示当前速率
        sys.stdout.write(f"\rCurrent playRate: {current_rate:.2f}x | Press '+'/'-' to adjust | 'r' to reset")
        sys.stdout.flush()

        # 检测键盘输入
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == '+':
                current_rate *= 2.0
                set_play_rate(current_rate)
            elif key == '-':
                current_rate /= 2.0
                set_play_rate(current_rate)
            elif key == 'r':
                current_rate = 1.0
                set_play_rate(current_rate)
            elif key == 'q':
                print("\nExiting...")
                return

        # 每秒更新一次显示（避免高CPU占用）
        time.sleep(0.5)
        current_rate = get_current_rate()

if __name__ == "__main__":
    main()