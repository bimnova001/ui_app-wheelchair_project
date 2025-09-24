import subprocess
import time
import os

def run(cmd, cwd=None):
    print(f"Starting: {' '.join(cmd)}")
    return subprocess.Popen(cmd, cwd=cwd)

def main():
    # กำหนด ROS2 environment
    os.environ["ROS_DOMAIN_ID"] = "10"  # ปรับตามที่ใช้จริง

    processes = []

    # 1. Start robot bringup
    processes.append(run(["ros2", "launch", "turtlebot3_bringup", "robot.launch.py"]))
    time.sleep(5)

    # 2. Start SLAM Mapping
    processes.append(run(["ros2", "launch", "slam_toolbox", "online_async_launch.py", "use_sim_time:=true"]))
    time.sleep(5)

    # 3. Start Nav2 navigation
    processes.append(run(["ros2", "launch", "nav2_bringup", "navigation_launch.py", "use_sim_time:=True"]))
    time.sleep(5)

    # 4. Start ros2 bridge (websocket)
    processes.append(run(["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml"]))
    time.sleep(2)

    # 5. Start burger_camera_node.py
    processes.append(run(["python3", "burger_camera_node.py"], cwd="pages"))
    time.sleep(2)

    # 6. Start burger_detection_node.py
    processes.append(run(["python3", "burger_detection_node.py"], cwd="pages"))

    # 7. (เพิ่ม node อื่นๆได้ที่นี่)

    print("All nodes started. Press Ctrl+C to stop.")

    try:
        # รอจนกว่าจะกด Ctrl+C
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping all nodes...")
        for p in processes:
            p.terminate()
        for p in processes:
            p.wait()
        print("All nodes stopped.")

if __name__ == "__main__":
    main()