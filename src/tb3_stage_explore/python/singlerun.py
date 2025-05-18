import os
import sys
import signal
import time
import subprocess
from os import makedirs, listdir, remove
from os.path import basename, join, exists
from threading import Thread
from PIL import Image

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from visualization_msgs.msg import MarkerArray
from rclpy.executors import MultiThreadedExecutor

last_update = 0


def kill_process(p):
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except Exception as e:
        print(f"Failed to kill process: {e}")


def get_map(p, folder):
    time.sleep(20)  # Aspetta che /map venga pubblicato
    minutes = 0
    maxmapsave = 60
    seconds_mapsave = 60

    maps_dir = join(folder, "Maps/")
    os.makedirs(maps_dir, exist_ok=True)

    while True:
        if p.poll() is not None:
            return

        map_path = join(maps_dir, f"{int(minutes)}Map")
        save_cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_path,"--ros-args", "--log-level", "error"]
        process = subprocess.Popen(save_cmd, preexec_fn=os.setsid)
        process.wait()

        try:
            Image.open(map_path + ".pgm").save(map_path + ".png")
            remove(map_path + ".pgm")
        except Exception as e:
            print("Cannot convert PGM to PNG:", e)

        if minutes > maxmapsave:
            print("KILLING PROCESS DUE TO TIMEOUT")
            kill_process(p)
            return

        minutes += seconds_mapsave / 60
        time.sleep(seconds_mapsave)



class FrontierListener(Node):
    def __init__(self):
        super().__init__('frontier_listener')
        self.subscription = self.create_subscription(
            MarkerArray,
            '/explore/frontiers',
            self.listener_callback,
            10)
        self.get_logger().info('Subscribed to /explore/frontiers')

    def listener_callback(self, msg):
        global last_update
        if not msg.markers:
            self.get_logger().info('No frontiers detected.')
            last_update = self.get_clock().now().nanoseconds // 1e9
        else:
            self.get_logger().info(f'Detected {len(msg.markers)} frontiers.')
            last_update = time.time()


def run_frontier_listener():
    rclpy.init()
    node = FrontierListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

from rcl_interfaces.msg import Log
from std_msgs.msg import Bool

class ExploreDoneListener(Node):
    def __init__(self, nav_process, folder):
        super().__init__('explore_done_listener')
        self.nav_process = nav_process
        self.folder = folder
        self.subscription = self.create_subscription(
            Bool,
            '/explore/done',
            self.callback,
            10)
        self.get_logger().info('Subscribed to /explore/done.')

    def callback(self, msg):
        if msg.data:
            self.get_logger().info("Exploration finished message received.")
            save_final_map(self.folder)
            kill_process(self.nav_process)



def save_final_map(folder):
    maps_dir = join(folder, "Maps/")
    os.makedirs(maps_dir, exist_ok=True)
    map_path = join(maps_dir, "Map")
    save_cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_path]
    process = subprocess.Popen(save_cmd, preexec_fn=os.setsid)
    process.wait()

    try:
        Image.open(map_path + ".pgm").save(map_path + ".png")
        remove(map_path + ".pgm")
    except Exception as e:
        print("cannot convert final map:", e)


def check_goal_timeout(p, folder):
    global last_update
    first_run = True
    print("Started goal monitor")
    while True:
        time.sleep(10)
        now = time.time()
        delta = now - last_update
        print(f"-- Delta since last goal: {delta:.2f} s")

        if p.poll() is not None or delta > 200:
            if first_run:
                print("Attempting to shutdown explore node")
                subprocess.run(["ros2", "lifecycle", "set", "/explore", "restart"])
                first_run = False
                last_update = now - 100
                continue

            save_final_map(folder)
            print("Timeout reached, killing nav process")
            kill_process(p)
            return


def record_rosbag(folder):
    rosbag_dir = os.path.join(folder, "rosbag")
    os.makedirs(rosbag_dir, exist_ok=True)

    record_cmd = [
        "ros2", "bag", "record",
        "-o", os.path.join(rosbag_dir, "explore"),
        "/ground_truth",
        #"/odom",
        "/base_scan"
    ]
    print("Starting rosbag2 recording...")
    p = subprocess.Popen(record_cmd, preexec_fn=os.setsid)
    return p

def launch_navigation(world_path, folder):
    try:
        rclpy.init()
        
        launch_cmd = [
            "ros2", "launch", "tb3_stage_explore", "turtlebot_stage_explore.launch.py",
            "world:=" + world_path, 
        ]
        print("Launching:", " ".join(launch_cmd))
        p = subprocess.Popen(launch_cmd, preexec_fn=os.setsid)

        bag_process = record_rosbag(folder)

        # Crea nodi
        frontier_node = FrontierListener()
        completion_node = ExploreDoneListener(p, folder)

        # Esegui nodi in executor multithread
        executor = MultiThreadedExecutor()
        executor.add_node(frontier_node)
        executor.add_node(completion_node)

        # Avvia thread executor
        t = Thread(target=executor.spin, daemon=True)
        t.start()

        # Avvia monitoraggio goal e salvataggio mappa
        Thread(target=check_goal_timeout, args=(p, folder), daemon=True).start()
        Thread(target=get_map, args=(p, folder), daemon=True).start()

        p.wait()

        kill_process(bag_process)
        executor.shutdown()
        frontier_node.destroy_node()
        completion_node.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        print("KeyboardInterrupt received, killing...")
        kill_process(p)
        kill_process(bag_process)
    except Exception as e:
        print("Exception in launch_navigation:", e)
        kill_process(p)
        kill_process(bag_process)


def explore_worlds(project_path, world_file):
    out_dir = os.path.join(project_path, "runs", "outputs")
    world_name = os.path.basename(world_file).replace(".world", "")
    folder = os.path.join(out_dir, world_name)

    os.makedirs(folder, exist_ok=True)
    existing_runs = [int(f[3:]) for f in listdir(folder) if f.startswith("run") and f[3:].isdigit()]
    maxrun = max(existing_runs, default=0) + 1
    run_folder = os.path.join(folder, f"run{maxrun}")
    os.makedirs(run_folder, exist_ok=True)

    print(f"Starting run {maxrun} in {run_folder}")
    launch_navigation(world_file, run_folder)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 singlerun.py path_to_world.world")
        sys.exit(1)

    world_path = os.path.abspath(sys.argv[1])
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))    
    explore_worlds(project_root, world_path)
