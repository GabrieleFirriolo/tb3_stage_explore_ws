import subprocess
import os
from concurrent.futures import ThreadPoolExecutor
import progressbar2 as progressbar
from PIL import Image

def filter_worlds_by_size(worlds_dir, width, height):
    img_dir = os.path.join(worlds_dir, "img/")
    world_files = [f for f in os.listdir(worlds_dir) if f.endswith(".world")]
    filtered_worlds = []

    for world_file in world_files:
        world_name = os.path.splitext(world_file)[0]
        img_path = os.path.join(img_dir, f"{world_name}.png")
        try:
            with Image.open(img_path) as img:
                w, h = img.size
                if w <= width and h <= height:
                    print(w, h)
                    filtered_worlds.append(world_file)
        except Exception as e:
            print(f"Error opening {img_path}: {e}")
    return filtered_worlds

def spawn_container(map_name: str, i: int, bar):
    bar.update(i)
    domain_id = 100 + i
    launch_cmd = [
        "docker", "run", "-it",
        "-e", f"ROS_DOMAIN_ID={domain_id}",
        "-v", f"{os.getcwd()}/worlds:/root/tb3_stage_explore_ws/src/tb3_stage_explore/worlds",
        "-v", f"{os.getcwd()}/output:/root/tb3_stage_explore_ws/src/tb3_stage_explore/runs/outputs",
        "ros2jazzy:explore", f"worlds/{map_name}"
    ]
    subprocess.run(launch_cmd)

def main(workers: int):
    dir_in_worlds =input(" Enter the directory of the world files or . if you want to use the default directory: ")
    if dir_in_worlds == ".":
        worlds_dir = "worlds"
    else:
        worlds_dir = f"worlds/{dir_in_worlds}" 
    width, height = 2500, 2500
    world_files = filter_worlds_by_size(worlds_dir, width, height)
    print(len(world_files))
    pool = ThreadPoolExecutor(max_workers=workers)
    try:
        with progressbar.ProgressBar(max_value=len(world_files)) as bar:
            futures = []
            for i, name in enumerate(world_files):
                futures.append(pool.submit(spawn_container, name, i, bar))
            pool.shutdown(wait=True)
    except Exception as e:
        print(f"Error: {e}")
        pool.shutdown(wait=False)
        subprocess.run("docker kill $(docker ps -q)", shell=True)

if __name__ == "__main__":
    n = input("Enter the number of worker containers to launch: ")
    main(int(n))
    print("All workers have finished.")
