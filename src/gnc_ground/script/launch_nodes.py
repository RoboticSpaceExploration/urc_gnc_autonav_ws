import subprocess
import time

def launch_ros_node(launch_file):
    """ Launch a ROS node using a specified .launch file """
    try:
        # Start the ROS node as a subprocess
        process = subprocess.Popen(['roslaunch', launch_file])
        print(f"Successfully launched {launch_file}")
        return process
    except subprocess.CalledProcessError as e:
        print(f"Failed to launch {launch_file}: {str(e)}")

def main():
    # List of launch files for hardware nodes
    launch_files = ['camera.launch', 'lidar.launch']

    # List to keep track of processes
    processes = []

    # Launch each node in the list
    for launch_file in launch_files:
        proc = launch_ros_node(launch_file)
        if proc:
            processes.append(proc)
        # Optional delay between launches if needed
        time.sleep(2)

    # Keep the script running until all nodes are terminated
    try:
        while True:
            time.sleep(10)
    except KeyboardInterrupt:
        # Kill all processes if the script is stopped manually
        for proc in processes:
            proc.kill()
        print("Stopped all ROS nodes.")

if __name__ == "__main__":
    main()

