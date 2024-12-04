import subprocess
import threading
import time
import os
import signal

class ROSLaunchManager:
    def __init__(self):
        self.node_processes = {}
        self.lock = threading.Lock()

    def start_launch(self, package_name, launch_file):
        with self.lock:
            key = (package_name, launch_file)
            if key in self.node_processes:
                print(f"Launch file {launch_file} for package {package_name} is already running.")
                return
            env = os.environ.copy()
            if 'ROS_NAMESPACE' in env:
                del env['ROS_NAMESPACE']

            process = subprocess.Popen(['ros2', 'launch', package_name, launch_file +'.launch.py'], env=env)
            self.node_processes[key] = process
            print(f"Started launch file {launch_file} for package {package_name}.")

    def stop_launch(self, package_name, launch_file):
        with self.lock:
            key = (package_name, launch_file)
            process = self.node_processes.pop(key, None)
            if process:
                # Terminate the entire process group
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait()  # Wait for the process to terminate
                print(f"Stopped launch file {launch_file} for package {package_name}.", flush=True)
            else:
                print(f"Launch file {launch_file} for package {package_name} is not running.", flush=True)


    def restart_launch(self, package_name, launch_file):
        with self.lock:
            self.stop_launch(package_name, launch_file)
            time.sleep(1)  # Optionally wait a bit before restarting
            self.start_launch(package_name, launch_file)

    def stop_all_launches(self):
        with self.lock:
            for key in list(self.node_processes.keys()):
                package_name, launch_file = key
                self.stop_launch(package_name, launch_file)
            print("Stopped all running launch files.")

    def list_running_launches(self):
        # with self.lock:
            # return list(self.node_processes.keys())
        with self.lock:
            # Clean up terminated processes
            for key, process in list(self.node_processes.items()):
                if process.poll() is not None:  # Process has terminated
                    self.node_processes.pop(key)
            
            # Return the updated list of running processes
            return list(self.node_processes.keys())