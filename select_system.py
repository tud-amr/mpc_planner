import sys, os

# NOTE: This script prevents system specific packages from being built when ran.

sys.path.append(os.path.join(sys.path[0], "solver_generator"))

from pathlib import Path
import shutil

from util.logging import print_value, print_warning

skip_packages = ["solver_generator", "docs", ".github", "mpc_planner_util", "mpc_planner_modules", "mpc_planner_msgs", "mpc_planner_solver", "mpc_planner_types", "mpc_planner"]


def get_files(folder, ros_version: str):
    cmake_file = Path(os.path.join(folder, f"CMakeLists{ros_version}.txt"))
    package_file = Path(os.path.join(folder, f"package{ros_version}.xml"))

    exists = True
    if not cmake_file.is_file():
        print(f"CMakeLists{ros_version}.txt does not exist in {folder}")
        exists = False

    if not package_file.is_file():
        print(f"package{ros_version}.xml does not exist in {folder}")
        exists = False

    return exists, [cmake_file, package_file]


def get_ros_mode(file_name) -> int:
    # Check if it is in ROS1 or ROS2 mode currently
    with open(file_name, "r") as file:
        content = file.read()
        if "catkin_package" in content:
            return 1
        elif "find_package(ament_cmake REQUIRED)" in content:
            return 2
        else:
            raise IOError(f"{file_name} is not in ROS1 or ROS2 mode")


def main():
    # Check if the correct number of command line arguments is provided
    if len(sys.argv) != 2:
        print("Usage: python select_system.py <system_name>")
        return

    system = sys.argv[1]
    print_value("Selected System", system)
    subfolders = [f.path for f in os.scandir(sys.path[0]) if f.is_dir() and len(f.name.split(".")) == 1 and f.name not in skip_packages]

    exist, check_files = get_files(subfolders[0], "")
    if not exist:
        cur_ros_mode = 0
    else:
        cur_ros_mode = get_ros_mode(check_files[0])

    if cur_ros_mode == 1:
        ros_build_name = "CATKIN"
    elif cur_ros_mode == 2:
        ros_build_name = "COLCON"

    for folder in subfolders:
        package_name = folder.split("/")[-1]
        if package_name not in skip_packages and package_name != f"mpc_planner_{system}":
            print_value("Package Name", package_name)

            ignore_file = Path(os.path.join(folder, f"{ros_build_name}_IGNORE"))
            if not ignore_file.is_file():
                # Make an empty file with the name of ignore file
                ignore_file.touch()

if __name__ == "__main__":
    main()
