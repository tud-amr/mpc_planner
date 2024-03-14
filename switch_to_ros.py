import sys, os

sys.path.append(os.path.join(sys.path[0], "mpc-planner-solver-generator"))

from pathlib import Path
import shutil

from util.logging import print_value, print_warning

skip_packages = ["mpc-planner-solver-generator", "docs"]


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
    print(file_name)
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
        print("Usage: python switch_to_ros.py <ROS_VERSION>")
        return

    # Access the command line argument
    ros_version = int(sys.argv[1])

    assert ros_version == 1 or ros_version == 2, "ROS version must be 1 or 2"

    print_value("ROS", ros_version)
    subfolders = [f.path for f in os.scandir(sys.path[0]) if f.is_dir() and len(f.name.split(".")) == 1 and f.name not in skip_packages]

    exist, check_files = get_files(subfolders[0], "")
    if not exist:
        cur_ros_mode = 0
    else:
        cur_ros_mode = get_ros_mode(check_files[0])

    if ros_version == cur_ros_mode:
        print(f"Already in ROS{ros_version} mode")
        return

    for folder in subfolders:

        exists, current = get_files(folder, "")
        exists1, ROS1 = get_files(folder, "1")
        exists2, ROS2 = get_files(folder, "2")

        if ros_version == 1:
            if not exists1:
                print_warning(f"ROS files for ROS1 do not exist in {folder}")
                continue

            print(f"Switching to ROS1")
            if cur_ros_mode != 0:
                # Move the current ROS2 files to the version with a 2
                for i in range(len(ROS2)):
                    print(current[i])
                    print(ROS2[i])
                    os.rename(current[i], ROS2[i])

            # Now move the ROS1 files to the current
            for i in range(len(ROS1)):
                print(ROS1[i])
                print(current[i])
                shutil.copyfile(ROS1[i], current[i])

        elif ros_version == 2:
            if not exists2:
                print_warning(f"ROS files for ROS2 do not exist in {folder}")
                continue

            print(f"Switching to ROS2")
            if cur_ros_mode != 0:
                if exists:
                    # Move the current ROS1 files to the version with a 1
                    for i in range(len(ROS1)):
                        print(current[i])
                        print(ROS1[i])
                        os.rename(current[i], ROS1[i])

            # Now move the ROS2 files to the current
            for i in range(len(ROS2)):
                print(ROS2[i])
                print(current[i])
                shutil.copyfile(ROS2[i], current[i])

            # for file in ROS2:
            #     os.remove(file)
            # for file in current:
            #     os.rename(file, file.with_name(file.name.split('.')[0]))


if __name__ == "__main__":
    main()
