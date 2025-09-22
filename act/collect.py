# -- coding: UTF-8
import os
import sys

sys.stdout = open(sys.stdout.fileno(), mode="w", buffering=1)
sys.stderr = open(sys.stderr.fileno(), mode="w", buffering=1)

from pathlib import Path

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
    os.chdir(str(ROOT))


import time
import h5py
import argparse
import rclpy
import cv2
import yaml
import threading
import pyttsx3
import signal
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from copy import deepcopy

from utils.ros_operator import Rate, RosOperator

# 全局退出标志
exit_flag = False


# 信号处理函数
def signal_handler(signum, frame):
    global exit_flag
    print("\nCtrl+C signal detected, exiting safely...")
    exit_flag = True
    # 不要在这里直接退出，让主程序正常清理


# 注册信号处理
signal.signal(signal.SIGINT, signal_handler)

from utils.setup_loader import setup_loader

np.set_printoptions(linewidth=200)

voice_engine = pyttsx3.init()
voice_engine.setProperty("voice", "en")
voice_engine.setProperty("rate", 120)  # 设置语速

voice_lock = threading.Lock()


def load_yaml(yaml_file):
    try:
        with open(yaml_file, "r", encoding="utf-8") as file:
            return yaml.safe_load(file)
    except FileNotFoundError:
        print(f"Error: File not found - {yaml_file}")

        return None
    except yaml.YAMLError as e:
        print(f"Error: Failed to parse YAML file - {e}")

        return None


def voice_process(voice_engine, line):
    with voice_lock:
        voice_engine.say(line)
        voice_engine.runAndWait()
        print(line)

    return


def voice_process_async(voice_engine, line):
    """异步语音播放，不阻塞主线程"""

    def _play_voice():
        with voice_lock:
            voice_engine.say(line)
            voice_engine.runAndWait()
            print(f"🔊 Voice played: {line}")

    # 使用线程池执行语音播放
    voice_executor = ThreadPoolExecutor(max_workers=1)
    voice_executor.submit(_play_voice)
    return voice_executor


def collect_detect(args, start_episode, voice_engine, ros_operator):
    global init_pos

    rate = Rate(args.frame_rate)
    print(f"Preparing to record episode {start_episode}")

    # 倒计时
    for i in range(3, -1, -1):
        print(f"\rwaiting {i} to start recording", end="")

        time.sleep(0.3)

    print(f"\nStart recording program...")

    # 键盘触发录制
    if args.key_collect:
        # input("Enter any key to record :")
        pass
    else:
        init_done = False

        while not init_done and rclpy.ok() and not exit_flag:
            obs_dict = ros_operator.get_observation()
            if obs_dict is None:
                print("synchronization frame")
                rate.sleep()

                continue

            # action = obs_dict['eef']
            action = obs_dict["qpos"]

            # 减少不必要的循环
            with ros_operator.joy_lock:
                triggered = dict(ros_operator.triggered_joys)
                ros_operator.triggered_joys.clear()

            if 0 in triggered:
                init_done = True
                # use the actual position of the robot when the button is pressed as the initial position
                init_pos = action.copy()  # record the actual joint position
                print(
                    f"✅ Button pressed! Initial position recorded: {init_pos[:7]}"
                )  # only show the first 7 joints
                print("🚀 Starting immediate data collection...")
            if 2 in triggered:
                delete_idx = start_episode - 1

                episode_path = os.path.join(args.datasets, f"episode_{delete_idx}.hdf5")
                if os.path.exists(episode_path):
                    os.remove(episode_path)

                    voice_process_async(voice_engine, f"delete {delete_idx}")

            if init_done:
                pass
            rate.sleep()

        # if because Ctrl+C exit, return False
        if exit_flag or not rclpy.ok():
            return False

        return True


def collect_information(args, ros_operator, voice_engine, episode_number):
    timesteps = []
    actions = []
    actions_eef = []
    count = 0
    rate = Rate(args.frame_rate)

    # 用于跟踪回到初始位置的时间
    return_home_start_time = None

    # gripper_idx = [6, 13]
    # gripper_close = -2.1

    # fix the trajectory continuity: use the position recorded when the button is pressed as the first frame
    global init_pos
    print("🎯 Recording first frame using button-press position for continuity...")

    # get the current observation data
    obs_dict = ros_operator.get_observation(ts=0)
    if obs_dict is not None:
        # first frame: use the position recorded when the button is pressed as the action (ensure continuity)
        action = init_pos.copy()
        action_eef = deepcopy(obs_dict["eef"])  # EEF use the current observation

        timesteps.append(obs_dict)
        actions.append(action)
        actions_eef.append(action_eef)
        count = 1

        print(f"✅ Frame 0 - Button position: {action[:7]}")
        print(f"📊 Frame 0 - Current observation: {obs_dict['qpos'][:7]}")

        # calculate the position difference, check if there is a jump
        pos_diff = np.array(obs_dict["qpos"]) - np.array(action)
        max_diff = np.max(np.abs(pos_diff))
        print(f"📈 Position difference: max={max_diff:.4f}, diff={pos_diff[:7]}")

        if max_diff > 0.1:
            print(f"⚠️ WARNING: Large position jump detected ({max_diff:.4f} rad)!")
            print(
                "   This suggests robot moved between button press and data collection."
            )

        # play the voice (asynchronous, not blocking data collection)
        voice_process_async(voice_engine, f"{episode_number % 100}")
        voice_process_async(voice_engine, "go")
    else:
        print("❌ Failed to get initial observation!")
        return timesteps, actions, actions_eef

    while (count < args.max_timesteps) and rclpy.ok():
        obs_dict = ros_operator.get_observation(ts=count)
        action_dict = ros_operator.get_action()

        # synchronization frame detection - reduce the number of dropped frames
        if obs_dict is None or action_dict is None:
            print(f"Synchronization frame {count} - waiting for data...")
            rate.sleep()
            continue

        # get the action and observation value
        action = deepcopy(obs_dict["qpos"])
        action_eef = deepcopy(obs_dict["eef"])

        # gripper action processing
        # for idx in gripper_idx:
        #     action[idx] = 0 if action[idx] > gripper_close else action[idx]
        #     action_eef[idx] = 0 if action_eef[idx] > gripper_close else action_eef[idx]

        # check if it exceeds 2s, and determine whether to stop
        if count > args.frame_rate * 2:
            if all(abs(val - init) <= 0.05 for val, init in zip(action, init_pos)):
                # detected return to initial position, start timing to keep 2 seconds
                if return_home_start_time is None:
                    return_home_start_time = time.time()
                    print(
                        "Detected return to home position, keeping recording for 2 more seconds..."
                    )

                # check if it has kept 2 seconds
                if time.time() - return_home_start_time >= 2.0:
                    print(
                        "2 seconds elapsed after returning home, stopping recording..."
                    )
                    break
            else:
                # if it deviates from the initial position, reset the timer
                if return_home_start_time is not None:
                    return_home_start_time = None

        # trajectory continuity check (from the second frame)
        if count > 0 and len(actions) > 0:
            prev_action = actions[-1]
            action_diff = np.array(action) - np.array(prev_action)
            max_diff = np.max(np.abs(action_diff))

            if max_diff > 0.2:  # exceeds 0.2 radian jump
                print(
                    f"⚠️ Frame {count}: Large trajectory jump detected! Max diff: {max_diff:.4f}"
                )
                print(f"   Previous: {prev_action[:7]}")
                print(f"   Current:  {action[:7]}")
                print(f"   Diff:     {action_diff[:7]}")

        # collect data
        timesteps.append(obs_dict)
        actions.append(action)
        actions_eef.append(action_eef)
        count += 1

        if not rclpy.ok():
            exit(-1)

        rate.sleep()

    print(f"\nlen(timesteps): {len(timesteps)}")
    print(f"len(actions)  : {len(actions)}")

    return timesteps, actions, actions_eef


def compress_and_pad_images(data_dict, camera_names, use_depth, quality=50):
    def compress_and_pad(key_prefix):
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        all_encoded = []

        for cam in camera_names:
            key = f"/observations/{key_prefix}/{cam}"
            encoded_list = []
            for img in data_dict[key]:
                _, enc = cv2.imencode(".jpg", img, encode_param)
                encoded_list.append(enc)
                all_encoded.append(len(enc))
            data_dict[key] = encoded_list

        padded_size = max(all_encoded)

        for cam in camera_names:
            key = f"/observations/{key_prefix}/{cam}"
            padded = [
                np.pad(enc, (0, padded_size - len(enc)), constant_values=0)
                for enc in data_dict[key]
            ]
            data_dict[key] = padded

        return padded_size

    # RGB
    padded_size = compress_and_pad("images")

    # Depth
    padded_size_depth = compress_and_pad("images_depth") if use_depth else 0

    return padded_size, padded_size_depth


def create_and_write_hdf5(
    args, data_dict, dataset_path, data_size, padded_size, padded_size_depth
):
    with h5py.File(dataset_path + ".hdf5", "w", rdcc_nbytes=1024**2 * 2) as root:
        root.attrs["sim"] = False
        root.attrs["task"] = str(args.task)

        obs_dict = root.create_group("observations")
        image = obs_dict.create_group("images")
        if args.use_depth_image:
            depth = obs_dict.create_group("images_depth")

        for cam_name in args.camera_names:
            img_shape = (data_size, padded_size)
            img_chunk = (1, padded_size)
            if args.use_depth_image:
                depth_shape = (data_size, padded_size_depth)
                depth_chunk = (1, padded_size_depth)

            image.create_dataset(cam_name, img_shape, "uint8", chunks=img_chunk)
            if args.use_depth_image:
                depth.create_dataset(cam_name, depth_shape, "uint8", chunks=depth_chunk)

        # create observation and action dicts
        state_dim = 14
        eef_dim = 14
        obs_specs = {
            "qpos": state_dim,
            "eef": eef_dim,
            "qvel": state_dim,
            "effort": state_dim,
        }
        act_specs = {
            "action": state_dim,
            "action_eef": eef_dim,
        }

        for name, dim in obs_specs.items():
            obs_dict.create_dataset(name, (data_size, dim))
        for name, dim in act_specs.items():
            root.create_dataset(name, (data_size, dim))

        for name, arr in data_dict.items():
            root[name][...] = arr


# 保存数据函数
def save_data(
    args,
    timesteps,
    actions,
    actions_eef,
    ros_operator,
    dataset_path,
):
    data_size = len(actions)

    # data dictionary
    data_dict = {
        "/observations/qpos": [],
        "/observations/qvel": [],
        "/observations/effort": [],
        "/observations/eef": [],
        "/action": [],
        "/action_eef": [],
    }

    # init camera dicts
    for cam_name in args.camera_names:
        data_dict[f"/observations/images/{cam_name}"] = []
        if args.use_depth_image:
            data_dict[f"/observations/images_depth/{cam_name}"] = []

    # collect datasets
    while actions and rclpy.ok():
        action = actions.pop(0)  # current action
        action_eef = actions_eef.pop(0)
        ts = timesteps.pop(0)  # current observation

        # append data
        data_dict["/observations/qpos"].append(ts["qpos"])
        data_dict["/observations/qvel"].append(ts["qvel"])
        data_dict["/observations/eef"].append(ts["eef"])
        data_dict["/observations/effort"].append(ts["effort"])
        data_dict["/action"].append(action)
        data_dict["/action_eef"].append(action_eef)

        # camera data
        for cam_name in args.camera_names:
            data_dict[f"/observations/images/{cam_name}"].append(ts["images"][cam_name])
            if args.use_depth_image:
                data_dict[f"/observations/images_depth/{cam_name}"].append(
                    ts["images_depth"][cam_name]
                )

    # compressed image data
    padded_size, padded_size_depth = compress_and_pad_images(
        data_dict, args.camera_names, args.use_depth_image
    )

    t0 = time.time()
    create_and_write_hdf5(
        args, data_dict, dataset_path, data_size, padded_size, padded_size_depth
    )

    voice_process_async(voice_engine, "Save")
    print(f"\033[32m\nSaved in {time.time() - t0:.1f}s: {dataset_path}\033[0m\n")

    return


def main(args):
    setup_loader(ROOT)

    rclpy.init()

    config = load_yaml(args.config)

    ros_operator = RosOperator(args, config, in_collect=True)

    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_operator,), daemon=True)
    spin_thread.start()

    datasets_dir = (
        args.datasets if sys.stdin.isatty() else Path.joinpath(ROOT, args.datasets)
    )

    num_episodes = args.num_episodes if args.episode_idx == -1 else 1
    current_episode = 0 if args.episode_idx == -1 else args.episode_idx

    # find the maximum episode number
    max_episode = -1
    if os.path.exists(datasets_dir):
        for filename in os.listdir(datasets_dir):
            if filename.startswith("episode_") and filename.endswith(".hdf5"):
                try:
                    episode_num = int(filename.split("_")[1].split(".")[0])
                    max_episode = max(max_episode, episode_num)
                except ValueError:
                    continue

    # if the existing episode is found, start from the next maximum number
    if max_episode >= 0:
        current_episode = max_episode + 1

    episode_num = 0
    while episode_num < num_episodes and rclpy.ok() and not exit_flag:
        print(f"Episode {episode_num}")
        if not collect_detect(args, current_episode, voice_engine, ros_operator):
            print("Data collection interrupted by user")
            break

        print(f"Start to record episode {current_episode}")

        (
            timesteps,
            actions,
            actions_eef,
        ) = collect_information(args, ros_operator, voice_engine, current_episode)

        if not os.path.exists(datasets_dir):
            os.makedirs(datasets_dir)

        dataset_path = os.path.join(datasets_dir, "episode_" + str(current_episode))
        threading.Thread(
            target=save_data,
            args=(
                args,
                timesteps,
                actions,
                actions_eef,
                ros_operator,
                dataset_path,
            ),
        ).start()

        episode_num = episode_num + 1
        current_episode = current_episode + 1

    ros_operator.destroy_node()
    rclpy.shutdown()
    spin_thread.join()


def parse_arguments(known=False):
    parser = argparse.ArgumentParser()

    # dataset configuration
    parser.add_argument(
        "--datasets",
        type=str,
        default=Path.joinpath(ROOT, "datasets"),
        help="dataset dir",
    )
    parser.add_argument(
        "--num_episodes", type=int, default=100, help="number of episodes"
    )
    parser.add_argument("--episode_idx", type=int, default=0, help="episode index")
    parser.add_argument("--max_timesteps", type=int, default=1800, help="max timesteps")
    parser.add_argument("--frame_rate", type=int, default=60, help="frame rate")

    # configuration file
    parser.add_argument(
        "--config",
        type=str,
        default=Path.joinpath(ROOT, "data/config.yaml"),
        help="config file",
    )

    # image processing options
    parser.add_argument(
        "--camera_names",
        nargs="+",
        type=str,
        choices=[
            "head",
            "left_wrist",
            "right_wrist",
        ],
        default=["head", "left_wrist", "right_wrist"],
        help="camera names",
    )
    parser.add_argument(
        "--use_depth_image", action="store_true", help="use depth image"
    )

    parser.add_argument(
        "--record",
        choices=["Distance", "Speed"],
        default="Distance",
        help="record data",
    )

    # data collection options
    parser.add_argument("--key_collect", action="store_true", help="use key collect")

    parser.add_argument("--task", type=str, default="", help="task name")

    return parser.parse_known_args()[0] if known else parser.parse_args()


if __name__ == "__main__":
    args = parse_arguments()
    main(args)
