#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import queue
import signal
import threading
import time
from contextlib import nullcontext

import matplotlib
import mujoco
import mujoco.viewer
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import florid_usb

try:
    import pin as pin
except Exception:
    import pinocchio as pin


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
XML_PATH = os.path.join(BASE_DIR, "Ragtime_Willow_description", "urdf", "scene.xml")
URDF_PATH = os.path.join(
    BASE_DIR, "Ragtime_Willow_description", "urdf", "Ragtime_Willow_description.urdf"
)

EE_FRAME_NAME = "link6"
MOCAP_BODY_NAME = "mocap_target"
ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
CAMERA_NAME = "wrist_camera"

KEYFRAME_FILE = os.path.join(BASE_DIR, "teach_keyframes.json")
VIDEO_DIR = os.path.join(BASE_DIR, "videos")
PLOT_DIR = os.path.join(BASE_DIR, "plots")

RENDER_H = 480
RENDER_W = 640
FPS = 30
PLAYBACK_PATH_SAMPLE_RATE = 120.0
PLAYBACK_PATH_RADIUS = 0.004
PLAYBACK_PATH_RGBA = np.array([0.10, 0.85, 0.25, 0.95], dtype=np.float32)
ROBOT_PATH_RADIUS = 0.005
ROBOT_PATH_RGBA = np.array([0.95, 0.30, 0.10, 0.95], dtype=np.float32)
ROBOT_PATH_MIN_DIST = 0.001

REAL_CONTROL_RATE = 500.0
REAL_MOVE_DURATION = 2.0

TAU_LIMIT = np.array([26.90, 26.90, 26.90, 6.90, 6.90, 6.90], dtype=np.float64)
# This variant uses the actuator's MIT PD loop plus gravity feedforward only.
MIT_KP = np.array([100.0, 100.0, 100.0, 50.0, 50.0, 50.0], dtype=np.float64)
MIT_KD = np.array([5.0, 5.0, 2.0, 2.0, 2.0, 2.0], dtype=np.float64)
VEL_FILTER_ALPHA = 1.0

quit_requested = False
shutdown_requested = threading.Event()
signal_count = 0

_cv2_module = None


def get_cv2():
    global _cv2_module
    if _cv2_module is None:
        import cv2 as _cv2

        _cv2_module = _cv2
    return _cv2_module


def request_shutdown(reason="shutdown"):
    global quit_requested
    quit_requested = True
    shutdown_requested.set()
    stop_cmd_thread.set()
    robot_thread_stop.set()
    try:
        cmd_queue.put("quit")
    except Exception:
        pass
    print(f"\n[shutdown] {reason}")


def app_signal_handler(signum, _frame):
    global signal_count
    signal_count += 1
    if signal_count == 1:
        request_shutdown(f"signal {signum}")
        return
    raise KeyboardInterrupt


signal.signal(signal.SIGINT, app_signal_handler)
signal.signal(signal.SIGTERM, app_signal_handler)

cmd_queue: queue.Queue[str] = queue.Queue()
stop_cmd_thread = threading.Event()


def command_thread_func():
    print("\n========== Teach Camera Arm Commands ==========")
    print("save NAME                 保存当前示教点")
    print("list                      查看所有点")
    print("goto NAME [T]             仿真平滑移动到某个点，默认 T=2.0s")
    print("play NAME1 NAME2 ...      按顺序回放关键帧")
    print("play_all                  按保存顺序回放全部关键帧")
    print("delete NAME               删除点")
    print("clear                     清空所有点")
    print("speed S                   设置速度倍率，例如 speed 0.5 / speed 2.0")
    print("record_on [filename.mp4]  开始录制 wrist_camera")
    print("record_off                停止录制")
    print("save_json                 保存关键帧到 json")
    print("load_json                 从 json 加载关键帧")
    print("plot_traj [T]             生成当前轨迹曲线图，默认每段 T=2.0s")
    print("mode teach                回到 mocap 示教模式")
    print("robot_connect [device]    连接真机，默认 /dev/ttyACM0")
    print("robot_disconnect          断开真机")
    print("robot_status              查看真机状态")
    print("robot_sync on|off         播放时真机同步 MIT_PD+gravity 跟踪")
    print("robot_goto NAME [T]       真机通过 MIT_PD+gravity 轨迹移动到某个示教点")
    print("robot_goto_sim [T]        真机通过 MIT_PD+gravity 轨迹移动到当前仿真姿态")
    print("help                      显示命令")
    print("quit                      退出")
    print("==============================================\n")

    while not stop_cmd_thread.is_set():
        try:
            cmd = input("cmd> ").strip()
            if cmd:
                cmd_queue.put(cmd)
        except EOFError:
            break
        except KeyboardInterrupt:
            cmd_queue.put("quit")
            break


threading.Thread(target=command_thread_func, daemon=True).start()


model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)

cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, CAMERA_NAME)
if cam_id < 0:
    raise RuntimeError(f"Camera '{CAMERA_NAME}' not found in XML.")

renderer = mujoco.Renderer(model, height=RENDER_H, width=RENDER_W)

mocap_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, MOCAP_BODY_NAME)
if mocap_body_id < 0:
    raise RuntimeError(f"Body '{MOCAP_BODY_NAME}' not found.")
mocap_id = int(model.body_mocapid[mocap_body_id])
if mocap_id == -1:
    raise RuntimeError("mocap_target has no mocap id")

mj_qadr: dict[str, int] = {}
for jn in ARM_JOINT_NAMES + ["joint7", "joint8"]:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
    if jid >= 0:
        mj_qadr[jn] = int(model.jnt_qposadr[jid])

arm_qadr = np.array([mj_qadr[j] for j in ARM_JOINT_NAMES], dtype=np.int32)

pin_model = pin.buildModelFromUrdf(URDF_PATH)
pin_data = pin_model.createData()


def pin_joint_index(joint, field):
    value = getattr(joint, field)
    return int(value() if callable(value) else value)


def valid_pin_joint_id(model_, jid, name):
    return 0 < jid < len(model_.names) and str(model_.names[jid]) == name


ee_fid = pin_model.getFrameId(EE_FRAME_NAME)
if ee_fid >= len(pin_model.frames):
    raise RuntimeError(f"Pinocchio frame '{EE_FRAME_NAME}' not found in URDF.")

pin_q_index: dict[str, int] = {}
pin_v_index: dict[str, int] = {}
for jn in ARM_JOINT_NAMES + ["joint7", "joint8"]:
    jid = pin_model.getJointId(jn)
    if valid_pin_joint_id(pin_model, jid, jn):
        pin_q_index[jn] = pin_joint_index(pin_model.joints[jid], "idx_q")
        pin_v_index[jn] = pin_joint_index(pin_model.joints[jid], "idx_v")

arm_vidx = np.array([pin_v_index[j] for j in ARM_JOINT_NAMES], dtype=np.int32)


def quat_wxyz_to_R(q):
    w, x, y, z = q
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = q / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def pin_state_from_mj():
    q = pin.neutral(pin_model)
    for jn, iq in pin_q_index.items():
        if jn in mj_qadr:
            q[iq] = float(data.qpos[mj_qadr[jn]])
    return q


def get_current_ee_pose():
    q_pin = pin_state_from_mj()
    pin.forwardKinematics(pin_model, pin_data, q_pin)
    pin.updateFramePlacements(pin_model, pin_data)
    oMe = pin_data.oMf[ee_fid]
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, oMe.rotation.reshape(-1))
    return oMe.translation.copy(), quat.copy()


def set_mocap_to_current_ee():
    ee_pos, ee_quat = get_current_ee_pose()
    data.mocap_pos[mocap_id] = ee_pos
    data.mocap_quat[mocap_id] = ee_quat


def build_joint_points(q_targets, q_now=None):
    if q_now is None:
        q_now = data.qpos[arm_qadr].copy()
    return [np.asarray(q_now, dtype=np.float64).copy()] + [
        np.asarray(q, dtype=np.float64).copy() for q in q_targets
    ]


def build_joint_spline(points, duration_per_segment):
    if len(points) < 2:
        raise RuntimeError("need at least two spline points")

    seg_T = max(0.1, float(duration_per_segment) / max(speed_scale, 1e-6))
    velocities = []
    accelerations = []
    last = len(points) - 1
    for i in range(len(points)):
        if i == 0 or i == last:
            velocities.append(np.zeros_like(points[i]))
            accelerations.append(np.zeros_like(points[i]))
        else:
            velocities.append(0.5 * (points[i + 1] - points[i - 1]) / seg_T)
            accelerations.append(
                (points[i + 1] - 2.0 * points[i] + points[i - 1]) / (seg_T**2)
            )

    segments = []
    for i in range(last):
        segments.append(
            (
                points[i],
                points[i + 1],
                velocities[i],
                velocities[i + 1],
                accelerations[i],
                accelerations[i + 1],
            )
        )
    return segments, velocities, accelerations, seg_T


def eval_joint_spline_segment(segment, tau, seg_T):
    q0, q1, v0, v1, a0, a1 = segment
    u = np.clip(tau / seg_T, 0.0, 1.0)
    u2 = u * u
    u3 = u2 * u
    u4 = u3 * u
    u5 = u4 * u

    h0 = 1.0 - 10.0 * u3 + 15.0 * u4 - 6.0 * u5
    h1 = u - 6.0 * u3 + 8.0 * u4 - 3.0 * u5
    h2 = 0.5 * (u2 - 3.0 * u3 + 3.0 * u4 - u5)
    h3 = 10.0 * u3 - 15.0 * u4 + 6.0 * u5
    h4 = -4.0 * u3 + 7.0 * u4 - 3.0 * u5
    h5 = 0.5 * (u3 - 2.0 * u4 + u5)

    dh0 = -30.0 * u2 + 60.0 * u3 - 30.0 * u4
    dh1 = 1.0 - 18.0 * u2 + 32.0 * u3 - 15.0 * u4
    dh2 = u - 4.5 * u2 + 6.0 * u3 - 2.5 * u4
    dh3 = 30.0 * u2 - 60.0 * u3 + 30.0 * u4
    dh4 = -12.0 * u2 + 28.0 * u3 - 15.0 * u4
    dh5 = 1.5 * u2 - 4.0 * u3 + 2.5 * u4

    ddh0 = -60.0 * u + 180.0 * u2 - 120.0 * u3
    ddh1 = -36.0 * u + 96.0 * u2 - 60.0 * u3
    ddh2 = 1.0 - 9.0 * u + 18.0 * u2 - 10.0 * u3
    ddh3 = 60.0 * u - 180.0 * u2 + 120.0 * u3
    ddh4 = -24.0 * u + 84.0 * u2 - 60.0 * u3
    ddh5 = 3.0 * u - 12.0 * u2 + 10.0 * u3

    q = h0 * q0 + h1 * seg_T * v0 + h2 * (seg_T**2) * a0 + h3 * q1 + h4 * seg_T * v1 + h5 * (
        seg_T**2
    ) * a1
    dq = (
        dh0 * q0
        + dh1 * seg_T * v0
        + dh2 * (seg_T**2) * a0
        + dh3 * q1
        + dh4 * seg_T * v1
        + dh5 * (seg_T**2) * a1
    ) / seg_T
    ddq = (
        ddh0 * q0
        + ddh1 * seg_T * v0
        + ddh2 * (seg_T**2) * a0
        + ddh3 * q1
        + ddh4 * seg_T * v1
        + ddh5 * (seg_T**2) * a1
    ) / (seg_T**2)
    return q, dq, ddq


def sample_joint_trajectory(q_targets, duration_per_segment=2.0, sample_rate=200.0, q_now=None):
    points = build_joint_points(q_targets, q_now=q_now)
    segments, _, _, seg_T = build_joint_spline(points, duration_per_segment)
    dt = 1.0 / sample_rate
    t_list = []
    q_list = []
    dq_list = []
    ddq_list = []
    seg_idx_list = []
    t_global = 0.0
    for seg_idx, segment in enumerate(segments):
        steps = max(2, int(np.ceil(seg_T / dt)) + 1)
        local_times = np.linspace(0.0, seg_T, steps)
        if seg_idx > 0:
            local_times = local_times[1:]
        for tau in local_times:
            q, dq, ddq = eval_joint_spline_segment(segment, tau, seg_T)
            t_list.append(t_global + tau)
            q_list.append(q)
            dq_list.append(dq)
            ddq_list.append(ddq)
            seg_idx_list.append(seg_idx)
        t_global += seg_T
    return {
        "time": np.asarray(t_list),
        "q": np.asarray(q_list),
        "dq": np.asarray(dq_list),
        "ddq": np.asarray(ddq_list),
        "segment_index": np.asarray(seg_idx_list),
        "duration_per_segment": seg_T,
        "sample_rate": sample_rate,
    }


def sample_ee_positions(q_samples):
    preview_pin_data = pin_model.createData()
    ee_positions = []
    for q_arm in q_samples:
        q_pin = pin.neutral(pin_model)
        for idx, joint_name in enumerate(ARM_JOINT_NAMES):
            q_pin[pin_q_index[joint_name]] = q_arm[idx]
        pin.forwardKinematics(pin_model, preview_pin_data, q_pin)
        pin.updateFramePlacements(pin_model, preview_pin_data)
        ee_positions.append(preview_pin_data.oMf[ee_fid].translation.copy())
    return np.asarray(ee_positions, dtype=np.float64)


def cache_playback_path(q_targets, duration_per_segment=2.0, q_now=None):
    global playback_path_points
    if len(q_targets) == 0:
        playback_path_points = np.empty((0, 3), dtype=np.float64)
        return
    result = sample_joint_trajectory(
        q_targets,
        duration_per_segment=duration_per_segment,
        sample_rate=PLAYBACK_PATH_SAMPLE_RATE,
        q_now=q_now,
    )
    playback_path_points = sample_ee_positions(result["q"])


def reset_robot_actual_path(q_start=None):
    global robot_actual_path_points
    if q_start is None:
        robot_actual_path_points = np.empty((0, 3), dtype=np.float64)
        return
    robot_actual_path_points = sample_ee_positions(np.asarray([q_start], dtype=np.float64))


def append_robot_actual_path(q_robot):
    global robot_actual_path_points
    ee_pos = sample_ee_positions(np.asarray([q_robot], dtype=np.float64))[0]
    if len(robot_actual_path_points) == 0:
        robot_actual_path_points = np.asarray([ee_pos], dtype=np.float64)
        return
    if np.linalg.norm(ee_pos - robot_actual_path_points[-1]) < ROBOT_PATH_MIN_DIST:
        return
    robot_actual_path_points = np.vstack((robot_actual_path_points, ee_pos))


def append_path_geoms(scn, points, radius, rgba):
    if len(points) < 2 or scn.ngeom >= scn.maxgeom:
        return

    max_segments = max(0, int(scn.maxgeom) - int(scn.ngeom))
    if max_segments <= 0:
        return

    draw_points = points
    if len(draw_points) - 1 > max_segments:
        keep = np.linspace(0, len(draw_points) - 1, max_segments + 1, dtype=np.int32)
        draw_points = draw_points[keep]

    for p0, p1 in zip(draw_points[:-1], draw_points[1:]):
        if scn.ngeom >= scn.maxgeom:
            break
        geom = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.zeros(9, dtype=np.float64),
            rgba,
        )
        mujoco.mjv_connector(
            geom,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            radius,
            np.asarray(p0, dtype=np.float64),
            np.asarray(p1, dtype=np.float64),
        )
        scn.ngeom += 1


def draw_playback_path(viewer):
    scn = viewer.user_scn
    scn.ngeom = 0
    append_path_geoms(scn, playback_path_points, PLAYBACK_PATH_RADIUS, PLAYBACK_PATH_RGBA)
    append_path_geoms(scn, robot_actual_path_points, ROBOT_PATH_RADIUS, ROBOT_PATH_RGBA)


def save_trajectory_plot(q_targets, names, duration_per_segment=2.0):
    os.makedirs(PLOT_DIR, exist_ok=True)
    result = sample_joint_trajectory(q_targets, duration_per_segment=duration_per_segment)
    t = result["time"]
    q = result["q"]
    dq = result["dq"]
    ddq = result["ddq"]

    stamp = time.strftime("%Y%m%d_%H%M%S")
    base = f"teach_traj_{stamp}"
    png_path = os.path.join(PLOT_DIR, f"{base}.png")
    json_path = os.path.join(PLOT_DIR, f"{base}.json")

    fig, axes = plt.subplots(
        len(ARM_JOINT_NAMES), 3, figsize=(18, 2.7 * len(ARM_JOINT_NAMES)), sharex="col"
    )
    if len(ARM_JOINT_NAMES) == 1:
        axes = np.asarray([axes])

    for i, joint_name in enumerate(ARM_JOINT_NAMES):
        axes[i, 0].plot(t, q[:, i], linewidth=1.3)
        axes[i, 0].set_ylabel(f"{joint_name}\nrad")
        axes[i, 0].grid(True, linestyle="--", alpha=0.35)
        axes[i, 1].plot(t, dq[:, i], linewidth=1.1)
        axes[i, 1].grid(True, linestyle="--", alpha=0.35)
        axes[i, 2].plot(t, ddq[:, i], linewidth=1.1)
        axes[i, 2].grid(True, linestyle="--", alpha=0.35)

    axes[0, 0].set_title("q_des")
    axes[0, 1].set_title("dq_des")
    axes[0, 2].set_title("ddq_des")
    axes[-1, 0].set_xlabel("Time (s)")
    axes[-1, 1].set_xlabel("Time (s)")
    axes[-1, 2].set_xlabel("Time (s)")
    fig.suptitle("Teach Trajectory Preview: " + " -> ".join(names))
    fig.tight_layout()
    fig.savefig(png_path, dpi=180, bbox_inches="tight")
    plt.close(fig)

    payload = {
        "joint_names": ARM_JOINT_NAMES,
        "keyframe_names": names,
        "duration_per_segment": float(result["duration_per_segment"]),
        "sample_rate": float(result["sample_rate"]),
        "time": t.tolist(),
        "q": q.tolist(),
        "dq": dq.tolist(),
        "ddq": ddq.tolist(),
    }
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    print(f"[plot] saved figure: {png_path}")
    print(f"[plot] saved data:   {json_path}")
    return png_path, json_path


def init_robot_playback_logs(names, duration_per_segment):
    return {
        "keyframe_names": list(names),
        "duration_per_segment": float(duration_per_segment),
        "time": [],
        "dt": [],
        "desired_pos": [],
        "actual_pos": [],
        "desired_vel": [],
        "actual_vel": [],
        "actual_vel_filt": [],
        "desired_acc": [],
        "commanded_torque": [],
        "feedforward_torque": [],
        "feedback_torque": [],
        "gravity_torque": [],
    }


def save_robot_playback_logs():
    global robot_playback_logs
    if not robot_playback_logs or not robot_playback_logs["time"]:
        return

    os.makedirs(PLOT_DIR, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    base = f"robot_replay_{stamp}"
    png_path = os.path.join(PLOT_DIR, f"{base}.png")
    json_path = os.path.join(PLOT_DIR, f"{base}.json")

    t = np.asarray(robot_playback_logs["time"], dtype=np.float64)
    q_des = np.asarray(robot_playback_logs["desired_pos"], dtype=np.float64)
    q_act = np.asarray(robot_playback_logs["actual_pos"], dtype=np.float64)
    tau_cmd = np.asarray(robot_playback_logs["commanded_torque"], dtype=np.float64)
    err = q_act - q_des

    fig, axes = plt.subplots(
        len(ARM_JOINT_NAMES), 3, figsize=(20, 2.8 * len(ARM_JOINT_NAMES)), sharex="col"
    )
    if len(ARM_JOINT_NAMES) == 1:
        axes = np.asarray([axes])

    for i, joint_name in enumerate(ARM_JOINT_NAMES):
        axes[i, 0].plot(t, q_des[:, i], label="desired", linewidth=1.5, color="black")
        axes[i, 0].plot(t, q_act[:, i], label="actual", linewidth=1.0)
        axes[i, 0].set_ylabel(f"{joint_name}\nrad")
        axes[i, 0].grid(True, linestyle="--", alpha=0.35)
        axes[i, 0].legend(loc="upper right")
        axes[i, 1].plot(t, err[:, i], linewidth=1.0)
        axes[i, 1].grid(True, linestyle="--", alpha=0.35)
        axes[i, 2].plot(t, tau_cmd[:, i], linewidth=1.0)
        axes[i, 2].grid(True, linestyle="--", alpha=0.35)

    axes[0, 0].set_title("Desired vs Actual")
    axes[0, 1].set_title("Tracking Error")
    axes[0, 2].set_title("Commanded Torque")
    axes[-1, 0].set_xlabel("Time (s)")
    axes[-1, 1].set_xlabel("Time (s)")
    axes[-1, 2].set_xlabel("Time (s)")
    fig.suptitle("Robot Replay Tracking: " + " -> ".join(robot_playback_logs["keyframe_names"]))
    fig.tight_layout()
    fig.savefig(png_path, dpi=180, bbox_inches="tight")
    plt.close(fig)

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(robot_playback_logs, f, ensure_ascii=False, indent=2)

    dt = np.asarray(robot_playback_logs.get("dt", []), dtype=np.float64)
    if len(dt) > 0:
        print(
            "[robot] replay dt stats: "
            f"mean={np.mean(dt):.6f}s  std={np.std(dt):.6f}s  "
            f"min={np.min(dt):.6f}s  max={np.max(dt):.6f}s"
        )
    print(f"[robot] replay figure saved: {png_path}")
    print(f"[robot] replay data saved:   {json_path}")
    robot_playback_logs = None


def pin_qv_from_robot(q_arm, dq_arm):
    q_pin = pin.neutral(pin_model)
    v_pin = np.zeros(pin_model.nv, dtype=np.float64)
    for idx, jn in enumerate(ARM_JOINT_NAMES):
        q_pin[pin_q_index[jn]] = q_arm[idx]
        v_pin[pin_v_index[jn]] = dq_arm[idx]
    return q_pin, v_pin


def compute_gravity_torque(q_arm):
    q_pin, _ = pin_qv_from_robot(q_arm, np.zeros(JOINT_COUNT, dtype=np.float64))
    tau = pin.computeGeneralizedGravity(pin_model, pin_data, q_pin)
    return np.asarray(tau[arm_vidx], dtype=np.float64)

JOINT_COUNT = 6
MODE_TEACH = "teach"
MODE_PLAYBACK = "playback"
mode = MODE_TEACH

keyframes: dict[str, dict] = {}
keyframe_order: list[str] = []
speed_scale = 1.0

playback_active = False
playback_segments = []
playback_seg_idx = 0
playback_t0 = 0.0
playback_duration = 2.0
playback_after_mode = MODE_TEACH
playback_path_points = np.empty((0, 3), dtype=np.float64)
robot_actual_path_points = np.empty((0, 3), dtype=np.float64)

robot_arm = None
robot_session_started = False
robot_device = None
robot_sync_enabled = False
robot_hold_enabled = False
robot_hold_q = None
robot_thread = None
robot_thread_stop = threading.Event()
robot_playback_lock = threading.Lock()
robot_playback_active = False
robot_playback_segments = []
robot_playback_seg_idx = 0
robot_playback_t0 = 0.0
robot_playback_duration = 2.0
robot_playback_names = []
robot_playback_log_t0 = None
robot_playback_logs = None
robot_vel_filt = None
robot_visual_state_lock = threading.Lock()
robot_visual_q = None
robot_playback_start_q = None
robot_last_result = None
robot_timing_last_print = 0.0
robot_timing_window_count = 0
robot_timing_window_total_ms = 0.0
robot_timing_window_status_ms = 0.0
robot_timing_window_compute_ms = 0.0
robot_timing_window_send_ms = 0.0
robot_timing_last_step_ms = 0.0

recording = False
video_writer = None
video_path = None

sub_iters = 6
alpha = 0.18
dls = 8e-2
dq_joint_limit = 0.06
dq_norm_limit = 0.12
beta = 0.35
dq_filt = np.zeros(6)

mujoco.mj_forward(model, data)
set_mocap_to_current_ee()


def robot_connected():
    return robot_arm is not None and robot_arm.is_connected() and robot_session_started


def get_sim_arm_q():
    return data.qpos[arm_qadr].copy()


def robot_get_status():
    if not robot_connected():
        raise RuntimeError("robot not connected")
    status = robot_arm.get_arm_status()
    mode = int(status["mode"])
    q = np.asarray(status["q"], dtype=np.float64)
    dq = np.asarray(status["dq"], dtype=np.float64)
    tau = np.asarray(status["tau"], dtype=np.float64)
    return mode, q, dq, tau


def publish_robot_visual_state(q):
    global robot_visual_q
    with robot_visual_state_lock:
        robot_visual_q = np.asarray(q, dtype=np.float64).copy()


def consume_robot_visual_state():
    with robot_visual_state_lock:
        if robot_visual_q is None:
            return None
        return robot_visual_q.copy()


def connect_robot(device=None):
    global robot_arm, robot_session_started, robot_thread, robot_device
    if robot_connected():
        print("[robot] already connected.")
        return

    cfg = florid_usb.Config()
    cfg.device = device or "/dev/ttyACM0"
    robot_device = cfg.device
    robot_arm = florid_usb.Arm(cfg)
    if not robot_arm.connect():
        robot_arm = None
        raise RuntimeError(f"failed to connect {cfg.device}")
    if not robot_arm.start_session(timeout=1.0):
        robot_arm.disconnect()
        robot_arm = None
        raise RuntimeError("start_session failed")
    robot_session_started = True
    deadline = time.time() + 1.0
    q_init = None
    while time.time() < deadline:
        if robot_arm.is_connected():
            status = robot_arm.get_arm_status()
            q_init = np.asarray(status["q"], dtype=np.float64)
            break
        time.sleep(0.01)
    if q_init is None:
        robot_arm.stop_session(1.0)
        robot_arm.disconnect()
        robot_arm = None
        robot_session_started = False
        raise RuntimeError("robot connected but no arm status received within 1.0s")
    publish_robot_visual_state(q_init)
    print(f"[robot] connected on {cfg.device}")

    if robot_thread is None or not robot_thread.is_alive():
        robot_thread_stop.clear()
        robot_thread = threading.Thread(target=robot_control_loop, daemon=True)
        robot_thread.start()


def disconnect_robot():
    global robot_arm, robot_session_started, robot_sync_enabled, robot_hold_enabled
    global robot_playback_active, robot_playback_segments, robot_device
    if robot_arm is None:
        print("[robot] not connected.")
        return

    robot_sync_enabled = False
    robot_hold_enabled = False
    with robot_playback_lock:
        robot_playback_active = False
        robot_playback_segments = []

    try:
        _, q, _, _ = robot_get_status() if robot_connected() else (None, None, None, None)
        if q is not None:
            zeros = np.zeros(JOINT_COUNT, dtype=np.float32)
            robot_arm.send_mit_command(
                np.asarray(q, dtype=np.float32), zeros, zeros, zeros, zeros
            )
    except Exception:
        pass

    try:
        if robot_session_started:
            robot_arm.stop_session(1.0)
    finally:
        robot_arm.disconnect()
        robot_arm = None
        robot_session_started = False
        robot_device = None
        publish_robot_visual_state(np.zeros(JOINT_COUNT, dtype=np.float64))
        reset_robot_actual_path()

    print("[robot] disconnected.")


def print_robot_status():
    if not robot_connected():
        print("[robot] disconnected.")
        return
    mode, q, dq, tau = robot_get_status()
    print(f"[robot] device={robot_device}")
    print(f"[robot] mode={mode}")
    print(f"[robot] q={np.round(q, 4)}")
    print(f"[robot] dq={np.round(dq, 4)}")
    print(f"[robot] tau={np.round(tau, 4)}")
    print(f"[robot] algo_step_ms={robot_timing_last_step_ms:.3f}")
    print(f"[robot] sync={'on' if robot_sync_enabled else 'off'} hold={'on' if robot_hold_enabled else 'off'}")


def set_robot_hold_current():
    global robot_hold_enabled, robot_hold_q
    if not robot_connected():
        return
    _, q, _, _ = robot_get_status()
    robot_hold_q = np.asarray(q, dtype=np.float64)
    robot_hold_enabled = True


def start_robot_playback(q_targets, duration_per_segment, names=None):
    global robot_hold_enabled, robot_hold_q
    global robot_playback_active, robot_playback_segments, robot_playback_seg_idx
    global robot_playback_t0, robot_playback_duration, robot_playback_names
    global robot_playback_log_t0, robot_playback_logs, robot_vel_filt
    global robot_playback_start_q, robot_last_result

    if not robot_connected():
        return

    _, q_now, _, _ = robot_get_status()
    reset_robot_actual_path(q_now)
    points = build_joint_points(q_targets, q_now=q_now)
    segments, _, _, seg_T = build_joint_spline(points, duration_per_segment)
    if not segments:
        return

    with robot_playback_lock:
        robot_playback_segments = segments
        robot_playback_seg_idx = 0
        robot_playback_t0 = time.time()
        robot_playback_duration = seg_T
        robot_playback_active = True
        robot_hold_enabled = False
        robot_hold_q = points[-1].copy()
        robot_playback_names = list(names or [])
        robot_playback_log_t0 = time.time()
        robot_playback_logs = init_robot_playback_logs(robot_playback_names, seg_T)
        robot_vel_filt = None
        robot_playback_start_q = q_now.copy()
        robot_last_result = None

    print(f"[robot] MIT playback armed: {len(segments)} segment(s)")


def stop_robot_playback():
    global robot_playback_active
    with robot_playback_lock:
        robot_playback_active = False
    if robot_connected():
        set_robot_hold_current()


def sample_robot_playback(now):
    global robot_playback_active, robot_playback_seg_idx, robot_playback_t0
    global robot_hold_enabled, robot_hold_q
    with robot_playback_lock:
        if not robot_playback_active or not robot_playback_segments:
            return None

        tau = now - robot_playback_t0
        q_des, dq_des, ddq_des = eval_joint_spline_segment(
            robot_playback_segments[robot_playback_seg_idx], tau, robot_playback_duration
        )

        if tau >= robot_playback_duration:
            robot_playback_seg_idx += 1
            if robot_playback_seg_idx >= len(robot_playback_segments):
                robot_playback_active = False
                robot_hold_enabled = True
                robot_hold_q = q_des.copy()
                print("[robot] MIT playback done.")
            else:
                robot_playback_t0 = now

        return q_des, dq_des, ddq_des


def robot_pd_gravity_step(q_des, dq_des, _ddq_des):
    global robot_vel_filt
    status_t0 = time.perf_counter()
    mode, actual_q, actual_dq_raw, actual_tau = robot_get_status()
    status_ms = (time.perf_counter() - status_t0) * 1000.0

    compute_t0 = time.perf_counter()
    publish_robot_visual_state(actual_q)
    if robot_vel_filt is None:
        robot_vel_filt = actual_dq_raw.copy()
    else:
        robot_vel_filt = (1.0 - VEL_FILTER_ALPHA) * robot_vel_filt + VEL_FILTER_ALPHA * actual_dq_raw
    actual_dq = robot_vel_filt.copy()

    gravity_torque = compute_gravity_torque(actual_q)
    feedforward_torque = gravity_torque.copy()
    feedback_torque = np.zeros(JOINT_COUNT, dtype=np.float64)
    commanded_torque = np.clip(feedforward_torque, -TAU_LIMIT, TAU_LIMIT)
    compute_ms = (time.perf_counter() - compute_t0) * 1000.0

    send_t0 = time.perf_counter()
    robot_arm.send_mit_command(
        np.asarray(q_des, dtype=np.float32),
        np.asarray(dq_des, dtype=np.float32),
        np.asarray(commanded_torque, dtype=np.float32),
        np.asarray(MIT_KP, dtype=np.float32),
        np.asarray(MIT_KD, dtype=np.float32),
    )
    send_ms = (time.perf_counter() - send_t0) * 1000.0
    step_ms = status_ms + compute_ms + send_ms

    return {
        "controller": "mit_pd_gravity",
        "mode": mode,
        "actual_q": actual_q,
        "actual_dq": actual_dq,
        "actual_tau": actual_tau,
        "gravity_torque": gravity_torque,
        "feedforward_torque": feedforward_torque,
        "feedback_torque": feedback_torque,
        "commanded_torque": commanded_torque,
        "timing_ms": {
            "status": status_ms,
            "compute": compute_ms,
            "send": send_ms,
            "total": step_ms,
        },
    }


def move_robot_mit(q_target, duration=REAL_MOVE_DURATION):
    if not robot_connected():
        raise RuntimeError("robot not connected")
    start_robot_playback([q_target], duration_per_segment=duration, names=["robot_move"])
    while robot_connected():
        with robot_playback_lock:
            active = robot_playback_active
        if not active:
            break
        time.sleep(0.02)
    if robot_last_result is not None and robot_playback_start_q is not None:
        final_q = robot_last_result["actual_q"]
        motion_norm = float(np.linalg.norm(final_q - robot_playback_start_q))
        final_err = q_target - final_q
        max_tau = float(np.max(np.abs(robot_last_result["commanded_torque"])))
        print(
            "[robot] move summary: "
            f"motion_norm={motion_norm:.4f} rad, "
            f"final_err={np.round(final_err, 4)}, "
            f"max|tau_cmd|={max_tau:.3f}, "
            f"mode={robot_last_result['mode']}"
        )
        if motion_norm < 0.02:
            print("[robot] warning: host trajectory finished but actual joint motion was very small")
    set_robot_hold_current()


def robot_control_loop():
    global robot_playback_logs, robot_last_result
    global robot_timing_last_print, robot_timing_window_count
    global robot_timing_window_total_ms, robot_timing_window_status_ms
    global robot_timing_window_compute_ms, robot_timing_window_send_ms
    global robot_timing_last_step_ms
    period = 1.0 / REAL_CONTROL_RATE
    next_tick = time.time()
    while not robot_thread_stop.is_set():
        if not robot_connected():
            time.sleep(0.05)
            next_tick = time.time()
            continue

        now = time.time()
        if now < next_tick:
            time.sleep(min(period, next_tick - now))
            continue
        next_tick += period

        try:
            cmd = sample_robot_playback(now) if robot_sync_enabled or robot_playback_active else None
            if cmd is not None:
                result = robot_pd_gravity_step(*cmd)
                robot_last_result = result
                timing = result.get("timing_ms", {})
                robot_timing_last_step_ms = float(timing.get("total", 0.0))
                robot_timing_window_count += 1
                robot_timing_window_total_ms += float(timing.get("total", 0.0))
                robot_timing_window_status_ms += float(timing.get("status", 0.0))
                robot_timing_window_compute_ms += float(timing.get("compute", 0.0))
                robot_timing_window_send_ms += float(timing.get("send", 0.0))
                if now - robot_timing_last_print >= 1.0 and robot_timing_window_count > 0:
                    count = float(robot_timing_window_count)
                    print(
                        "[robot] algo timing ms | "
                        f"status={robot_timing_window_status_ms / count:.3f} "
                        f"compute={robot_timing_window_compute_ms / count:.3f} "
                        f"send={robot_timing_window_send_ms / count:.3f} "
                        f"total={robot_timing_window_total_ms / count:.3f}"
                    )
                    robot_timing_last_print = now
                    robot_timing_window_count = 0
                    robot_timing_window_total_ms = 0.0
                    robot_timing_window_status_ms = 0.0
                    robot_timing_window_compute_ms = 0.0
                    robot_timing_window_send_ms = 0.0
                if robot_playback_logs is not None and robot_playback_log_t0 is not None:
                    cur_t = now - robot_playback_log_t0
                    if robot_playback_logs["time"]:
                        robot_playback_logs["dt"].append(cur_t - robot_playback_logs["time"][-1])
                    robot_playback_logs["time"].append(cur_t)
                    robot_playback_logs["desired_pos"].append(cmd[0].tolist())
                    robot_playback_logs["actual_pos"].append(result["actual_q"].tolist())
                    robot_playback_logs["desired_vel"].append(cmd[1].tolist())
                    robot_playback_logs["actual_vel"].append(result["actual_dq"].tolist())
                    robot_playback_logs["actual_vel_filt"].append(result["actual_dq"].tolist())
                    robot_playback_logs["desired_acc"].append(cmd[2].tolist())
                    robot_playback_logs["commanded_torque"].append(result["commanded_torque"].tolist())
                    robot_playback_logs["feedforward_torque"].append(result["feedforward_torque"].tolist())
                    robot_playback_logs["feedback_torque"].append(result["feedback_torque"].tolist())
                    robot_playback_logs["gravity_torque"].append(result["gravity_torque"].tolist())
                continue

            if robot_hold_enabled and robot_hold_q is not None:
                zeros = np.zeros_like(robot_hold_q)
                robot_pd_gravity_step(robot_hold_q, zeros, zeros)
        except Exception as exc:
            print(f"[robot] control loop error: {exc}")
            time.sleep(0.1)


def save_keyframes_to_json(path=KEYFRAME_FILE):
    payload = {"version": 1, "joint_names": ARM_JOINT_NAMES, "keyframes": keyframes, "order": keyframe_order}
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
    print(f"[json] saved to {path}")


def load_keyframes_from_json(path=KEYFRAME_FILE):
    global keyframes, keyframe_order
    if not os.path.exists(path):
        print(f"[json] file not found: {path}")
        return
    with open(path, "r", encoding="utf-8") as f:
        payload = json.load(f)
    keyframes = payload.get("keyframes", {})
    keyframe_order = payload.get("order", list(keyframes.keys()))
    print(f"[json] loaded {len(keyframe_order)} keyframes from {path}")


def make_video_writer(filename):
    cv2 = get_cv2()
    os.makedirs(VIDEO_DIR, exist_ok=True)
    if not filename:
        filename = time.strftime("wrist_camera_%Y%m%d_%H%M%S.mp4")
    if not filename.lower().endswith((".mp4", ".avi")):
        filename += ".mp4"
    path = os.path.join(VIDEO_DIR, filename)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(path, fourcc, FPS, (RENDER_W, RENDER_H))
    if not writer.isOpened():
        raise RuntimeError(f"Cannot open video writer: {path}")
    return writer, path


def start_joint_playback(q_targets, duration_per_segment=2.0, after_mode=MODE_TEACH, names=None):
    global mode, playback_active, playback_segments, playback_seg_idx
    global playback_t0, playback_duration, playback_after_mode

    if len(q_targets) == 0:
        print("[play] no target.")
        return

    q_now = data.qpos[arm_qadr].copy()
    points = build_joint_points(q_targets, q_now=q_now)
    playback_segments, _, _, seg_T = build_joint_spline(points, duration_per_segment)
    playback_seg_idx = 0
    playback_t0 = time.time()
    playback_duration = seg_T
    playback_after_mode = after_mode
    playback_active = True
    mode = MODE_PLAYBACK
    cache_playback_path(q_targets, duration_per_segment=duration_per_segment, q_now=q_now)
    print(f"[play] start {len(playback_segments)} segment(s), duration/seg={playback_duration:.2f}s")
    if robot_sync_enabled:
        start_robot_playback(q_targets, duration_per_segment, names=names)


def update_playback():
    global mode, playback_active, playback_seg_idx, playback_t0
    if not playback_active:
        return
    now = time.time()
    tau = now - playback_t0
    q_cmd, _, _ = eval_joint_spline_segment(playback_segments[playback_seg_idx], tau, playback_duration)
    data.qpos[arm_qadr] = q_cmd
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    if tau >= playback_duration:
        playback_seg_idx += 1
        if playback_seg_idx >= len(playback_segments):
            playback_active = False
            mode = playback_after_mode
            set_mocap_to_current_ee()
            print("[play] done. back to teach mode.")
            stop_robot_playback()
            save_robot_playback_logs()
        else:
            playback_t0 = now
            print(f"[play] segment {playback_seg_idx + 1}/{len(playback_segments)}")


def update_robot_visualization():
    if not robot_connected() or not robot_sync_enabled:
        return
    q_robot = consume_robot_visual_state()
    if q_robot is None:
        return
    if robot_playback_active:
        append_robot_actual_path(q_robot)
    data.qpos[arm_qadr] = q_robot
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


def run_ik_teach_step():
    global dq_filt
    x_tgt = data.mocap_pos[mocap_id].copy()
    R_tgt = quat_wxyz_to_R(data.mocap_quat[mocap_id].copy())
    oMd = pin.SE3(R_tgt, x_tgt)

    for _ in range(sub_iters):
        q_pin = pin_state_from_mj()
        pin.forwardKinematics(pin_model, pin_data, q_pin)
        pin.updateFramePlacements(pin_model, pin_data)
        oMe = pin_data.oMf[ee_fid]
        dMe = oMe.inverse() * oMd
        err6 = pin.log6(dMe).vector
        if np.linalg.norm(err6[:3]) < 0.01 and np.linalg.norm(err6[3:]) < 0.005:
            break
        Jfull = pin.computeFrameJacobian(
            pin_model, pin_data, q_pin, ee_fid, pin.ReferenceFrame.LOCAL
        )
        J = Jfull[:, arm_vidx]
        JJt = J @ J.T
        dq = J.T @ np.linalg.solve(JJt + dls * np.eye(6), err6)
        dq = np.clip(dq, -dq_joint_limit, dq_joint_limit)
        n = np.linalg.norm(dq)
        if n > dq_norm_limit:
            dq *= dq_norm_limit / (n + 1e-12)
        dq_filt = (1.0 - beta) * dq_filt + beta * dq
        data.qpos[arm_qadr] = data.qpos[arm_qadr] + alpha * dq_filt
        data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


def handle_command(cmd):
    global mode, speed_scale, keyframes, keyframe_order, robot_sync_enabled
    global recording, video_writer, video_path, playback_active, playback_path_points

    parts = cmd.split()
    if not parts:
        return False
    op = parts[0].lower()

    if op == "help":
        print("Commands: save/list/goto/play/play_all/delete/clear/speed/record_on/record_off/save_json/load_json/mode teach")
        print("          robot_connect/robot_disconnect/robot_status/robot_sync on|off/robot_goto/robot_goto_sim/quit")
    elif op == "save":
        if len(parts) < 2:
            print("usage: save NAME")
            return False
        name = parts[1]
        ee_pos, ee_quat = get_current_ee_pose()
        q = data.qpos[arm_qadr].copy()
        keyframes[name] = {
            "name": name,
            "time": time.time(),
            "q": q.tolist(),
            "ee_pos": ee_pos.tolist(),
            "ee_quat_wxyz": ee_quat.tolist(),
            "mocap_pos": data.mocap_pos[mocap_id].copy().tolist(),
            "mocap_quat_wxyz": data.mocap_quat[mocap_id].copy().tolist(),
        }
        if name not in keyframe_order:
            keyframe_order.append(name)
        print(f"[save] {name}: q={np.round(q, 4)}")
    elif op == "list":
        if not keyframe_order:
            print("[list] no keyframes.")
        else:
            print("[list] keyframes:")
            for i, name in enumerate(keyframe_order):
                q = np.array(keyframes[name]["q"])
                print(f"  {i:02d}. {name:>12s}  q={np.round(q, 4)}")
    elif op == "goto":
        if len(parts) < 2:
            print("usage: goto NAME [T]")
            return False
        name = parts[1]
        if name not in keyframes:
            print(f"[goto] unknown keyframe: {name}")
            return False
        T = float(parts[2]) if len(parts) >= 3 else 2.0
        q_target = np.array(keyframes[name]["q"], dtype=np.float64)
        start_joint_playback([q_target], duration_per_segment=T, names=[name])
    elif op == "play":
        if len(parts) < 2:
            print("usage: play NAME1 NAME2 ...")
            return False
        names = parts[1:]
        missing = [n for n in names if n not in keyframes]
        if missing:
            print(f"[play] unknown keyframe(s): {missing}")
            return False
        q_targets = [np.array(keyframes[n]["q"], dtype=np.float64) for n in names]
        start_joint_playback(q_targets, duration_per_segment=2.0, names=names)
    elif op == "play_all":
        if not keyframe_order:
            print("[play_all] no keyframes.")
            return False
        q_targets = [np.array(keyframes[n]["q"], dtype=np.float64) for n in keyframe_order]
        start_joint_playback(q_targets, duration_per_segment=2.0, names=list(keyframe_order))
    elif op == "delete":
        if len(parts) < 2:
            print("usage: delete NAME")
            return False
        name = parts[1]
        if name in keyframes:
            del keyframes[name]
            keyframe_order = [n for n in keyframe_order if n != name]
            print(f"[delete] {name}")
        else:
            print(f"[delete] unknown keyframe: {name}")
    elif op == "clear":
        keyframes = {}
        keyframe_order = []
        playback_path_points = np.empty((0, 3), dtype=np.float64)
        reset_robot_actual_path()
        print("[clear] all keyframes cleared.")
    elif op == "speed":
        if len(parts) < 2:
            print(f"[speed] current={speed_scale}")
            return False
        speed_scale = max(0.05, float(parts[1]))
        print(f"[speed] set to {speed_scale}x")
    elif op == "record_on":
        if recording:
            print(f"[record] already recording: {video_path}")
            return False
        filename = parts[1] if len(parts) >= 2 else None
        video_writer, video_path = make_video_writer(filename)
        recording = True
        print(f"[record] ON -> {video_path}")
    elif op == "record_off":
        if not recording:
            print("[record] not recording.")
            return False
        recording = False
        if video_writer is not None:
            video_writer.release()
        print(f"[record] OFF -> saved {video_path}")
        video_writer = None
        video_path = None
    elif op == "save_json":
        save_keyframes_to_json()
    elif op == "load_json":
        load_keyframes_from_json()
    elif op == "plot_traj":
        if not keyframe_order:
            print("[plot] no keyframes.")
            return False
        T = float(parts[1]) if len(parts) >= 2 else 2.0
        names = list(keyframe_order)
        q_targets = [np.array(keyframes[n]["q"], dtype=np.float64) for n in names]
        save_trajectory_plot(q_targets, names, duration_per_segment=T)
    elif op == "mode":
        if len(parts) >= 2 and parts[1].lower() == "teach":
            playback_active = False
            mode = MODE_TEACH
            set_mocap_to_current_ee()
            stop_robot_playback()
            reset_robot_actual_path()
            save_robot_playback_logs()
            print("[mode] teach")
        else:
            print("usage: mode teach")
    elif op == "robot_connect":
        device = parts[1] if len(parts) >= 2 else None
        connect_robot(device=device)
    elif op == "robot_disconnect":
        disconnect_robot()
    elif op == "robot_status":
        print_robot_status()
    elif op == "robot_sync":
        if len(parts) < 2 or parts[1].lower() not in ("on", "off"):
            print("usage: robot_sync on|off")
            return False
        robot_sync_enabled = parts[1].lower() == "on"
        if not robot_sync_enabled:
            stop_robot_playback()
            reset_robot_actual_path()
        print(f"[robot] sync {'ON' if robot_sync_enabled else 'OFF'}")
    elif op == "robot_goto":
        if len(parts) < 2:
            print("usage: robot_goto NAME [T]")
            return False
        if not robot_connected():
            print("[robot] not connected.")
            return False
        name = parts[1]
        if name not in keyframes:
            print(f"[robot] unknown keyframe: {name}")
            return False
        duration = float(parts[2]) if len(parts) >= 3 else REAL_MOVE_DURATION
        q_target = np.array(keyframes[name]["q"], dtype=np.float64)
        move_robot_mit(q_target, duration=duration)
        print(f"[robot] moved to {name}")
    elif op == "robot_goto_sim":
        if not robot_connected():
            print("[robot] not connected.")
            return False
        duration = float(parts[1]) if len(parts) >= 2 else REAL_MOVE_DURATION
        move_robot_mit(get_sim_arm_q(), duration=duration)
        print("[robot] moved to current sim q")
    elif op == "quit" or op == "exit":
        print("[quit]")
        request_shutdown("user quit")
        return True
    else:
        print(f"[cmd] unknown command: {op}")
    return False


def run():
    global quit_requested, signal_count
    quit_requested = False
    signal_count = 0
    shutdown_requested.clear()

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and not quit_requested and not shutdown_requested.is_set():
                while not cmd_queue.empty():
                    cmd = cmd_queue.get()
                    try:
                        quit_requested = handle_command(cmd) or quit_requested
                    except Exception as exc:
                        print(f"[cmd error] {cmd}: {exc}")

                if mode == MODE_TEACH:
                    mujoco.mj_forward(model, data)
                    run_ik_teach_step()
                elif mode == MODE_PLAYBACK:
                    update_playback()
                else:
                    mujoco.mj_forward(model, data)

                if mode == MODE_PLAYBACK:
                    update_robot_visualization()

                renderer.update_scene(data, camera=CAMERA_NAME)
                rgb = renderer.render()
                bgr = rgb[:, :, ::-1].copy()
                if recording and video_writer is not None:
                    video_writer.write(bgr)
                lock_ctx = viewer.lock() if hasattr(viewer, "lock") else nullcontext()
                with lock_ctx:
                    draw_playback_path(viewer)
                viewer.sync()
    finally:
        stop_cmd_thread.set()
        robot_thread_stop.set()
        if recording and video_writer is not None:
            video_writer.release()
            print(f"[record] saved {video_path}")
        if robot_connected() or robot_arm is not None:
            disconnect_robot()
        renderer.close()


if __name__ == "__main__":
    try:
        run()
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received. Exiting MuJoCo viewer cleanly.")
