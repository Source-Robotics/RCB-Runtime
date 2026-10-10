#!/usr/bin/env python3
"""Where does an RTI tick go? Per-function cost of the superloop's per-tick work, measured OFFLINE on this Pi
with the real PAR6 model, the real filter banks and the real recipes, in a sandboxed shared-memory namespace.
Also: does Pinocchio's generalized gravity equal rtb's rne(q, 0, 0) on the dynamics model (the candidate
replacement for the per-tick Python RNE)?

Run (robot DISABLED or off; it never touches the live memory):  RCB_SHM_PREFIX=bench_ nice -n 19 .venv/bin/python tools/bench_rti_tick.py
"""
import os
import sys
import timeit
from collections import deque

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "dummy_test_scripts"))
import _shm_sandbox  # noqa: F401,E402
import numpy as np  # noqa: E402

from config import xml_parser as xp  # noqa: E402
from project_paths import CONFIG_DIR, DATA_RECIPE_DIR, GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML  # noqa: E402
from utility.robot_factory import create_robot  # noqa: E402
from utility.pinocchio_attach import attach_pinocchio, build_pinocchio_dynamics  # noqa: E402
from robotics.dynamics import calculate_gravity_compensation, estimate_external_force_tcp, motor_current_to_joint_torque  # noqa: E402
from robotics.robot_kinematics import calculate_all_kinematics  # noqa: E402
from utility.filter_config import load_filter_banks  # noqa: E402
from utility.signal_filters import apply_filter_bank  # noqa: E402
from utility.data_init import initialize_robot_data, process_joint_data, process_motor_data  # noqa: E402
from data import shared_data  # noqa: E402
from data.update_shared_data import update_joint_and_tcp_data, update_motor_and_gripper_data  # noqa: E402
from data.data_recipe import RecipeManager, build_payload  # noqa: E402
from hardware.motor_setup import create_joint_objects, create_motor_objects  # noqa: E402
from utility.signal_processing import update_rolling_percentiles  # noqa: E402
from utility.error_checks import check_errors  # noqa: E402
import Spectral_BLDC as Spectral  # noqa: E402
import can  # noqa: E402

N = 6
DT = 0.004
rows = []


def t(name, fn, number=300, repeat=5):
    best = min(timeit.repeat(fn, number=number, repeat=repeat)) / number * 1e6
    rows.append((name, best))
    print(f"  {best:8.1f} us  {name}")
    return best


print("== setup")
system = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
cfg = xp.load_full_robot_config(str(system["robot_xml_path"]), grippers_dir=GRIPPERS_DIR, active_gripper=system["active_gripper"])
robot = create_robot(cfg["robot_name"], gripper_name=cfg["gripper_name"])
attach_pinocchio(robot)
import pinocchio as pin  # noqa: E402
dyn_model, dyn_data = build_pinocchio_dynamics(robot)
lo = np.asarray(cfg["software_joint_limit_negative"], float)
hi = np.asarray(cfg["software_joint_limit_positive"], float)
rng = np.random.default_rng(1)
Q = lo + rng.random((300, N)) * (hi - lo)
q = Q[0].copy()
qd = rng.normal(0, 0.3, N)
qdd = rng.normal(0, 1.0, N)

print("== gravity: rtb rne(q,0,0) vs pinocchio computeGeneralizedGravity on the dynamics model")
g_rtb = np.array([calculate_gravity_compensation(robot, qq) for qq in Q])
g_pin = np.array([pin.computeGeneralizedGravity(dyn_model, dyn_data, qq) for qq in Q])
d = np.abs(g_rtb - g_pin)
print(f"  max |diff| over 300 poses: {d.max():.3e} Nm (per joint max {np.round(d.max(axis=0), 6)}); |g| up to {np.abs(g_rtb).max():.2f} Nm")
print(f"  model gravity vector: pin {dyn_model.gravity.linear}  rtb {getattr(robot, 'gravity', '?')}")
t("gravity: rtb robot.rne(q, 0, 0)            [per tick today]", lambda: calculate_gravity_compensation(robot, q))
t("gravity: pin.computeGeneralizedGravity      [candidate]", lambda: pin.computeGeneralizedGravity(dyn_model, dyn_data, q))

print("== kinematics (pinocchio backend, as RTI runs it)")
t("calculate_all_kinematics(q, qd, qdd) full   [per tick today]", lambda: calculate_all_kinematics(robot, q, qd, joint_acceleration=qdd))
t("calculate_all_kinematics, no diagnostics (no SVD)", lambda: calculate_all_kinematics(robot, q, qd, joint_acceleration=qdd, include_diagnostics=False))
kin = calculate_all_kinematics(robot, q, qd, joint_acceleration=qdd)
J = kin["J"]
t("np.linalg.svd(J, compute_uv=False)", lambda: np.linalg.svd(J, compute_uv=False))
tau = rng.normal(0, 1, N)
t("estimate_external_force_tcp (solve J^T)      [per tick today]", lambda: estimate_external_force_tcp(tau, J))
t("robot.pinocchio_fkine_and_jacob0 (= _fk_commanded)", lambda: robot.pinocchio_fkine_and_jacob0(q))

print("== filters")
banks, _src = load_filter_banks(CONFIG_DIR / "filters.xml", N, DT)
x = rng.normal(0, 1, N); out = np.zeros(N)
for name in ("speed", "accel", "jerk", "torque"):
    t(f"apply_filter_bank['{name}'] ({banks[name]['filters'][0]['kind'] if 'kind' in banks[name]['filters'][0] else '?'})", lambda b=banks[name]: apply_filter_bank(b, x, out))
x6 = rng.normal(0, 1, 6); out6 = np.zeros(6)
t("apply_filter_bank['tcp_accel'] (6 ch)", lambda: apply_filter_bank(banks["tcp_accel"], x6, out6))

print("== shared memory + data path (sandboxed shm, real structures)")
shared_data.setup(N)
data = initialize_robot_data(N)
Motor_data, Gripper_data, Joint_data = data["Motor_data"], data["Gripper_data"], data["Joint_data"]
for i in range(N + 1):
    process_motor_data(Motor_data, i, 10000 + i, 5, 120, 0.0, (i, 3, 0), 1, 0)
Joint = create_joint_objects(N, cfg["motor_encoder"], cfg["master_position"], cfg["gear_ratio"], cfg["motor_offset"], cfg["motor_dir"])
motor_error_bytes = np.zeros(N, dtype=np.int32)
shared_data.rti["loop_tick"][0] = 7
t("update_motor_and_gripper_data (tick%5 != 0,1,2)", lambda: update_motor_and_gripper_data(
    shared_data=shared_data, Motor_data=Motor_data, Motor_temperature_data=data["Motor_temperature_data"],
    Motor_voltage_data=data["Motor_voltage_data"], Motor_info_data=data["Motor_info_data"], Motor_errors_data=data["Motor_errors_data"],
    Motor_Kt_data=data["Motor_Kt_data"], Gripper_data=Gripper_data, motor_error_bytes=motor_error_bytes, joint_num=N, active_CAN_gripper=1,
    gripper_kt=0.1, gripper_speed_bank=banks["gripper_speed"]))
jac = np.zeros((6, N))
gear_ratio = np.asarray(cfg["gear_ratio"], float); gear_eff = np.asarray(cfg["gear_efficiency"], float)
motor_kt = np.asarray(cfg["motor_kt"], float); motor_signs = np.sign(gear_ratio)
t("update_joint_and_tcp_data (FK+J+Jdot+SVD+RNE+solve+filters) [per tick today]", lambda: update_joint_and_tcp_data(
    shared_data=shared_data, Joint=Joint, Joint_data=Joint_data, filter_banks=banks, joint_num=N, delta_t=DT, gear_ratio=gear_ratio,
    gear_efficiency=gear_eff, motor_kt=motor_kt, motor_signs=motor_signs, robot=robot, jacobian_out=jac))
sj, st_ = shared_data.joint, shared_data.tcp
t("process_joint_data (36 array copies + dict -> deque)   [per tick today]", lambda: process_joint_data(
    Joint_data, sj['joint_position'], sj['joint_speed'], sj['joint_torque'], sj['joint_acceleration'], sj['joint_jerk'],
    sj['joint_speed_filtered'], sj['joint_torque_filtered'], sj['joint_acceleration_filtered'], sj['joint_jerk_filtered'],
    st_['robot_pose'], st_['robot_pose_matrix'], st_['robot_pose_quaternion'], st_['TCP_speed'], st_['torque_external'],
    st_['force_external'], st_['TCP_speed_filtered'], st_['TCP_acceleration_raw'], st_['TCP_acceleration_filtered'],
    st_['force_external_tool'], sj['joint_position_commanded'], sj['joint_speed_commanded'], sj['joint_acceleration_commanded'],
    sj['joint_torque_commanded'], st_['TCP_pose_commanded'], st_['TCP_speed_commanded'], sj['joint_position_target'],
    sj['joint_speed_target'], sj['joint_acceleration_target'], sj['joint_torque_target'], st_['TCP_pose_target'],
    st_['TCP_speed_target'], 0.0, 0.002, 7, motor_error_bytes))
t("joint_position comprehension (6x Joint.get_joint_position)", lambda: [Joint[i].get_joint_position(shared_data.motor['motor_position'][i]) for i in range(N)])
t("motor_current_to_joint_torque (4 temporaries)", lambda: motor_current_to_joint_torque(shared_data.motor['motor_current'][:], gear_ratio, gear_eff, motor_kt, motor_signs))
t("SharedString.get() x1 (homed_state)", lambda: shared_data.homed_state.get())
t("shared_data.rti['loop_tick'][0] read x10", lambda: [shared_data.rti["loop_tick"][0] for _ in range(10)])
t("robot_command.set_truncated(60-char)", lambda: shared_data.robot_command.set_truncated('{"msg": "set_mode:IDLE", "id": "vwKy0p", "source": "TCP"}'))

print("== telemetry serialisation (build_payload) per recipe")
mgr = RecipeManager(DATA_RECIPE_DIR)
motor_buffers = {"data": Motor_data, "temperature": data["Motor_temperature_data"], "voltage": data["Motor_voltage_data"]}
for name in ("minimal", "standard", "haptic_wall", "commanded", "diagnostics", "full"):
    r = mgr.get(name)
    if r is None:
        continue
    n = len(build_payload(r, Joint_data, Gripper_data, motor_buffers, N))
    t(f"build_payload('{name}') -> {n} bytes", lambda r=r: build_payload(r, Joint_data, Gripper_data, motor_buffers, N))
t("recipe_mgr.get('standard') (cached)", lambda: mgr.get("standard"))

print("== CAN send path (library packing; the bus is a no-op)")
class _NoBus:
    def send(self, m): pass
comm = Spectral.CanCommunication(channel=None)
comm.bus = _NoBus()
Motor = create_motor_objects(N, comm, True)
t("Motor.Send_data_pack_1(pos, speed, current) x1  [x6-7 per tick]", lambda: Motor[0].Send_data_pack_1(Position=12345, Speed=100, Current=50))
t("Motor.Send_data_pack_1(None, None, 0) (idle hold)", lambda: Motor[0].Send_data_pack_1(Position=None, Speed=None, Current=0))
t("Motor.Send_Respond_Temperature (poll)", lambda: Motor[0].Send_Respond_Temperature())
msg = can.Message(arbitration_id=0x123, data=b"\x01" * 8, is_extended_id=False)
t("can.Message(...) construction only", lambda: can.Message(arbitration_id=0x123, data=b"\x01" * 8, is_extended_id=False))
build_can_frame = None
pass
import struct  # noqa: E402
t("struct.pack raw CAN frame (what a fast path would cost)", lambda: struct.pack("=IB3x8s", 0x123, 8, b"\x01" * 8))

print("== bookkeeping")
pb, eb = np.zeros(500), np.zeros(500)
t("update_rolling_percentiles, ordinary tick", lambda: update_rolling_percentiles(pb, eb, 3, 500, 0.004, 0.002, 7, 0, 0, 0, 0, stats_interval=50))
t("update_rolling_percentiles, the every-50th tick (2x np.percentile over 500)", lambda: update_rolling_percentiles(pb, eb, 3, 500, 0.004, 0.002, 50, 0, 0, 0, 0, stats_interval=50))


class _Homed:
    def get(self): return "HOMED"


class _SD:
    def __init__(self, n=6):
        z = lambda: np.zeros(n)  # noqa: E731
        self.motor = {f: z() for f in ("motor_temperature_error", "motor_encoder_error", "motor_vbus_error", "motor_driver_error",
                                       "motor_velocity_error", "motor_current_error", "motor_estop_error", "motor_watchdog_error")}
        self.motor["global_motor_error"] = z()
        self.rti = {"software_estop": np.zeros(1), "CAN_node_connection_error": z(), "old_data_warning": z(), "homing_status": z()}
        self.homed_state = _Homed()


sd = _SD(); latched = set()
t("check_errors (clean robot, no gripper)", lambda: check_errors(sd, N, latched, estop_active=False, active_CAN_gripper=0))
from hardware.gpio_handler import _apply_debounce, ISOLATED_INPUT_1  # noqa: E402
t("_apply_debounce x9 (the per-tick GPIO filter, minus the 9 lgpio reads)", lambda: [_apply_debounce(ISOLATED_INPUT_1, 1) for _ in range(9)])

print()
print(f"{'us/tick':>9}  item")
tot = 0.0
for name, us in rows:
    print(f"{us:9.1f}  {name}")
