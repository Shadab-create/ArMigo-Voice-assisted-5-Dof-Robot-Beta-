import pybullet as p
import pybullet_data
import time
import numpy as np
import serial
import math
import os
import threading

# ===========================
# SERIAL CONFIG (ESP32)
# ===========================
SERIAL_PORT = "COM8"
BAUD_RATE = 115200

ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print("[Serial] Connected")
except Exception as e:
    print("[Serial Warning]", e)
    ser = None

joint_flips = [-1, -1, -1, -1, -1]

# ==========================
# SEND RATE LIMITER
# Limits serial to 20Hz max — prevents ESP32 buffer overflow
# ==========================
last_send_time    = 0
SEND_INTERVAL     = 0.05  # 20Hz
prev_servo_angles = [90, 90, 90, 90, 90, 0]

def send_angles(arm_joints, gripper_angle):
    global ser, last_send_time, prev_servo_angles

    # Rate limit — only send every 50ms
    current_time = time.time()
    if current_time - last_send_time < SEND_INTERVAL:
        return
    last_send_time = current_time

    if ser is None:
        return
    try:
        servo_angles = []
        for i, rad in enumerate(arm_joints):
            deg = math.degrees(rad) * joint_flips[i] + 90
            deg = int(np.clip(deg, 0, 180))
            servo_angles.append(deg)
        servo_angles.append(int(np.clip(gripper_angle, 0, 70)))

        # Jump detector
        for i in range(5):
            jump = abs(servo_angles[i] - prev_servo_angles[i])
            if jump > 3:
                print(f"[JUMP] Joint {i}: {prev_servo_angles[i]}° → {servo_angles[i]}°  delta={jump}°")
                print(f"       Full angles: {servo_angles}")

        prev_servo_angles = servo_angles.copy()

        msg = ",".join(str(a) for a in servo_angles) + "\n"
        ser.write(msg.encode())
        ser.flush()  # force immediate send, clear buffer

    except Exception as e:
        print("[Serial Write Error]", e)
        ser = None

URDF_PATH = "/home/av_nt/ArMigo-Voice-assisted-5-Dof-Robot-Beta-/Inverse kinematics full along with manual control/my_robot/robot.urdf"
if not os.path.exists(URDF_PATH):
    raise FileNotFoundError(f"URDF not found: {URDF_PATH}")

# ==========================
# PYBULLET SETUP
# ==========================
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

robot = p.loadURDF(URDF_PATH, useFixedBase=True)
num_joints = p.getNumJoints(robot)

movable_joints = [
    i for i in range(num_joints)
    if p.getJointInfo(robot, i)[2] != p.JOINT_FIXED
]

# ==========================
# GUI SLIDERS
# ==========================
gripper_slider      = p.addUserDebugParameter("Gripper (0=open,70=close)", 0, 70, 0)
pitch_target_slider = p.addUserDebugParameter(
    "Gripper Pitch  (90=horizontal  0=up  180=down)", 0, 180, 90
)

# ==========================
# JOINT LIMITS FOR IK
# ==========================
lower_limits = []
upper_limits = []
joint_ranges = []
rest_poses   = []

for j in movable_joints:
    info = p.getJointInfo(robot, j)
    ll = info[8] if info[8] is not None else -math.pi / 2
    ul = info[9] if info[9] is not None else math.pi / 2
    lower_limits.append(ll)
    upper_limits.append(ul)
    joint_ranges.append(ul - ll if ul > ll else math.pi)
    rest_poses.append(0)

J3_LOWER = lower_limits[3]
J3_UPPER = upper_limits[3]

print(f"[Info] Joint 3 limits: "
      f"{round(math.degrees(J3_LOWER),1)}° to "
      f"{round(math.degrees(J3_UPPER),1)}°")

# ==========================
# INITIAL POSES
# ==========================
current_angles = [0.0] * len(movable_joints)
target_angles  = current_angles.copy()
gripper_angle  = 0

for idx, joint in enumerate(movable_joints):
    p.resetJointState(robot, joint, current_angles[idx])
    p.setJointMotorControl2(robot, joint, p.POSITION_CONTROL,
                            targetPosition=current_angles[idx])

send_angles(current_angles, gripper_angle)

# ==========================
# END EFFECTOR
# ==========================
EE_INDEX = None
for i in range(num_joints):
    if p.getJointInfo(robot, i)[12].decode() == "ee":
        EE_INDEX = i
        break
if EE_INDEX is None:
    raise ValueError("EE link 'ee' not found")

ee_pos, ee_orn = p.getLinkState(robot, EE_INDEX)[:2]
ee_pos = np.array(ee_pos)

# ==========================
# IK FREEZE
# ==========================
MOVEMENT_THRESHOLD = 1e-6
FREEZE_DELAY       = 5
ee_pos_prev        = ee_pos.copy()
ik_frozen          = False
freeze_counter     = 0

# ==========================
# AUTO PITCH STATE
# ==========================
auto_j3           = 0.0
auto_pitch_lock   = threading.Lock()
auto_pitch_active = True

SMOOTH_N  = 8
pitch_buf = []
KP_PITCH  = 0.8

def pitch_controller_thread():
    global auto_j3, pitch_buf
    while auto_pitch_active:
        try:
            target_pitch_deg = p.readUserDebugParameter(pitch_target_slider)
            target_x_rad     = math.radians(target_pitch_deg)

            ee_state  = p.getLinkState(robot, EE_INDEX)
            ee_euler  = p.getEulerFromQuaternion(ee_state[5])
            current_x = ee_euler[0]

            error = target_x_rad - current_x

            if abs(error) < math.radians(2.0):
                error = 0.0

            j3_new = current_angles[3] + KP_PITCH * error
            j3_new = float(np.clip(j3_new, J3_LOWER, J3_UPPER))

            pitch_buf.append(j3_new)
            if len(pitch_buf) > SMOOTH_N:
                pitch_buf.pop(0)
            smoothed = sum(pitch_buf) / len(pitch_buf)

            with auto_pitch_lock:
                auto_j3 = smoothed

        except Exception as e:
            print(f"[Pitch Thread Error] {e}")

        time.sleep(0.05)

pitch_thread = threading.Thread(target=pitch_controller_thread, daemon=True)
pitch_thread.start()
print("[Pitch Controller] Thread started")

# ==========================
# PID CONTROLLER
# ==========================
class JointPID:
    def __init__(self, Kp, Ki, Kd, dt,
                 output_limit=0.1,
                 integral_limit=0.2):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.output_limit   = output_limit
        self.integral_limit = integral_limit
        self.prev_error     = 0.0
        self.integral       = 0.0
        self.prev_output    = 0.0
        self.filtered_deriv = 0.0

    def reset(self):
        self.prev_error     = 0.0
        self.integral       = 0.0
        self.prev_output    = 0.0
        self.filtered_deriv = 0.0

    def update(self, current, target):
        error = (target - current + math.pi) % (2 * math.pi) - math.pi

        P = self.Kp * error

        self.integral += error * self.dt
        self.integral  = np.clip(self.integral,
                                 -self.integral_limit,
                                  self.integral_limit)
        I = self.Ki * self.integral

        alpha               = 0.15
        raw_deriv           = (error - self.prev_error) / self.dt
        self.filtered_deriv = (alpha * raw_deriv +
                               (1 - alpha) * self.filtered_deriv)
        D = self.Kd * self.filtered_deriv

        output = np.clip(P + I + D,
                         -self.output_limit,
                          self.output_limit)

        self.prev_error  = error
        self.prev_output = output

        return output

# ==========================
# PID INSTANCES
# ==========================
DT = 0.016

pid = [
    JointPID(1.0, 0.0, 0.30, DT, 0.04, 0.2),  # joint 0 — base
    JointPID(1.0, 0.0, 0.30, DT, 0.04, 0.2),  # joint 1 — shoulder
    JointPID(1.0, 0.0, 0.30, DT, 0.04, 0.2),  # joint 2 — elbow
    JointPID(1.0, 0.0, 0.35, DT, 0.04, 0.2),  # joint 3 — wrist pitch
    JointPID(1.0, 0.0, 0.25, DT, 0.04, 0.2),  # joint 4 — wrist roll
]

# ==========================
# ANTI-FLIP GUARD
# ==========================
MAX_SINGLE_JUMP = 1.2

def is_flip(new_solution, reference):
    for i in range(len(new_solution)):
        diff = (new_solution[i] - reference[i] + math.pi) % (2 * math.pi) - math.pi
        if abs(diff) > MAX_SINGLE_JUMP:
            return True
    return False

# ==========================
# MAIN LOOP PARAMETERS
# ==========================
# Increase the step size so keyboard movement is visible in the PyBullet GUI.
STEP_SIZE     = 0.01
workspace_min = np.array([-0.5, -0.5, 0])
workspace_max = np.array([ 0.5,  0.5,  0.5])

def draw_triad(pos, orn, scale=0.08):
    rot = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
    colors = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]
    for i in range(3):
        p.addUserDebugLine(pos, pos + rot[:, i] * scale,
                           colors[i], 2, lifeTime=0.05)


def is_key_down(keys, key_code):
    return key_code in keys and keys[key_code] & p.KEY_IS_DOWN

print("\n[Controls]")
print("  Arrow keys / WASD → X / Y position")
print("  PgUp/PgDn / Q/E → Z position")
print("  Pitch slider → gripper angle (90=horizontal)")
print("  Click inside the PyBullet window first to capture keys.")
print("\n[Serial] Rate limited to 20Hz — buffer overflow prevention active\n")

# ==========================
# MAIN LOOP
# ==========================
try:
    while True:
        keys = p.getKeyboardEvents()

        gripper_angle = p.readUserDebugParameter(gripper_slider)

        # --- EE movement ---
        if is_key_down(keys, p.B3G_LEFT_ARROW) or is_key_down(keys, ord('a')):
            ee_pos[0] -= STEP_SIZE
        if is_key_down(keys, p.B3G_RIGHT_ARROW) or is_key_down(keys, ord('d')):
            ee_pos[0] += STEP_SIZE
        if is_key_down(keys, p.B3G_UP_ARROW) or is_key_down(keys, ord('w')):
            ee_pos[1] += STEP_SIZE
        if is_key_down(keys, p.B3G_DOWN_ARROW) or is_key_down(keys, ord('s')):
            ee_pos[1] -= STEP_SIZE
        if is_key_down(keys, p.B3G_PAGE_UP) or is_key_down(keys, ord('q')):
            ee_pos[2] += STEP_SIZE
        if is_key_down(keys, p.B3G_PAGE_DOWN) or is_key_down(keys, ord('e')):
            ee_pos[2] -= STEP_SIZE

        ee_pos = np.clip(ee_pos, workspace_min, workspace_max)

        # --- IK freeze logic ---
        ee_moved = np.linalg.norm(ee_pos - ee_pos_prev) > MOVEMENT_THRESHOLD
        ee_pos_prev = ee_pos.copy()

        if ee_moved:
            freeze_counter = 0
            ik_frozen      = False
        else:
            freeze_counter += 1
            if freeze_counter >= FREEZE_DELAY:
                ik_frozen = True

        # --- IK (only when moving) ---
        if not ik_frozen:
            ik_solution = p.calculateInverseKinematics(
                robot, EE_INDEX, ee_pos.tolist(),
                lowerLimits=lower_limits,
                upperLimits=upper_limits,
                jointRanges=joint_ranges,
                restPoses=current_angles,
                maxNumIterations=200,
                residualThreshold=1e-5
            )

            new_target = [ik_solution[j] for j in movable_joints]

            # --- Anti-flip guard ---
            if not is_flip(new_target, target_angles):
                target_angles = new_target

        # --- Pitch override on joint 3 ---
        with auto_pitch_lock:
            j3_candidate = auto_j3

        if abs(j3_candidate - target_angles[3]) > math.radians(1.0):
            target_angles[3] = j3_candidate

        # --- PID update ---
        for i in range(len(current_angles)):
            pid_output         = pid[i].update(current_angles[i], target_angles[i])
            current_angles[i] += pid_output
            current_angles[i]  = np.clip(current_angles[i],
                                          lower_limits[i], upper_limits[i])

        # --- Apply to simulation ---
        for idx, joint in enumerate(movable_joints):
            p.setJointMotorControl2(robot, joint, p.POSITION_CONTROL,
                                    targetPosition=current_angles[idx],
                                    maxVelocity=3.0)

        # --- Send to ESP32 (rate limited to 20Hz) ---
        send_angles(current_angles, gripper_angle)

        draw_triad(ee_pos, p.getQuaternionFromEuler([0, 0, 0]))
        p.stepSimulation()
        time.sleep(DT)

except KeyboardInterrupt:
    print("Exiting simulation...")

finally:
    auto_pitch_active = False
    if ser:
        ser.close()
    p.disconnect()
    input("Press Enter to exit...")