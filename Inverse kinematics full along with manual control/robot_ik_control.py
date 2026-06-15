import pybullet as p
import pybullet_data
import time
import numpy as np
import serial
import math
import os

# ==========================
# SERIAL CONFIG (ESP32)
# ==========================
SERIAL_PORT = "COM4"
BAUD_RATE = 115200

ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print("[Serial] Connected")
except Exception as e:
    print("[Serial Warning]", e)
    ser = None

# ==========================
# JOINT FLIPS (ARM ONLY)
# ==========================
joint_flips = [-1, -1, -1, -1, -1]

# ==========================
# SEND ANGLES (5 ARM + 1 GRIPPER)
# ==========================
def send_angles(arm_joints, gripper_angle):
    global ser
    if ser is None:
        return
    try:
        servo_angles = []

        # Arm joints (rad → deg)
        for i, rad in enumerate(arm_joints):
            deg = math.degrees(rad) * joint_flips[i] + 90
            deg = int(np.clip(deg, 0, 180))
            servo_angles.append(deg)

        # Gripper (already in degrees: 0–70)
        servo_angles.append(int(np.clip(gripper_angle, 0, 70)))

        msg = ",".join(str(a) for a in servo_angles) + "\n"
        ser.write(msg.encode())

    except Exception as e:
        print("[Serial Write Error]", e)
        ser = None

# ==========================
# URDF PATH
# ==========================
URDF_PATH = r"D:\ArMigo(Camera_detection)\my_robot\robot.urdf"
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

# Arm joints ONLY (5 joints)
movable_joints = [
    i for i in range(num_joints)
    if p.getJointInfo(robot, i)[2] != p.JOINT_FIXED
]

# ==========================
# GUI SLIDER (GRIPPER)
# ==========================
gripper_slider = p.addUserDebugParameter("Gripper (0=open,70=close)", 0, 70, 0)

# ==========================
# JOINT LIMITS FOR IK
# ==========================
lower_limits = []
upper_limits = []
joint_ranges = []
rest_poses = []

for j in movable_joints:
    info = p.getJointInfo(robot, j)
    ll = info[8] if info[8] is not None else -math.pi / 2
    ul = info[9] if info[9] is not None else math.pi / 2
    lower_limits.append(ll)
    upper_limits.append(ul)
    joint_ranges.append(ul - ll if ul > ll else math.pi)
    rest_poses.append(0)

# ==========================
# INITIAL POSES
# ==========================
current_angles = [0] * len(movable_joints)
target_angles = current_angles.copy()

# Gripper (hardware only)
gripper_angle = 0  # 0=open, 70=close

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
# MAIN LOOP PARAMETERS
# ==========================
STEP_SIZE = 0.0005
workspace_min = np.array([-0.5, -0.5, 0])
workspace_max = np.array([0.5, 0.5, 0.5])
JOINT_STEP = 0.02

def draw_triad(pos, orn, scale=0.08):
    rot = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
    colors = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]
    for i in range(3):
        p.addUserDebugLine(pos, pos + rot[:, i] * scale,
                           colors[i], 2, lifeTime=0.05)

# ==========================
# MAIN LOOP
# ==========================
try:
    while True:
        keys = p.getKeyboardEvents()

        # --- Gripper slider ---
        gripper_angle = p.readUserDebugParameter(gripper_slider)

        # --- Move EE ---
        if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
            ee_pos[0] -= STEP_SIZE
        if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
            ee_pos[0] += STEP_SIZE
        if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
            ee_pos[1] += STEP_SIZE
        if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
            ee_pos[1] -= STEP_SIZE
        if p.B3G_PAGE_UP in keys and keys[p.B3G_PAGE_UP] & p.KEY_IS_DOWN:
            ee_pos[2] += STEP_SIZE
        if p.B3G_PAGE_DOWN in keys and keys[p.B3G_PAGE_DOWN] & p.KEY_IS_DOWN:
            ee_pos[2] -= STEP_SIZE

        ee_pos = np.clip(ee_pos, workspace_min, workspace_max)

        # --- IK ---
        ik_solution = p.calculateInverseKinematics(
            robot,
            EE_INDEX,
            ee_pos.tolist(),
            lowerLimits=lower_limits,
            upperLimits=upper_limits,
            jointRanges=joint_ranges,
            restPoses=current_angles,
            maxNumIterations=200,
            residualThreshold=1e-5
        )

        target_angles = [ik_solution[j] for j in movable_joints]

        # --- Smooth arm motion ---
        for i in range(len(current_angles)):
            diff = target_angles[i] - current_angles[i]
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi
            current_angles[i] += np.clip(diff, -JOINT_STEP, JOINT_STEP)

        # --- Apply to simulation ---
        for idx, joint in enumerate(movable_joints):
            p.setJointMotorControl2(
                robot, joint, p.POSITION_CONTROL,
                targetPosition=current_angles[idx]
            )

        # --- Send to ESP32 ---
        send_angles(current_angles, gripper_angle)

        draw_triad(ee_pos, p.getQuaternionFromEuler([0, 0, 0]))

        p.stepSimulation()
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Exiting simulation...")

finally:
    if ser:
        ser.close()
    p.disconnect()
    input("Press Enter to exit...")
