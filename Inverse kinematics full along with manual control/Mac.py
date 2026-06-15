import pybullet as p
import pybullet_data
import time
import numpy as np
import serial
import math
import os
import threading

# ==========================
# SERIAL CONFIG (macOS)
# ==========================
# On Mac, ports usually look like "/dev/cu.usbserial-XXXX" or "/dev/cu.usbmodemXXXX"
SERIAL_PORT = "/dev/cu.usbserial-0001" 
BAUD_RATE = 115200

ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print(f"[Serial] Connected to {SERIAL_PORT}")
except Exception as e:
    print("[Serial Warning] Could not connect. Check your port name in /dev/")
    print(f"Error: {e}")
    ser = None

joint_flips = [-1, -1, -1, -1, -1]

# ==========================
# SEND RATE LIMITER
# ==========================
last_send_time    = 0
SEND_INTERVAL     = 0.05  # 20Hz
prev_servo_angles = [90, 90, 90, 90, 90, 0]

def send_angles(arm_joints, gripper_angle):
    global ser, last_send_time, prev_servo_angles

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

        # Jump detector (Helps debug if IK is flickering)
        for i in range(5):
            jump = abs(servo_angles[i] - prev_servo_angles[i])
            if jump > 5: # Slightly higher tolerance without PID
                print(f"[JUMP] Joint {i}: {prev_servo_angles[i]}° → {servo_angles[i]}°")

        prev_servo_angles = servo_angles.copy()

        msg = ",".join(str(a) for a in servo_angles) + "\n"
        ser.write(msg.encode())
        ser.flush() 

    except Exception as e:
        print("[Serial Write Error]", e)
        ser = None

# Update this path for your Mac file system
URDF_PATH = "robot.urdf" 
if not os.path.exists(URDF_PATH):
    print(f"Current Directory: {os.getcwd()}")
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
pitch_target_slider = p.addUserDebugParameter("Gripper Pitch (90=H)", 0, 180, 90)

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

# ==========================
# INITIAL STATE
# ==========================
target_angles = [0.0] * len(movable_joints)
gripper_angle = 0

EE_INDEX = None
for i in range(num_joints):
    if p.getJointInfo(robot, i)[12].decode() == "ee":
        EE_INDEX = i
        break
if EE_INDEX is None:
    raise ValueError("EE link 'ee' not found")

ee_pos, _ = p.getLinkState(robot, EE_INDEX)[:2]
ee_pos = np.array(ee_pos)
ee_pos_prev = ee_pos.copy()

# ==========================
# AUTO PITCH THREAD
# ==========================
auto_j3 = 0.0
auto_pitch_lock = threading.Lock()
auto_pitch_active = True

def pitch_controller_thread():
    global auto_j3
    while auto_pitch_active:
        try:
            target_pitch_deg = p.readUserDebugParameter(pitch_target_slider)
            target_x_rad = math.radians(target_pitch_deg)

            ee_state = p.getLinkState(robot, EE_INDEX)
            ee_euler = p.getEulerFromQuaternion(ee_state[5])
            current_x = ee_euler[0]

            error = target_x_rad - ee_euler[0]
            
            # Simple direct adjustment for pitch
            with auto_pitch_lock:
                # We use the current target and nudge it to fix orientation
                auto_j3 = np.clip(target_angles[3] + error, J3_LOWER, J3_UPPER)

        except Exception:
            pass
        time.sleep(0.05)

threading.Thread(target=pitch_controller_thread, daemon=True).start()

# ==========================
# MAIN LOOP
# ==========================
STEP_SIZE = 0.005 # Increased slightly since we removed PID smoothing
DT = 0.016
workspace_min = np.array([-0.5, -0.5, 0])
workspace_max = np.array([ 0.5,  0.5,  0.5])

try:
    while True:
        keys = p.getKeyboardEvents()
        gripper_angle = p.readUserDebugParameter(gripper_slider)

        # Movement Inputs
        if p.B3G_LEFT_ARROW  in keys and keys[p.B3G_LEFT_ARROW]  & p.KEY_IS_DOWN: ee_pos[0] -= STEP_SIZE
        if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN: ee_pos[0] += STEP_SIZE
        if p.B3G_UP_ARROW    in keys and keys[p.B3G_UP_ARROW]    & p.KEY_IS_DOWN: ee_pos[1] += STEP_SIZE
        if p.B3G_DOWN_ARROW  in keys and keys[p.B3G_DOWN_ARROW]  & p.KEY_IS_DOWN: ee_pos[1] -= STEP_SIZE
        if p.B3G_PAGE_UP     in keys and keys[p.B3G_PAGE_UP]     & p.KEY_IS_DOWN: ee_pos[2] += STEP_SIZE
        if p.B3G_PAGE_DOWN   in keys and keys[p.B3G_PAGE_DOWN]   & p.KEY_IS_DOWN: ee_pos[2] -= STEP_SIZE

        ee_pos = np.clip(ee_pos, workspace_min, workspace_max)

        # Calculate IK
        ik_solution = p.calculateInverseKinematics(
            robot, EE_INDEX, ee_pos.tolist(),
            lowerLimits=lower_limits,
            upperLimits=upper_limits,
            jointRanges=joint_ranges,
            restPoses=rest_poses
        )
        
        # Update targets directly
        for idx, j_idx in enumerate(movable_joints):
            target_angles[idx] = ik_solution[idx]

        # Pitch Override
        with auto_pitch_lock:
            target_angles[3] = auto_j3

        # Apply directly to simulation (No PID)
        for idx, joint in enumerate(movable_joints):
            p.setJointMotorControl2(
                robot, joint, p.POSITION_CONTROL,
                targetPosition=target_angles[idx],
                force=500
            )

        # Send to Hardware
        send_angles(target_angles, gripper_angle)

        p.stepSimulation()
        time.sleep(DT)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    auto_pitch_active = False
    if ser: ser.close()
    p.disconnect()