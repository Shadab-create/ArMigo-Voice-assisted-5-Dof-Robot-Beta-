# ----------------------------- main.py -----------------------------
import time
import os
import cv2 as cv
import numpy as np
import motor_control
import camera_detection
import inverse_kinematics  # Use the new IK code

# ----------------------------- CONFIG -----------------------------
LINKS = [1.313, 12.492, 15.95, 4.646, 0]  # cm

current_angles = [90, 50, 0, 135, 90, 0]  # 6th joint = gripper
HOME_ANGLES = [90, 50, 0, 135, 90, 0]

DROP_ANGLES = [175, 130, 25, 125, 90]  # 5 DOF
DROP_POSITION = np.array([10.0, 30.0, 5.0])  # reference if needed

# ----------------------------- UTILITIES -----------------------------
def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

# ----------------------------- IK MOVEMENT -----------------------------
def move_to_object(target_pos, hover_offset=5.0):
    global current_angles
    target_hover = target_pos.copy()
    target_hover[2] += hover_offset
    target_hover[2] = max(target_hover[2], 5.0)

    # Small offsets to improve IK success
    offsets = [(0,0,0),(2,0,0),(-2,0,0),(0,2,0),(0,-2,0)]
    for dx, dy, dz in offsets:
        pos_try = target_hover + np.array([dx, dy, dz])
        res = inverse_kinematics.solve_ik_best_dh_deg_pos(
            pos_try, theta0_deg=current_angles[:5]
        )
        if res['success']:
            target_deg = list(res['theta_deg']) + [current_angles[5]]
            current_angles = motor_control.move_joints(target_deg, current_angles)
            return True
    return False

# ----------------------------- PICK & DROP -----------------------------
def pick_object(coords_robot):
    global current_angles
    if not move_to_object(coords_robot):
        print("[Main] IK failed: hover")
        return False

    pick_pos = coords_robot.copy()
    pick_pos[2] -= 0.5
    if not move_to_object(pick_pos, hover_offset=0.0):
        print("[Main] IK failed: pick")
        return False

    current_angles[5] = 60  # close gripper
    current_angles = motor_control.move_joints(current_angles, current_angles)

    hover_pos = pick_pos.copy()
    hover_pos[2] += 5.0
    move_to_object(hover_pos, hover_offset=0.0)

    return True

def drop_object():
    global current_angles
    target_deg = DROP_ANGLES + [current_angles[5]]
    current_angles = motor_control.move_joints(target_deg, current_angles)

    current_angles[5] = 0  # open gripper
    current_angles = motor_control.move_joints(current_angles, current_angles)
    return True

# ----------------------------- MAIN LOOP -----------------------------
def main():
    global current_angles
    print(f"[Startup] Moving to Home: {HOME_ANGLES}")
    current_angles = motor_control.move_joints(HOME_ANGLES, current_angles)
    print("[Startup] Robot at Home.")

    try:
        while True:
            coords_base, coords_robot, frame = camera_detection.get_object_coords()

            key = cv.waitKey(1) & 0xFF
            action = None
            if key == ord('q'):
                break
            elif key == ord('p'):
                action = 'pick'
            elif key == ord('h'):
                action = 'home'

            clear_screen()
            print("=== ROBOT STATUS ===")
            print(f"Current joints: {current_angles}")
            if coords_robot is not None:
                print(f"Object (robot frame): {coords_robot}")
                print(f"Object (base frame): {coords_base}")
            else:
                print("No object detected")
            print("\n=== CONTROLS ===")
            print("[P] Pick & place  [H] Home  [Q] Quit")

            if frame is not None:
                cv.imshow("Camera Feed", frame)

            if action == 'pick' and coords_robot is not None:
                print("[Main] Picking object...")
                if pick_object(coords_robot):
                    print("[Main] Dropping object...")
                    drop_object()
                    print("[Main] Pick-and-place complete.")
                else:
                    print("[Main] Pick failed.")
            elif action == 'home':
                current_angles = motor_control.move_joints(HOME_ANGLES, current_angles)

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[Main] KeyboardInterrupt received.")
    finally:
        try:
            motor_control.serial_comm.close()
        except:
            pass
        camera_detection.close_camera()
        cv.destroyAllWindows()
        print("[Main] Program exited safely.")

if __name__ == "__main__":
    main()
