import serial
import time
import tkinter as tk
import threading
import numpy as np

# ================== SERIAL ==================
SERIAL_PORT = 'COM8'
BAUD_RATE = 115200

ser = None
serial_lock = threading.Lock()

def connect_serial():
    global ser
    try:
        if ser and ser.is_open:
            return True
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print("[Serial] Connected")
        return True
    except Exception as e:
        print("[Serial Error]", e)
        ser = None
        return False

def is_connected():
    return ser is not None and ser.is_open

connect_serial()

# ================== SEND ANGLES ==================
def send_angles(joints):
    joints = joints.copy()
    joints[5] = int(np.clip(joints[5], 0, 70))

    msg = ",".join(str(int(round(a))) for a in joints) + "\n"

    with serial_lock:
        try:
            if not is_connected():
                connect_serial()
            if ser and ser.is_open:
                ser.write(msg.encode())
        except Exception as e:
            print("[Serial Write Error]", e)
            try:
                ser.close()
            except:
                pass

# ================== ROBOT STATE ==================
current_angles = [90, 90, 90, 90, 90, 0]
last_sent = current_angles.copy()
angles_lock = threading.Lock()

# ================== GUI ==================
root = tk.Tk()
root.title("Robot Arm Manual Control")

joint_names = ["Base", "Shoulder", "Elbow", "Wrist1", "Wrist2"]
direction_labels = [
    "(0°=Right, 180°=Left)",
    "(0°=Up, 180°=Down)",
    "(0°=Down, 180°=Up)",
    "(0°=Up, 180°=Down)",
    "(0°=Left, 180°=Right)"
]

slider_vars = []

def on_slider_change(_=None):
    global last_sent
    with angles_lock:
        for i in range(5):
            current_angles[i] = slider_vars[i].get()

        # Only send if changed
        if any(abs(current_angles[i] - last_sent[i]) >= 1 for i in range(5)):
            last_sent = current_angles.copy()
            send_angles(current_angles)

for i in range(5):
    tk.Label(root, text=f"{joint_names[i]} {direction_labels[i]}",
             font=("Arial", 12, "bold")).grid(row=i, column=0, padx=10, pady=5)

    var = tk.IntVar(value=current_angles[i])
    slider = tk.Scale(root, from_=0, to=180, orient=tk.HORIZONTAL,
                      length=420, variable=var, command=on_slider_change)
    slider.grid(row=i, column=1, padx=10, pady=5)
    tk.Label(root, textvariable=var, font=("Arial", 12)).grid(row=i, column=2, padx=10)

    slider_vars.append(var)

# ================== GRIPPER ==================
def set_gripper(val):
    with angles_lock:
        current_angles[5] = val
        send_angles(current_angles)

tk.Label(root, text="Gripper (0°=Open, 70°=Close)", font=("Arial",12,"bold")).grid(
    row=5, column=0, padx=10, pady=5)

tk.Button(root, text="Open", bg="lightblue", command=lambda: set_gripper(0)).grid(
    row=5, column=1, sticky="we")
tk.Button(root, text="Close", bg="lightcoral", command=lambda: set_gripper(70)).grid(
    row=5, column=2, sticky="we")

# ================== PRESETS ==================
def set_preset(pose):
    with angles_lock:
        for i in range(5):
            slider_vars[i].set(pose[i])
            current_angles[i] = pose[i]
        send_angles(current_angles)

presets = [
    ("Middle", [90, 90, 90, 90, 90]),
    ("Home",   [90, 40, 95,145, 90]),
    ("Drop",   [180,145, 0, 20, 90]),
    ("Left1",  [141,128,27,113,90]),
    ("Left2",  [102,135,41,132,90]),
    ("Right1", [18,107,6,155,90]),
    ("Right2", [38,147,18,43,82]),
]

tk.Label(root, text="Preset Positions", font=("Arial",13,"bold")).grid(
    row=6, column=0, columnspan=3, pady=10)

for i, (name, pose) in enumerate(presets):
    tk.Button(root, text=name, command=lambda p=pose: set_preset(p)).grid(
        row=7+i, column=0, columnspan=3, sticky="we", pady=3)

# ================== STATUS ==================
status_label = tk.Label(root, text="Checking...", font=("Arial",12,"bold"))
status_label.grid(row=15, column=0, columnspan=3)

def update_status():
    if is_connected():
        status_label.config(text="🟢 Connected", fg="green")
    else:
        status_label.config(text="🔴 Disconnected", fg="red")
        threading.Thread(target=connect_serial, daemon=True).start()
    root.after(1000, update_status)

update_status()

def on_close():
    try:
        if ser:
            ser.close()
    except:
        pass
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()
