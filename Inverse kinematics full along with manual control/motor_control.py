import serial
import time
import serial.tools.list_ports

# -----------------------------
# Serial communication
# -----------------------------
class SerialComm:
    def __init__(self, baud=115200):
        self.baud = baud
        self.ser = None
        self.port = self.find_esp32_port()
        if self.port:
            self.connect()
        else:
            print("[Serial] ESP32 not found on any COM port.")

    def find_esp32_port(self):
        ports = serial.tools.list_ports.comports()
        for p in ports:
            desc = p.description.lower()
            if "esp32" in desc or "usb serial" in desc or "ch340" in desc:
                print(f"[Serial] Found ESP32 on {p.device} ({p.description})")
                return p.device
        print("[Serial] No ESP32 found. Available ports:")
        for p in ports:
            print(f" - {p.device}: {p.description}")
        return None

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)  # Wait for ESP32 boot
            print(f"[Serial] Connected to {self.port} at {self.baud}")
        except Exception as e:
            print("[Serial] Connection error:", e)
            self.ser = None

    def send_angles(self, joint_angles):
        if self.ser is None:
            print("[Serial] Not connected")
            return
        try:
            # Format: 5 joints + gripper, comma separated
            data = ",".join(str(int(a)) for a in joint_angles) + "\n"
            self.ser.write(data.encode())
            # print(f"[Serial] Sent: {data.strip()}")
        except Exception as e:
            print("[Serial] Write error:", e)

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[Serial] Port closed")


# -----------------------------
# Smooth movement
# -----------------------------
ANGLE_LIMITS = [(0,180)]*5 + [(0,60)]
RAMP_STEP = 1
RAMP_DELAY = 0.02

def clamp(value, min_val, max_val):
    return max(min_val, min(max_val, value))

def ramp_move(current, target):
    new_angles = []
    for c, t, lim in zip(current, target, ANGLE_LIMITS):
        t = clamp(t, lim[0], lim[1])
        if abs(t - c) <= RAMP_STEP:
            new_angles.append(t)
        elif t > c:
            new_angles.append(c + RAMP_STEP)
        else:
            new_angles.append(c - RAMP_STEP)
    return new_angles

def move_joints(target_angles, current_angles=None):
    if current_angles is None:
        current_angles = [(lim[0]+lim[1])//2 for lim in ANGLE_LIMITS]
    while current_angles != target_angles:
        current_angles = ramp_move(current_angles, target_angles)
        serial_comm.send_angles(current_angles)
        time.sleep(RAMP_DELAY)
    return current_angles


# -----------------------------
# Create SerialComm instance
# -----------------------------
serial_comm = SerialComm()
