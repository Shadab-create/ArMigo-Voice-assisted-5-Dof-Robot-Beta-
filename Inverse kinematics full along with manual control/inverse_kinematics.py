import numpy as np

# -----------------------------
# DH Table (5 DOF including tool)
# -----------------------------
DH_TABLE_5DOF = [
    {'a': 0.0127,  'alpha':  90.0,   'd': 0.1473,  'offset': 90.0},   # Joint 1
    {'a': 0.1270,  'alpha': 180.0,   'd': 0.0000,  'offset': -2.826}, # Joint 2
    {'a': 0.1596,  'alpha': -180.0,  'd': 0.0000,  'offset': 87.1},   # Joint 3
    {'a': 0.0463,  'alpha': -90.3,   'd': 0.0244,  'offset': 180.0},  # Joint 4
    {'a': 0.0000,  'alpha': 0.0,     'd': 0.0900,  'offset': 0.0}     # Tool offset
]

JOINT_LIMITS_4DOF = [(0.0,180.0), (0.0,180.0), (0.0,180.0), (0.0,180.0)]
FIXED_5TH = 90.0  # wrist/EE fixed

# -----------------------------
# Utilities
# -----------------------------
def deg2rad(d): return d * np.pi / 180.0
def rad2deg(r): return r * 180.0 / np.pi

def dh_matrix(a, alpha_deg, d, theta_deg):
    alpha = deg2rad(alpha_deg)
    theta = deg2rad(theta_deg)
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ ct, -st*ca,  st*sa, a*ct],
        [ st,  ct*ca, -ct*sa, a*st],
        [  0,     sa,     ca,    d],
        [  0,      0,      0,    1]
    ], dtype=float)

# -----------------------------
# Forward Kinematics
# -----------------------------
def forward_kinematics_deg(theta_deg_4):
    """Return EE position for 4 joint angles (deg)."""
    T = np.eye(4)
    for i in range(4):
        dh = DH_TABLE_5DOF[i]
        theta_dh = float(theta_deg_4[i]) - dh['offset']
        T = T @ dh_matrix(dh['a'], dh['alpha'], dh['d'], theta_dh)
    # include 5th tool offset
    dh5 = DH_TABLE_5DOF[4]
    T = T @ dh_matrix(dh5['a'], dh5['alpha'], dh5['d'], 0.0)
    return T[:3,3].copy()

# -----------------------------
# Analytic Jacobian (position only)
# -----------------------------
def analytic_jacobian_deg(theta_deg_4):
    T = np.eye(4)
    Z = [np.array([0.0,0.0,1.0])]
    O = [np.zeros(3)]
    for i in range(4):
        dh = DH_TABLE_5DOF[i]
        theta_dh = float(theta_deg_4[i]) - dh['offset']
        T = T @ dh_matrix(dh['a'], dh['alpha'], dh['d'], theta_dh)
        Z.append(T[0:3,2].copy())
        O.append(T[0:3,3].copy())
    # include tool offset
    T = T @ dh_matrix(DH_TABLE_5DOF[4]['a'], DH_TABLE_5DOF[4]['alpha'], DH_TABLE_5DOF[4]['d'], 0)
    O_e = T[:3,3].copy()
    J = np.zeros((3,4), dtype=float)
    for i in range(4):
        J[:,i] = np.cross(Z[i], (O_e - O[i]))
    return J

# -----------------------------
# Robust Damped Least Squares IK
# -----------------------------
def ik_dls_line_search(target_cm, theta0_deg_4, max_iters=600, tol_cm=0.4,
                       lambda0=0.1, lambda_min=1e-6, lambda_max=1e4,
                       max_step_deg=10.0, verbose=False):
    theta = np.array(theta0_deg_4, dtype=float)
    lam = float(lambda0)
    target = np.array(target_cm, dtype=float)

    for it in range(max_iters):
        current_pos = forward_kinematics_deg(theta)
        err_vec = target - current_pos
        err_norm = np.linalg.norm(err_vec)
        if err_norm < tol_cm:
            return {'success': True, 'theta_deg_4': theta, 'theta5': FIXED_5TH,
                    'final_pos': current_pos, 'error_norm': err_norm, 'iters': it}
        J = analytic_jacobian_deg(theta)
        JTJ = J.T @ J
        A = JTJ + (lam**2) * np.eye(4)
        rhs = J.T @ err_vec
        delta_theta_rad = np.linalg.solve(A, rhs)
        delta_deg = rad2deg(delta_theta_rad)
        max_mag = np.max(np.abs(delta_deg))
        if max_mag > max_step_deg:
            delta_deg = delta_deg * (max_step_deg / max_mag)
        theta += delta_deg
        # enforce joint limits
        for j in range(4):
            low, high = JOINT_LIMITS_4DOF[j]
            theta[j] = np.clip(theta[j], low, high)
    final_pos = forward_kinematics_deg(theta)
    return {'success': False, 'theta_deg_4': theta, 'theta5': FIXED_5TH,
            'final_pos': final_pos, 'error_norm': np.linalg.norm(target - final_pos),
            'iters': max_iters}

# -----------------------------
# Quick Test
# -----------------------------
if __name__ == "__main__":
    target = np.array([10.0, 0.0, 5.0])  # cm
    theta0 = [90, 50, 10, 135]
    res = ik_dls_line_search(target, theta0, verbose=True)
    print("Success:", res['success'])
    print("Theta (deg):", res['theta_deg_4'], "Theta5:", res['theta5'])
    print("FK check (cm):", forward_kinematics_deg(res['theta_deg_4']))
