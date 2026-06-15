import cv2 as cv
import numpy as np
import glob
import pickle

# Chessboard dimensions (inner corners)
CHECKERBOARD = (9, 6)

# Stop criteria for corner refinement
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare 3D points (0,0,0 ... 8,5,0)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

objpoints = []  # 3D points in real world
imgpoints = []  # 2D points in image plane

images = glob.glob('images/*.png') + glob.glob('images/*.jpg')

print(f"Found {len(images)} images")

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    ret, corners = cv.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        cv.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv.imshow('Corners', img)
        cv.waitKey(200)
    else:
        print(f"Chessboard not detected in {fname}")

cv.destroyAllWindows()

if len(objpoints) > 0:
    ret, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    print("Calibration successful!")
    print("Camera matrix:\n", cameraMatrix)
    print("Distortion coefficients:\n", dist)

    # Save results
    with open("cameraMatrix.pkl", "wb") as f:
        pickle.dump(cameraMatrix, f)
    with open("dist.pkl", "wb") as f:
        pickle.dump(dist, f)
else:
    print("❌ No chessboard patterns detected! Try taking clearer pictures.")
