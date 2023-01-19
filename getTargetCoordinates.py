import numpy as np
import cv2
import sys
import time
import serial


ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}
def encode_pitch_yaw_vel(pitch, yaw, vel):
    p = str(int(pitch))
    y = str(int(yaw))
    v = str(int(vel))
    return str(len(p)) + p + str(len(y)) + y + str(len(v)) + v 

def z_rotangle(x,y,depth):
    #xy coordinates refer to our vision looking forward where z is just the depth
    return np.arctan(x/depth)
def velocity_anglecalculator(x,height,depthz):
    depth = np.sqrt(x**2 + depthz**2)
    theta = np.arctan(-1*depth/height)/2
    theta +=np.pi/2
    velocity = np.sqrt(9.8*depth*np.tan(theta))
    angle_yaxis = theta
    return angle_yaxis, velocity

def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict, parameters=parameters) # , cameraMatrix=matrix_coefficients, distCoeff=distortion_coefficients

    last_r = 0
    last_t = 0
        
    if len(corners) > 0:
        for i in range(0, len(ids)):
           
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients, distortion_coefficients)
            last_r, last_t = rvec, tvec
            print("last_r: ", last_r)
            cv2.aruco.drawDetectedMarkers(frame, corners) 
            print("rotation: ", rvec, "translation: ", tvec, "\n") 
            # store pose somwhow on this line.

            img = cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  
    print("last_r: ", last_r)
    return frame, last_r, last_t


    

aruco_type = "DICT_5X5_250"

arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])

arucoParams = cv2.aruco.DetectorParameters_create()


intrinsic_camera = np.array(((2251.11452, 0.0, 848.670891),(0.0, 2242.63879, 341.392068),(0.0, 0.0, 1.0)))
distortion = np.array((0.0,0.0,0,0))


cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
 


while cap.isOpened():
    
    ret, img = cap.read()
    
    output, rvec, tvec = pose_estimation(img, ARUCO_DICT[aruco_type], intrinsic_camera, distortion)

    cv2.imshow('Estimated Pose', output)
    lastrvec, lasttvec = rvec, tvec

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("\n\n\n\n\n\n\n\n\n")
print("mevement details")

x_coord = lasttvec[0][0][0]
y_coord = -1*lasttvec[0][0][1]
z_coord = -1*lasttvec[0][0][2]
print("last_tvec: \n")
print("x: ", x_coord)
print("y: ", y_coord)
print("z: ", z_coord)

print("\n\n")
print("Planned motion: ")

z_rot = z_rotangle(x_coord, y_coord, z_coord)
y_rot, launch_vel = velocity_anglecalculator(x_coord, y_coord, z_coord)
print("yaw angle(rad): ", z_rot)
print("pitch(rad): ", y_rot)
print("yaw angle(deg): ", z_rot * (180 / np.pi)) 
print("pitch(deg): ", y_rot * (180 / np.pi))
print("launch velocity: ", launch_vel)

pitch_deg = y_rot * (180 / np.pi)
yaw_deg =  z_rot * (180 / np.pi)
# send through serial. 

arduinoData = serial.Serial('COM7', 9600)
time.sleep(1)
msg = encode_pitch_yaw_vel(pitch_deg, yaw_deg, launch_vel)
while True:
    input("Press Enter to send")
    user_cmd = msg
    user_cmd = user_cmd + '\r'
    print("before: ", user_cmd)
    arduinoData.write(user_cmd.encode())
    print("sent command", user_cmd)