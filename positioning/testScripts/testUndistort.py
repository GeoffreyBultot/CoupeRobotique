import cv2
import json
from picamera.array import PiRGBArray
from picamera import PiCamera
from cv2 import aruco
import numpy as np

map1 = None
map2 = None



""" DIM=(1280, 960) #good by Geoff
K=np.array([[630.932402116786, 0.0, 585.6531301759157], [0.0, 631.6869826709609, 478.8413904560236], [0.0, 0.0, 1.0]])
D=np.array([[-0.06670587491284909], [0.1057157290509116], [-0.13122001638126551], [0.04714118291127774]]) """


""" DIM=(1280, 960) #NEW CALIB 20/12 by Antho the GOD
K=np.array([[636.2973331204615, 0.0, 728.8166434876724], [0.0, 632.4666494131404, 460.8562085116319], [0.0, 0.0, 1.0]])
D=np.array([[-0.06005220364781312], [0.04832812120892501], [-0.04895075355487911], [0.017239151765319385]]) """

""" DIM=(2592, 1952)
K=np.array([[1271.6340818922563, 0.0, 1167.4678127068892], [0.0, 1267.583299646622, 938.5488313394765], [0.0, 0.0, 1.0]])
D=np.array([[-0.08022559999087736], [0.10435020556133874], [-0.11171079602705103], [0.03853140815187616]]) """

""" DIM=(1280, 960)
K=np.array([[614.0108742011905, 0.0, 709.5935590570987], [0.0, 613.5623922391759, 461.8950210905432], [0.0, 0.0, 1.0]])
D=np.array([[0.017020688212589168], [-0.23621049845164338], [0.46262833901215655], [-0.33009162778673434]]) """

#21-12
DIM=(1280, 960)
K=np.array([[612.6769659225217, 0.0, 573.732399040683], [0.0, 614.0511112656127, 464.15063277573313], [0.0, 0.0, 1.0]])
D=np.array([[-0.027945714136134205], [-0.012776430253932694], [0.00586270163443667], [-0.0015790193010345587]])




def initUndis():
    global map1
    global map2
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)

def undistort(img, balance=0, dim2=None, dim3=None):
    #img = cv2.resize(img,DIM)
    dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
    assert dim1[0]/dim1[1] == DIM[0]/DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    if not dim2:
        dim2 = dim1
    if not dim3:
        dim3 = dim1
    scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
    # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
    new_K = None #cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)#, borderMode=cv2.BORDER_CONSTANT)
    return(undistorted_img)



camera = PiCamera()
#camera.iso = 800 # max ISO to force exposure time to minimum to get less motion blur
#camera.contrast = 100
camera.resolution = DIM
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=camera.resolution)


if __name__ == '__main__':
    initUndis()
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100 )
    parameters =  aruco.DetectorParameters_create()

    for frame_pi in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        frame = frame_pi.array
        frame = undistort(frame)
        frame = cv2.resize(frame,(1280,720))
        gray = cv2.cvtColor((frame), cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        frame = aruco.drawDetectedMarkers(frame, corners)
        cv2.imshow("ENT",frame)

        rawCapture.truncate(0)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break