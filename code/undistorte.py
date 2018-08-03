import cv2
import sys
import numpy as np

#K=np.array([[206.26480624709637, 0.0, 323.5], [0.0, 206.26480624709637, 242.5], [0.0, 0.0, 1.0]])
#DIM=(648, 486)
#D = np.array([[0.2630630021513974], [0.05010380465347993], [0.030779363074071555], [0.008559784664570531]])

DIM=(1296, 972)
K=np.array([[3060.312632089803, 0.0, 604.3141174341204], [0.0, 3072.872190303729, 580.3611977255326], [0.0, 0.0, 1.0]])
D=np.array([[-12.668944998435986], [448.42917866848154], [-2644.667219660557], [-87931.31321309994]])

def undistort(img_path, balance=0.0, dim2=None, dim3=None):

    img = cv2.imread(img_path)
    print(img.shape)
    dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort

    assert dim1[0]/dim1[1] == DIM[0]/DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"

    if not dim2:
        dim2 = dim1

    if not dim3:
        dim3 = dim1

    scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0

    # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    cv2.imshow("undistorted", undistorted_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    undistort(sys.argv[1], float(sys.argv[2]))
