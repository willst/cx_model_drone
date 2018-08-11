import cv2
import sys
import numpy as np

K=np.array([[743.8040173335771, 0.0, 647.9940524434143], 
           [0.0, 728.8909350192504, 485.7950206412609], [0.0, 0.0, 1.0]])
D=np.array([[-0.20926662485054526], [-0.04800755535197234], 
           [0.26419146114701453], [-0.1540385750579161]])
DIM=(1296, 972)
UNDISTORTED_DIM = (200, 108)
scale = 2

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

    # crop images 
    fh_s = int(0.2377*dim1[1])
    fh_e = int(0.7377*dim1[1])
    fw_s = int(0.15*dim1[0])
    fw_e = int(0.85*dim1[0])
    undistorted_img = undistorted_img[fh_s:fh_e, fw_s:fw_e] 

    cv2.imshow('image', cv2.resize(undistorted_img, (0, 0), fx=scale, fy=scale))
    ch = 0xFF & cv2.waitKey(0)
    if ch == ord('q'):
        print "quit"
    elif ch == ord('s'):
        picture_name = "undistorted_image.jpg"
        cv2.imwrite(picture_name,undistorted_img)
        print 'Save image'

    cv2.destroyAllWindows()

if __name__ == '__main__':
    undistort(sys.argv[1], float(sys.argv[2]))
