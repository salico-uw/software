# Debugging: use an ideal image
import cv2

def detect_and_draw(img_path, arucoParams):
  img = cv2.imread(img_path, cv2.IMREAD_COLOR)

  aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

  detector = cv2.aruco.ArucoDetector(aruco_dict, arucoParams)

  (corners, ids, rejected) = detector.detectMarkers(img)

  output_image = cv2.aruco.drawDetectedMarkers(img, corners, ids, borderColor=(255, 0, 0))

  cv2.imwrite("../output.png", output_image)
  return (img, corners)


arucoParams = cv2.aruco.DetectorParameters()
# arucoParams.markerBorderBits = 2
arucoParams.adaptiveThreshWinSizeStep = 1
# Output the detected corners in a new imag
(img, corners) = detect_and_draw("../img/apriltags_30mm.png", arucoParams)
print(corners)