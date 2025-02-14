# Debugging: use an ideal image
import cv2
import time

def detect_tag_corners(img_path, arucoParams):
  img = cv2.imread(img_path, cv2.IMREAD_COLOR)

  aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

  detector = cv2.aruco.ArucoDetector(aruco_dict, arucoParams)

  (coords, ids, rejected) = detector.detectMarkers(img)

  output_image = cv2.aruco.drawDetectedMarkers(img, coords, ids, borderColor=(255, 0, 0))

  return (img, coords)

webcam_path = "/home/salico/salico/webcam.png"

arucoParams = cv2.aruco.DetectorParameters()
# arucoParams.markerBorderBits = 2
arucoParams.adaptiveThreshWinSizeStep = 1
# Output the detected corners in a new imag

# Corners are outputted in order: bottom right, bottom left, top left, top right

Y_UPPER_LIMIT = 300
Y_LOWER_LIMIT = 60

OUTPUT_PATH = "/home/salico/salico/am_i_in_range.txt"

while True:
  try:
    (img, coords) = detect_tag_corners(webcam_path, arucoParams)
  except:
    print("could not detect marker, perhaps webcam not found")
    time.sleep(1)
    continue
  output_char = "0"
  y_coords = [0, 0]
  if len(coords) == 0:
    print("No marker found.")
  else:
    y_coords = sorted([corner[1] for corner in coords[0][0]])
    # min, max
    print(y_coords[0], y_coords[-1])
    # are we within the limit?
    if Y_LOWER_LIMIT > y_coords[0]:
      output_char = "↑"
      print("Move Forward")
    elif Y_UPPER_LIMIT < y_coords[-1]:
      output_char = "↓"
      print("Move Backwards")
    else:
      output_char = "-"
      print("In range")
     
  with open(OUTPUT_PATH, "w") as f:
    f.write(output_char)

  time.sleep(1)
