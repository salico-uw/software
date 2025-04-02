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

Y_UPPER_LIMIT = 310
Y_LOWER_LIMIT = 250

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
    # print(coords)
    # [-1][-1] gets the last 
    y_coords = sorted([corner[1] for corner in coords[-1][-1]])

    y_low = (y_coords[0] + y_coords[1]) / 2
    y_high = (y_coords[2] + y_coords[3]) / 2

    print(y_low, y_high)
    # print(y_coords)
    # are we within the limit?
    if y_low < Y_UPPER_LIMIT and y_low > Y_LOWER_LIMIT and y_high < Y_UPPER_LIMIT and y_high > Y_LOWER_LIMIT:
      output_char = "1"
      print("In range")
      
    elif y_low <= Y_LOWER_LIMIT and y_high <= Y_UPPER_LIMIT and y_high >= Y_LOWER_LIMIT:
      output_char = "+"
      print("Move Forward")

    elif y_high >= Y_UPPER_LIMIT and y_low <= Y_UPPER_LIMIT and y_low >= Y_LOWER_LIMIT:
      output_char = "-"
      print("Move Backwards")
    
    else:
      output_char = "0"
      print("Tags detected, but none within bounds")
      
    # if y_coords[1] < Y_LOWER_LIMIT and Y:
    #   output_char = "+"
    #   print("Move Forward")
    # elif y_coords[-1] > Y_UPPER_LIMIT and:
    #   output_char = "-"
    #   print("Move Backwards")
    # elif :
    #   output_char = "1"
    #   print("In range")
     
  with open(OUTPUT_PATH, "w") as f:
    f.write(output_char)

  time.sleep(0.2)
