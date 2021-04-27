"""
*************************************************************************************************************
                            SECTION-B
SUBSECTION B.3
Q) OpenCV is the most used image processing library on the planet! So it
   is a crime to not include any questions from the same topic!
   
  2) Track the moving pedestrians in the video given, by drawing a
     bounding box around them

Ans explanation:
  Basic idea behind detecting movement is to find the differences between two consecutive frames and locating contours

*************************************************************************************************************"""


import cv2 as cv
import numpy as np

# Capturing video
# Enter a valid Image Path
# Try to give Input as RAW string
cap = cv.VideoCapture(
    r"C:\Users\Admin\Desktop\ML_Projects\Image rec\videos\pedestrians.mp4"
)  # Input your Video
frame_width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))

# For exporting frames into video
fourcc = cv.VideoWriter_fourcc("X", "V", "I", "D")
out = cv.VideoWriter("test_case1.mp4", fourcc, 5.0, (1280, 720))

# Reading first two frames
ret, frame1 = cap.read()
ret, frame2 = cap.read()

# Loop will run untill last frame
while cap.isOpened():
    # Finding the difference between two frames
    diff = cv.absdiff(frame1, frame2)
    # Converting into grayscale
    gray = cv.cvtColor(diff, cv.COLOR_BGR2GRAY)
    # Blurring image to reduce noise
    blur = cv.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv.threshold(blur, 20, 255, cv.THRESH_BINARY)
    dilated = cv.dilate(thresh, None, iterations=3)
    # Finding the location of contours
    contours, _ = cv.findContours(dilated, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    # Loop to draw boxes around contours in frames based on the input of difference between two frames
    for contour in contours:
        (x, y, w, h) = cv.boundingRect(contour)
        # Neglecting minor contours (if present)
        if cv.contourArea(contour) < 450:
            continue
        cv.rectangle(frame1, (x, y), (x + w, y + h), (0, 255, 0), 2)

    image = cv.resize(frame1, (1280, 720))
    # Exporting frame to th output video
    out.write(image)
    cv.imshow("Tracking Output", frame1)
    # Assigning the next frame that will run in the while loop
    frame1 = frame2
    # Shifting next frame by one frame
    ret, frame2 = cap.read()

    if cv.waitKey(40) == 27:
        break

cv.destroyAllWindows()
cap.release()
out.release()

