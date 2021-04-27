"""
*****************************************************************************************************
                            SECTION-B
SUBSECTION B.3
Q) OpenCV is the most used image processing library on the planet! So it
   is a crime to not include any questions from the same topic!

 1) Use this image of an IGVC Track. Find a way to draw bounding boxes,
    only to the obstacle which is orange-white in color. Using Histogram
    Backprojection method given in OpenCV

ROI = Region of interest
target = Image in which we want to detect objects

****************************************************************************************************"""


import numpy as np
import cv2 as cv


roi = cv.imread(
    r"C:\Users\Admin\Desktop\ML_Projects\Image rec\images\base_obj.png"
)  # Input images of object we want to detect


# Validating Image Path
try:
    hsv = cv.cvtColor(roi, cv.COLOR_BGR2HSV)
except:
    print("\nInvalid object Image Path")
    print(
        r"Try changing images\object.png to images\\object.png or Convert it to RAW string"
    )
    exit()


target = cv.imread(
    r"C:\Users\Admin\Desktop\ML_Projects\Image rec\images\abhiyan_appl.png"
)  # Input target Image


# Validating Image Path
try:
    hsvt = cv.cvtColor(target, cv.COLOR_BGR2HSV)
except:
    print("\nInvalid Target Image Path")
    print(
        r"Try changing images\object.png to images\\object.png or Convert it to RAW string"
    )
    exit()

# calculating object histogram
roihist = cv.calcHist([hsv], [0, 1], None, [180, 256], [0, 180, 0, 256])

# normalize histogram and apply backprojection
cv.normalize(roihist, roihist, 0, 255, cv.NORM_MINMAX)
dst = cv.calcBackProject([hsvt], [0, 1], roihist, [0, 180, 0, 256], 1)

# Now convolute with circular disc
disc = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
cv.filter2D(dst, -1, disc, dst)


# threshold and binary AND
ret, thresh = cv.threshold(dst, 50, 255, 0)
thresh = cv.merge((thresh, thresh, thresh))

# Overlapping Original image and detected parts
res = cv.bitwise_and(target, thresh)
cv.imshow("Objects Detected", res)

# Now that the image parts are detected we have to draw bounding boxes

# Threshold image to draw contours
gray = cv.cvtColor(res, cv.COLOR_BGR2GRAY)
blur = cv.GaussianBlur(gray, (5, 5), 0)
_, thresh = cv.threshold(blur, 20, 255, cv.THRESH_BINARY)

# Assigning Kernel size
# Kernel size assigned such that dilation more occurs in vertical direction than horizontal direction
# So that, objects which are close will not get detected as one
kernel = cv.getStructuringElement(cv.MORPH_RECT, ksize=(1, 5))
dilated = cv.dilate(thresh, kernel, iterations=3)
cv.imshow("Dilated Image", dilated)

# Getting contours
contours, _ = cv.findContours(dilated, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

for contour in contours:
    (x, y, w, h) = cv.boundingRect(contour)

    # Drawing boxes neglecting some minor contours (if persent)
    if cv.contourArea(contour) < 250:
        continue
    cv.rectangle(target, (x, y), (x + w, y + h), (0, 255, 0), 2)


print(len(contours))
cv.imshow("Final", target)
cv.imwrite("Result_image.jpg", target)

cv.waitKey(0)
