#!/usr/bin/env python
# run as:
# python faceTracker.py haarcascade_frontalface_default.xml

import cv2
import sys
import ach
import controller_include as ci
import time

cascPath = sys.argv[1]
faceCascade = cv2.CascadeClassifier(cascPath)

video_capture = cv2.VideoCapture(1)

ret, frame = video_capture.read()
height, width, depth = frame.shape
print 'H = ', height, ' W = ', width

e = ach.Channel(ci.CONTROLLER_REF_NAME)
e.flush()
controller = ci.CONTROLLER_REF()


while True:
    # Capture frame-by-frame
    ret, frame = video_capture.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30),
        flags=cv2.cv.CV_HAAR_SCALE_IMAGE
    )
    
    # No face = zero error
    x = 0
    y = 0

    # Draw a rectangle around the faces
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        x = int(x + w/2)
        y = int(y + h/2)
        
        cv2.circle(frame,(x,y),2,(0,255,0),-1)
        
        # get the errors from the center:
        x = int(x - (width/2))
        y = int(y - (height/2))
        break; # recognize only the first object

    # Display the resulting frame
    cv2.imshow('Video', frame)
    
    print '\nError in x & y: ', x, '\t', y
    
    # send the error to controller
    controller.x = x
    controller.y = y
    e.put(controller)
    
    time.sleep(0.01)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture
video_capture.release()
cv2.destroyAllWindows()
