"""
A tracker class for controlling the Tello and some sample code for showing how
it works. you can test it using your webcam or a video file to make sure it works.

it computes a vector of the ball's direction from the center of the
screen. The axes are shown below (assuming a frame width and height of 600x400):

Based on the tutorial:
https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/


@author Leonie Buckley and Jonathan Byrne
@copyright 2018 see license file for details
"""

import argparse
import time
import cv2
import imutils
from imutils.video import VideoStream


class Tracker:
    """
    A basic color tracker, it will look for colors in a range and
    create an x and y offset value from the midpoint
    """

    def __init__(self, height, width, color_lower, color_upper):
        self.color_lower = color_lower
        self.color_upper = color_upper
        self.midx = int(width / 2)
        self.midy = int(height / 2)
        self.object_area = width * height
        self.xoffset = 0
        self.yoffset = 0

    def draw_arrows(self, frame):
        """Show the direction vector output in the cv2 window"""
        #cv2.putText(frame,"Color:", (0, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, thickness=2)
        cv2.arrowedLine(frame, (self.midx, self.midy),
                        (self.midx + self.xoffset, self.midy - self.yoffset),
                        (0, 0, 255), 5)
        return frame

    def track(self, frame):
        """Simple HSV color space tracking"""
        # resize the frame, blur it, and convert it to the HSV
        # color space
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color then perform
        # a series of dilation and erosion to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0]
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

                self.xoffset = int(center[0] - self.midx)
                self.yoffset = int(self.midy - center[1])
            else:
                self.xoffset = 0
                self.yoffset = 0
        else:
            self.xoffset = 0
            self.yoffset = 0
        return self.xoffset, self.yoffset, self.object_area
