#!/usr/bin/env python
import cv2
import math
import time
import copy
from matplotlib import pyplot as plt
from numpy.linalg import inv
import roslib
import rospy
from geometry_msgs.msg import Pose2D
import numpy as np;
 
# Read image

hd_cam = 0

cap = cv2.VideoCapture(hd_cam)

#fourcc = cv2.VideoWriter_fourcc(*'DIVX')
#out = cv2.VideoWriter('QR_tracking.avi',fourcc, 10.0, (1280,720))

cap.set(3,1280)
cap.set(4,720)
cap.set(12,0.64)
cap.set(16,1.0)
if hd_cam == 1:
	cap.set(3,1280)
	cap.set(4,720)

cap.set(10,0.5)

# initialization for template matching
template = cv2.imread('/home/bare/catkin_ros/src/shop-robo/qr_tracker/src/qr_match.png', 0)

w, h = template.shape[::-1]
template_2 = template.copy()

### KALMAN FILTER initializations ###
Ts = 0.02

Pk = np.eye(4,k=0,dtype=float)*100

xk_ = np.transpose(np.matrix([1280/2,720/2,0,0]))
xk = xk_

FI = np.matrix([[1,0,Ts,0],
		[0,1,0,Ts],
		[0,0,1,0],
		[0,0,0,1]])

H = np.matrix([	[1,0,0,0],
		[0,1,0,0]])

Q = np.eye(4,k=0,dtype=float)*1

R = np.eye(2,k=0,dtype=float)*1
 
last_yk = np.matrix([640.0, 360.0])
yk = np.matrix([640.0, 360.0])

last_qr_center = []
qr_center = []
qr_template_coor = []
qr_template_coor_last = []
template_vel_diff = [0,0]
scale = 1
i = 0
im_avg = 0

# initialization of pose2d msg 
qr_pose2d = Pose2D()

while not rospy.is_shutdown():

    	pub = rospy.Publisher('qr_pose2d', Pose2D, queue_size=0)
    	rospy.init_node('qr_pose2d', anonymous=True)
    	rate = rospy.Rate(int(1/Ts)) # 50hz

	#i = i+1

	### frame grabbing and preprocessing ###	

	# take frame from camera
	_, im = cap.read()


	# copy original frame for KF result representation
	im2 = im.copy()

	# filter image a
	img = cv2.medianBlur(im,5)
	
	# convert BGR to GRAY
    	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

	# smooth frame with gaussian filter - noise compensation
	img = cv2.GaussianBlur(gray,(7,7),2)


	# threshold image to get black and white image
	ret_simple, th_simple = cv2.threshold(img,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)	# try experimenting with threshold value ; e.g. >200

	kernel = np.ones((5,5),np.uint8)
#	erosion = cv2.erode(th_simple,kernel,iterations = 1)

	closing = cv2.morphologyEx(th_simple, cv2.MORPH_CLOSE, kernel)
	th_simple = closing

#	cv2.imshow('dilate',closing)

	# template resizing
	template_2 = cv2.resize(template,None,fx=scale, fy=scale, interpolation = cv2.INTER_LINEAR)
	w, h = template_2.shape[::-1]
#	cv2.imshow('template',template_2)	

	### QR-code marker detection ###
	# find contours in the image
	_, contours, hierarchy = cv2.findContours(th_simple,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	
	centroids = []
	
	# get centroids of all contours
	for i in range(0,len(contours)-1):
		cnt = contours[i]
		M = cv2.moments(cnt)
		if M['m00'] != 0:
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
#			cv2.circle(im,(cx,cy), 2, (0,0,255), -1)
			centroids.append([cx,cy])
		
	qr_pose = []
	cont_no = []

	# find centroids of QR markers : QR-code markers consists of concetrical squares => their contours have same centroids
	for i in range(0,len(centroids)-1):
		M = cv2.moments(contours[i])
		for j in range(0,len(centroids)-1):
			if i != j:
				if (centroids[i][0] == centroids[j][0] and centroids[i][1] == centroids[j][1]):
					qr_pose.append([centroids[i],i])
					cont_no.append(i)

	max_dist = math.sqrt(pow(1280-0,2) + pow(720-0,2))
	lu = [1280,720]
	ru = [0,720]
	ld = [1280,0]

	### determine the position of the markers in respect to the QR-code ###
	# qr marker extrapolation 		
	for i in range(0,len(qr_pose)):
		if math.sqrt(pow(qr_pose[i][0][0]-0,2) + pow(qr_pose[i][0][1]-0,2)) < math.sqrt(pow(0-lu[0],2) + pow(0-lu[1],2)):
			lu = [qr_pose[i][0][0],qr_pose[i][0][1]]

		if math.sqrt(pow(qr_pose[i][0][0]-1280,2) + pow(qr_pose[i][0][1]-0,2)) < math.sqrt(pow(1280-ru[0],2) + pow(0-ru[1],2)):
			ru = [qr_pose[i][0][0],qr_pose[i][0][1]]

		if math.sqrt(pow(qr_pose[i][0][0]-0,2) + pow(qr_pose[i][0][1]-720,2)) < math.sqrt(pow(0-ld[0],2) + pow(720-ld[1],2)):
			ld = [qr_pose[i][0][0],qr_pose[i][0][1]]
	
	# image averaging
	im_avg = im2*10
	im_avg[int(xk[1,0]-h):int(xk[1,0]+h),int(xk[0,0]-w):int(xk[0,0]+w),:] = im2[int(xk[1,0]-h):int(xk[1,0]+h),int(xk[0,0]-w):int(xk[0,0]+w),:] 


	# draw the position of the markers in the image
	if lu == ru or lu == ld:
		if (ru[0]-ld[0]) == 0:
			k = float('inf')
		else:
			k = abs(float((ru[1]-ld[1]))/float((ru[0]-ld[0])))

		if k <= 0.5:
			ld = [lu[0], lu[1]+int(math.sqrt(pow(lu[0]-ru[0],2) + pow(lu[1]-ru[1],2)))]
		elif k >= 5:
			ru = [lu[0] + int(math.sqrt(pow(lu[0]-ld[0],2) + pow(lu[1]-ld[1],2))), lu[1]]
		elif k > 0.5 and k < 5:
			lu = [ld[0], ru[1]]
	if lu == ru and ru == ld:
		im_avg[int(lu[1]-h):int(lu[1]+h),int(lu[0]-w):int(lu[0]+w),:] = im2[int(lu[1]-h):int(lu[1]+h),int(lu[0]-w):int(lu[0]+w),:]

#	cv2.imshow('im_avg',im_avg)
	
	# filter image a
	im_avg = cv2.medianBlur(im_avg,5)

	# convert BGR to GRAY
	im_avg_gray = cv2.cvtColor(im_avg, cv2.COLOR_BGR2GRAY)

	
	# smooth frame with gaussian filter - noise compensation
	im_avg_blur = cv2.GaussianBlur(im_avg_gray,(7,7),2)

	
	### template match ###
	res = cv2.matchTemplate(im_avg_blur,template_2,5)
	min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
#	print max_val
#	cv2.imshow('template',res)
#	cv2.imshow('img',im_avg_blur)
	top_left = max_loc
	bottom_right = (top_left[0] + w, top_left[1] + h)
	cv2.rectangle(im,top_left, bottom_right, 255, 2)
	qr_template_coor_last = qr_template_coor
	qr_template_coor = [top_left[0] + w/2, top_left[1] + h/2]
	cv2.circle(im,(top_left[0] + w/2,top_left[1] + h/2), 6, (0,0,255), -1)
	if qr_template_coor_last != []:
		template_vel_diff =  [qr_template_coor_last[0] - qr_template_coor[0] + template_vel_diff[0], qr_template_coor_last[1] - qr_template_coor[1] + template_vel_diff[1]]
#		print template_vel_diff

	
	if lu[0] != 1280 and lu[1] != 720:
		cv2.circle(im,(lu[0],lu[1]), 6, (0,0,255), -1)

	if ru[0] != 0 and ru[1] != 720 and ru != lu:
		cv2.circle(im,(ru[0],ru[1]), 6, (0,0,255), -1)

	if ld[0] != 1280 and ld[1] != 0 and ld != lu and ld != ru:
		cv2.circle(im,(ld[0],ld[1]), 6, (0,0,255), -1)	

	if lu[0] != 1280 and lu[1] != 720 and ru[0] != 0 and ru[1] != 720 and ld[0] != 1280 and ld[1] != 0:
		cv2.line(im,(lu[0],lu[1]),(ru[0],ru[1]),(0,255,0),2)
		cv2.line(im,(lu[0],lu[1]),(ld[0],ld[1]),(0,255,0),2)
		cv2.line(im,(ld[0],ld[1]),(ru[0],ru[1]),(0,255,0),2)

	# check if 3 unique markers are detected
	if lu != ru and lu != ld and ru != ld:
		qr_markers_3 = True
	else:
		qr_markers_3 = False

	# find the centroid of the QR-code based on the found markers
	if lu != [] and ru != [] and ld != []:
		qr_area = abs(ld[1] - ru[1])*abs(ld[0] - ru[0])
		qr_a = pow(qr_area,0.5)
#		print float(qr_area)/float((1280*720))*100, '%'
		last_qr_center = qr_center
		qr_center = [abs(ld[0] + ru[0])/2 , abs(ld[1] + ru[1])/2]
		cv2.circle(im,(qr_center[0],qr_center[1]), 6, (255,0,0),-1)
	
	# display the result
#	cv2.imshow('threshold', th_simple)
	cv2.imshow('result without KF',im)
	
	if last_qr_center == []:
		continue

	### KALMAN FILTER : 2D QR-code tracking ###
	
	# prediction based on model 
	xk_ = np.dot(FI, xk_)
	Pk = np.dot(FI, np.dot(Pk, FI.transpose())) + Q
	Kk = np.dot(Pk, np.dot(H.transpose(), inv(np.dot(H, np.dot(Pk, H.transpose())) + R)))

	## Additional qr-marker algorithm used for measurements ##

	
	
	##########################################################

	# measurement
	if lu != ld and lu != ru and lu != [1280,720] and ru != [0,720] and ld != [1280,0]:	
		last_yk = yk
		yk = np.matrix([float(qr_center[0]), float(qr_center[1])])
		scale = (float(qr_a)/float(200))*1.5
		R = np.eye(2,k=0,dtype=float)*1
		print (math.sqrt(pow(lu[0]-ru[0],2) + pow(lu[1]-ru[1],2)))/(math.sqrt(pow(lu[0]-ld[0],2) + pow(lu[1]-ld[1],2)))
		if abs((math.sqrt(pow(lu[0]-ru[0],2) + pow(lu[1]-ru[1],2)))/(math.sqrt(pow(lu[0]-ld[0],2) + pow(lu[1]-ld[1],2)))) <= 1:
			qr_pose2d.theta = (180/3.14)*math.acos((math.sqrt(pow(lu[0]-ru[0],2) + pow(lu[1]-ru[1],2)))/(math.sqrt(pow(lu[0]-ld[0],2) + pow(lu[1]-ld[1],2))))
		
	elif abs(template_vel_diff[0]) < 100 and abs(template_vel_diff[1]) < 100:
		last_yk = yk
		yk = np.matrix([float(qr_template_coor[0]), float(qr_template_coor[1])])
		R = np.eye(2,k=0,dtype=float)*10
	else:
		last_yk = yk
		yk = np.transpose(np.dot(H, xk_))


	# korection based on measurement
	xk = xk_ + np.dot(Kk, (np.transpose(yk) - np.dot(H, xk_)))
	Pk = np.dot((np.eye(4,k=0,dtype=float) - np.dot(Kk,H)), Pk)
	xk_[2,0] = (yk[0,0]-last_yk[0,0])/1
	xk_[3,0] = (yk[0,1]-last_yk[0,1])/1
	xk_[0,0] = xk[0,0]
	xk_[1,0] = xk[1,0]

#	print "KF center: ", [xk[0,0], xk[1,0]]
#	print "measured center: ", qr_center
#	print xk


#	cv2.circle(im2,(qr_center[0],qr_center[1]), 8, (0,255,255), -1)
	cv2.circle(im2,(int(xk[0,0]),int(xk[1,0])), 20, (255,0,255), -1)

	if int(xk[2,0]) > 0:
		cv2.line(im2,(int(xk[0,0]),int(xk[1,0])),(int(xk[2,0])+int(xk[0,0]),int(xk[1,0])),(0,255,0),2)
	else:
		cv2.line(im2,(int(xk[0,0]),int(xk[1,0])),(int(xk[2,0])+int(xk[0,0]),int(xk[1,0])),(0,0,255),2)
	if int(xk[3,0]) > 0:
		cv2.line(im2,(int(xk[0,0]),int(xk[1,0])),(int(xk[0,0]),int(xk[3,0])+int(xk[1,0])),(0,255,0),2)
	else:
		cv2.line(im2,(int(xk[0,0]),int(xk[1,0])),(int(xk[0,0]),int(xk[3,0])+int(xk[1,0])),(0,0,255),2)

	cv2.ellipse(img,(int(xk[0,0]),int(xk[1,0])),(int(Pk[0,0]),int(Pk[1,1])),0,0,360,(255,0,255),2)
	cv2.imshow('result with KF',im2)
#	out.write(im2)

	# fill and publish 2D pose msg of QR code
	qr_pose2d.x = xk[0,0]
	qr_pose2d.y = xk[1,0]
#	qr_pose2d.theta = 0
	pub.publish(qr_pose2d)
        rate.sleep()

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()
