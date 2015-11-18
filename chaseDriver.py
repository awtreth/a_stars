class driver(object) 
	#This function accepts two wheel velocities and a time interval.
	def spinWheels(u1, u2, time):
   		global pub

    		lin_vel = (u1+u2)/2	#Determines the linear velocity of base based on the wheels
    		ang_vel = (u1-u2)/base	#Determines the angular velocity of base on the wheels.
   		twist_msg = Twist();	#Creates two messages: 
    		stop_msg = Twist();
    		twist_msg.linear.x = lin_vel	#Populate messages with data.
    		twist_msg.angular.z = ang_vel
   		stop_msg.linear.x = 0
    		stop_msg.angular.z = 0
    		#While the specified amount of time has not elapsed, send Twist messages.
    		now = rospy.Time.now().secs
    		while (rospy.Time.now().secs - now <= time and not rospy.is_shutdown()):
       			pub.publish(twist_msg)
    			pub.publish(stop_msg)

	#This function consumes linear and angular velocities
	#and creates a Twist message.  This message is then published.
	def publishTwist(lin_vel, ang_vel):
    		global pub

    		twist_msg = Twist();		#Create Twist Message
    		twist_msg.linear.x = lin_vel	#Populate message with data
    		twist_msg.angular.z = ang_vel
    		pub.publish(twist_msg)		#Send Message

	def driveStraight(speed, distance):
    		global odom_list
   		global pose 

   		x0 = pose.pose.position.x	#Set origin
    		y0 = pose.pose.position.y
    		done = False
    		while (not done and not rospy.is_shutdown()):
        		x1 = pose.pose.position.x
        		y1 = pose.pose.position.y
        		d = math.sqrt(math.pow(x1-x0, 2) + math.pow(y1-y0, 2))	#Distance formula
 	
 			if (d >= distance):	# if d is greaterthan or equal to distance val, 
						# make op done. 
           			done = True
            			publishTwist(speed, 0)
			else:			# else publish the movement!
	    			publishTwist(speed, 0)

	#Accepts an angle and makes the robot rotate around it.
	def rotate(angle):
    		global odom_list
    		global pose

    		#This node was created using Coordinate system transforms and numpy arrays.
   		#The goal is measured in the turtlebot's frame, transformed to the odom.frame 
    		transformer = tf.TransformerROS()	
    		rotation = numpy.array([[math.cos(angle), -math.sin(angle), 0],	#Create goal rotation
                            [math.sin(angle), math.cos(angle), 0],
                            [0,          0,          1]])

    		#Get transforms for frames
    		odom_list.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(3.0))
    		(trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
    		T_o_t = transformer.fromTranslationRotation(trans, rot)
   		R_o_t = T_o_t[0:3,0:3]

    		#Setup goal matrix
    		goal_rot = numpy.dot(rotation, R_o_t)
    		goal_o = numpy.array([[goal_rot[0,0], goal_rot[0,1], goal_rot[0,2], T_o_t[0,3]],
                    [goal_rot[1,0], goal_rot[1,1], goal_rot[1,2], T_o_t[1,3]],
                    [goal_rot[2,0], goal_rot[2,1], goal_rot[2,2], T_o_t[2,3]],
                    [0,             0,             0,             1]])

    		#Continues creating and matching coordinate transforms.
    		done = False
    		while (not done and not rospy.is_shutdown()):
        		(trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        		state = transformer.fromTranslationRotation(trans, rot)
        		within_tolerance = abs((state - goal_o)) < .75
        		if ( within_tolerance.all() ):
            			spinWheels(0,0,0)
            			done = True
        		else:
            			if (angle > 0):
                			spinWheels(.1,-.1,.1)
            			else:
                			spinWheels(-.1,.1,.1)
	def readOdom(msg):
    		global pose
    		global odom_tf

    		pose = msg.pose
    		geo_quat = pose.pose.orientation
  		#This next segment takes the x, y, and z orientations of the robot current pose
    		odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0),
       		(pose.pose.orientation.x,
		pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w),
		rospy.Time.now(),"base_footprint","odom")
