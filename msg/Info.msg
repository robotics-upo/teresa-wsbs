Header header

uint8 status						# WAITING_FOR_START    = 0
							# WAITING_FOR_ODOM     = 1 
							# WAITING_FOR_LASER    = 2 
							# WAITING_FOR_XTION    = 3
							# WAITING_FOR_PEOPLE   = 4
							# RUNNING              = 5 
							# TARGET_LOST          = 6
							# FINISHED             = 7
							# ABORTED	       = 8

uint8 mode				        	# LEFT	      = 0
							# RIGHT	      = 1
							# BEHIND      = 2
							# FOLLOW_PATH = 3
							# WAIT        = 4 

bool target_detected					# True is the target is being detected
int32 target_id						# Target ID

geometry_msgs/Pose2D target_pose 			# measured by sensors
geometry_msgs/Pose2D robot_pose  			# measured by odometry

float64 target_lin_vel					# measured by the sensors
float64 robot_lin_vel   				# measured by odometry
float64 robot_ang_vel					# measured by odometry

geometry_msgs/Point robot_local_goal 			# calculated following the mode
bool robot_antimove 					# true if no exists a local goal for the robot


geometry_msgs/Vector3 target_group_force
geometry_msgs/Vector3 target_group_vis_force
geometry_msgs/Vector3 target_group_att_force
geometry_msgs/Vector3 target_group_rep_force


geometry_msgs/Vector3 robot_global_force
geometry_msgs/Vector3 robot_desired_force
geometry_msgs/Vector3 robot_obstacle_force
geometry_msgs/Vector3 robot_social_force
geometry_msgs/Vector3 robot_group_force
geometry_msgs/Vector3 robot_group_vis_force
geometry_msgs/Vector3 robot_group_att_force
geometry_msgs/Vector3 robot_group_rep_force
geometry_msgs/Vector3 robot_vref			# Velocity vector calculated by applying the forces

