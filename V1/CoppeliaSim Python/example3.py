#Import Libraries:
from sim import sim                 #V-rep library
import sys
import time                #used to keep track of time
import numpy as np         #array library
import math
import matplotlib as mpl   #used for image plotting
from plotter import RobotDataPlotter
from fuzzycontrol.oafcl import OAFLCControl
from fuzzycontrol.tflc import TFLCControl

class CoppeliaSimInterface(object):
	"""docstring for CoppeliaSimInterface"""
	def __init__(self):
		super(CoppeliaSimInterface, self).__init__()
		
		if self.start_connection():
			self.get_handlers()
			pass		
		pass
	
	def start_connection(self):
		sim.simxFinish(-1)
		self.clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5)
		#check if client connection successful
		if self.clientID!=-1:  
			print('Connected to remote API server')
			return True 	    
		else:
			print('Connection not successful') 
			sys.exit('Could not connect')
		pass

	def get_handlers(self):

		# get motor handler
		returnCode, self.w_hdl_1 = sim.simxGetObjectHandle(
			self.clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
		returnCode, self.w_hdl_2 = sim.simxGetObjectHandle(
			self.clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)

		# get ultrasound handler
		returnCode, self.ultsnd_h_1 = sim.simxGetObjectHandle(
			self.clientID, 'Pioneer_p3dx_ultrasonicSensor2', sim.simx_opmode_blocking)
		returnCode, self.ultsnd_h_2 = sim.simxGetObjectHandle(
		self.clientID, 'Pioneer_p3dx_ultrasonicSensor4', sim.simx_opmode_blocking) 
		returnCode, self.ultsnd_h_3 = sim.simxGetObjectHandle(
			self.clientID, 'Pioneer_p3dx_ultrasonicSensor5', sim.simx_opmode_blocking)

		# get robot and GOAL handler
		returnCode, self.rob_h = sim.simxGetObjectHandle(
			self.clientID, 'Pioneer_p3dx', sim.simx_opmode_blocking)
		returnCode, self.gol_h = sim.simxGetObjectHandle(
			self.clientID, 'GOAL', sim.simx_opmode_blocking)			
		pass

	def getting_position_error(self):

		# -----getting data-----
		# get robot pos and orientation
		returnCode, self.rob_poss = sim.simxGetObjectPosition(
				self.clientID, self.rob_h, -1, sim.simx_opmode_blocking)  # ret in meter, vector
		returnCode, self.rob_ortt = sim.simxGetObjectOrientation(
				self.clientID, self.rob_h, -1, sim.simx_opmode_blocking)  # ret in radian, vector
		
		# get goal pos and orientation
		returnCode, self.gol_poss = sim.simxGetObjectPosition(
				self.clientID, self.gol_h, -1, sim.simx_opmode_blocking)  # ret in meter, vector
		returnCode, self.gol_ortt = sim.simxGetObjectOrientation(
				self.clientID, self.gol_h, -1, sim.simx_opmode_blocking)  # ret in radian, vector
		
		# -----processing data-----
		
		# position goal error / vector magnitude, and orientation error
		# position error
		
		vec_pos = np.array(self.gol_poss) - np.array(self.rob_poss)
		self.pos_err = np.sqrt(np.sum(vec_pos ** 2))  # in meter
		
		# orientation error
		gamma_r = self.rob_ortt[2]
		gamma_g = self.gol_ortt[2]
		rob_x_2_g_or = np.array([[np.cos(gamma_r), np.sin(gamma_r)], [-np.sin(gamma_r), np.cos(gamma_r)]]) @ np.array([[np.cos(gamma_g), -np.sin(gamma_g)], [np.sin(gamma_g), np.cos(gamma_g)]]) @ np.array([1, 0])
		orr_err = np.arctan2(rob_x_2_g_or[1], rob_x_2_g_or[0])
		self.orr_err = np.rad2deg(orr_err)
		
		# Goal position relative to robot position to find heading error
		fTr = np.array([[np.cos(gamma_r), -np.sin(gamma_r), self.rob_poss[0]], [np.sin(gamma_r), np.cos(gamma_r), self.rob_poss[1]], [0, 0, 1]])
		gol_2_r_pos = np.linalg.inv(fTr) @ np.array([self.gol_poss[0], self.gol_poss[1], 1])
		
		# heading error
		hed_err = np.arctan2(gol_2_r_pos[1], gol_2_r_pos[0])
		self.hed_err = np.rad2deg(hed_err)

		
		return self.rob_poss, self.rob_ortt, self.pos_err, self.orr_err, self.hed_err 		
		pass

	def get_sensor_data(self):
		ultsnd_h = [self.ultsnd_h_1, self.ultsnd_h_2, self.ultsnd_h_3]
		detectedPoint = {}
		detectionState = np.zeros(3)
		for i in range(3):
			returnCode, detectionState[i], detectedPoint[i], detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
				self.clientID, ultsnd_h[i], sim.simx_opmode_blocking)
			#sim.simxResetProximitySensor(self.clientID, self.ultsnd_h[i], sim.simx_opmode_blocking)
		return detectedPoint, detectionState 
		pass

	def close(self):
		sim.simxFinish(self.clientID);

		returnCode=sim.simxSetJointTargetVelocity(
				self.clientID,self.w_hdl_1,0,sim.simx_opmode_blocking)
		returnCode=sim.simxSetJointTargetVelocity(
				self.clientID,self.w_hdl_2,0,sim.simx_opmode_blocking)	

		sim.delete()
		pass

	def move_pltaform(self, centeroid_TFLC_R, centeroid_TFLC_L):
		#left
		returnCode=sim.simxSetJointTargetVelocity(
			self.clientID, self.w_hdl_1, centeroid_TFLC_R, sim.simx_opmode_blocking);
		#right
		returnCode=sim.simxSetJointTargetVelocity(
			self.clientID, self.w_hdl_2, centeroid_TFLC_L, sim.simx_opmode_blocking);		
		pass





def main():
	interface = CoppeliaSimInterface()
	#plotter = RobotDataPlotter()
	control_traye = TFLCControl()
	control_obsta = OAFLCControl()
	ultsnd_masure_Zaxs = [400, 400, 400]
	while True:
		try:
			rob_poss, rob_ortt, pos_err, orr_err, hed_err = interface.getting_position_error()
			#print(rob_poss, rob_ortt, pos_err, orr_err, hed_err)
			detectedPoint, detectionState = interface.get_sensor_data()
			print(f"{detectedPoint}")
			print(f"{detectionState}")

			for i in range(3):
				if detectionState[i]:
					ultsnd_masure_Zaxs[i] = detectedPoint[i][2] * 100  # ret in centimeter, vector
				else:
					ultsnd_masure_Zaxs[i] = 400	

			print(ultsnd_masure_Zaxs)

			if min(ultsnd_masure_Zaxs) < 100:  # OBSTACLE DETECTED, OAFLC control
				print("OAFLC control routine")
				# Then actuate the wheel
				centeroid_TFLC_L, centeroid_TFLC_R = control_obsta.run_OAFLC_control(ultsnd_masure_Zaxs)
				interface.move_pltaform(centeroid_TFLC_R, centeroid_TFLC_L)
				time.sleep(0.5)
				print('\n')
			else:  # Obstacle NOT detected, TFLC control
				# TFLC control routine
				# Then actuate the wheel
				centeroid_TFLC_L, centeroid_TFLC_R = control_traye.run_TFLC_control(pos_err, hed_err)			
				interface.move_pltaform(centeroid_TFLC_R, centeroid_TFLC_L)							

			#plotter.update_robot_data(rob_poss, rob_ortt, pos_err, orr_err, hed_err)
			time.sleep(0.5)
		except KeyboardInterrupt:
			interface.close()
			print("\n¡Has presionado Ctrl-C! Saliendo del programa...")
			break	
	pass	

if __name__ == '__main__':
	main()	