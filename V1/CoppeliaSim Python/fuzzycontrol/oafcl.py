import numpy as np
from fuzzycontrol.aux import *

class OAFLCControl(object):
	"""docstring for OAFLCControl"""
	def __init__(self):
		super(OAFLCControl, self).__init__()

		self.rules_OAFLC_R_L = np.array([[1, 1],
										 [2, 1],
										 [2, 1],
										 [1, 1],
										 [2, 1],
										 [2, 1],
										 [1, 1],
										 [2, 1],
										 [2, 1],
										 [1, 2],
										 [1, 1],
										 [5, 3],
										 [3, 5],
										 [5, 3],
										 [5, 3],
										 [1, 2],
										 [5, 3],
										 [5, 3],
										 [1, 2],
										 [3, 5],
										 [1, 1],
										 [1, 2],
										 [3, 5],
										 [5, 3],
										 [1, 2],
										 [3, 5],
										 [4, 4]])

		self.var_OAFLC_output = ['nh', 'n', 'p', 'hp', 'vhp']

	def check_obstacle_membership(self, input_val):
		n = trapezoid(input_val, 0, 10, 30, 40, 0, 1, 1, 0)  # near
		m = trapezoid(input_val, 30, 40, 60, 70, 0, 1, 1, 0)  # medium
		f = trapezoid(input_val, 60, 70, 90, 100, 0, 1, 1, 1)  # far
		out = np.array([n, m, f])
		return out

	def check_vl_vr_oaflc_membership(self, input_val):
		nh = triangle(input_val, -10, -5, 0, 0, 1, 0)  # negative high
		n =  triangle(input_val, -5, -2.5, 0, 0, 1, 0)  # negative
		p =  triangle(input_val, 0, 2.5, 5, 0, 1, 0)  # positive
		hp = triangle(input_val, 0, 5, 10, 0, 1, 0)  # high positive
		vhp =triangle(input_val, 5, 7.5, 10, 0, 1, 0)  # very high positive
		out = np.array([nh, n, p, hp, vhp])
		return out

	def relate_OAFLC(self, set1, set2, set3, rules_index):
		m = np.zeros((len(rules_index), 2))
		c = 0
		for i in range(len(set1)):
			for j in range(len(set2)):
				for k in range(len(set3)):
					c = c + 1
					MIN = min(min(set1[i], set2[j]), set3[k])
					m[c-1, 0] = MIN  # MIN output1
					m[c-1, 1] = MIN  # MIN output2

		v_out1 = np.zeros(max(rules_index[:, 0]))
		v_out2 = np.zeros(max(rules_index[:, 1]))
		for i in range(len(rules_index)):
			v_out1[rules_index[i, 0]-1] = max(v_out1[rules_index[i, 0]-1], m[i, 0])  # MAX
			v_out2[rules_index[i, 1]-1] = max(v_out2[rules_index[i, 1]-1], m[i, 1])  # MAX

		return m, v_out1, v_out2

	def calculate_centroid(self, t, v_th):
		out = []
		for i in range(len(t)):
			ret = self.check_vl_vr_oaflc_membership(t[i])  # Output fuzzification
			ret = np.minimum(v_th, ret)
			out.append(np.max(ret))  # MAX
		centroid = np.sum(np.array(out) * t) / np.sum(out)
		return centroid

	def run_OAFLC_control(self, ultsnd_masure_Zaxs):
		# Fuzzyfication
		fuz_s1 = self.check_obstacle_membership(ultsnd_masure_Zaxs[0])  # Left sensor
		fuz_s2 = self.check_obstacle_membership(ultsnd_masure_Zaxs[1])  # Front sensor
		fuz_s3 = self.check_obstacle_membership(ultsnd_masure_Zaxs[2])  # Right sensor

		# Inference
		mat_rules_val, v_OAFLC_L_th, v_OAFLC_R_th = self.relate_OAFLC(fuz_s1, fuz_s2, fuz_s3, self.rules_OAFLC_R_L)

		# Calculating centroid OAFLC R
		dt = 0.01
		t = np.arange(-10, 15, dt)  # Output data range
		centroid_OAFLC_R = self.calculate_centroid(t, v_OAFLC_R_th)

		# Calculating centroid OAFLC L
		centroid_OAFLC_L = self.calculate_centroid(t, v_OAFLC_L_th)

		return centroid_OAFLC_R, centroid_OAFLC_L