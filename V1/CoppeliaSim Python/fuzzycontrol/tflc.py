import numpy as np
from fuzzycontrol.aux import *

class TFLCControl(object):
	"""docstring for TFLCcontrol"""
	def __init__(self):
		super(TFLCControl, self).__init__()

		self.rules_TFLC_L = np.array([
			[1, 1, 1, 1, 3, 3, 4],
			[2, 2, 1, 2, 3, 5, 6],
			[2, 2, 2, 3, 5, 6, 7],
			[1, 1, 1, 4, 6, 6, 7],
			[2, 2, 3, 5, 5, 6, 7],
			[1, 1, 4, 6, 5, 6, 7],
			[2, 2, 3, 7, 5, 6, 7]
		])

		self.rules_TFLC_L_var = ['z', 's', 'nm', 'm', 'nh', 'h', 'vh']
		self.rules_TFLC_R = np.flip(self.rules_TFLC_L, axis=1)		
		
	def check_distance_membership(self, input):
		z = triangle(input, 0, 0, 0.50, 1, 1, 0)  # zero
		nz = triangle(input, 0, 0.50, 1.00, 0, 1, 0)  # near zero
		n = triangle(input, 0.50, 1.00, 1.50, 0, 1, 0)  # near
		m = triangle(input, 1.00, 1.50, 2.00, 0, 1, 0)  # medium
		nf = triangle(input, 1.50, 2.00, 2.50, 0, 1, 0)  # near far
		f = triangle(input, 2.00, 2.50, 3.00, 0, 1, 0)  # far
		vf = triangle(input, 2.50, 3.00, 3.00, 0, 1, 1)  # very far
		out = [z, nz, n, m, nf, f, vf]
		return out

	def check_err_angle_membership(self, input):
		n = triangle(input, -175, -175, -88, 1, 1, 0)  # negative
		sn = triangle(input, -175, -88, -43, 0, 1, 0)  # small neg
		nnz = triangle(input, -88, -43, 0, 0, 1, 0)  # near neg zero
		z = triangle(input, -43, 0, 43, 0, 1, 0)  # zero
		npz = triangle(input, 0, 43, 88, 0, 1, 0)  # near pos zero
		sp = triangle(input, 43, 88, 175, 0, 1, 0)  # small pos
		p = triangle(input, 88, 175, 175, 0, 1, 1)  # positive
		out = [n, sn, nnz, z, npz, sp, p]
		return out

	def check_velocity_membership(self, input):
		z = triangle(input, 0, 0, 2, 1, 1, 0)  # zero
		s = triangle(input, 0, 2, 4, 0, 1, 0)  # slow
		nm = triangle(input, 2, 4, 6, 0, 1, 0)  # near medium
		m = triangle(input, 4, 6, 8, 0, 1, 0)  # medium
		nh = triangle(input, 6, 8, 10, 0, 1, 0)  # near high
		h = triangle(input, 8, 10, 12, 0, 1, 0)  # high
		vh = triangle(input, 10, 12, 12, 0, 1, 1)  # very high
		out = [z, s, nm, m, nh, h, vh]
		return out

	def relate_TFLC(self, set1, set2, rules_index):
		# set1 is rows, and set2 is columns
		# find corresponding values for each rule
		m = np.zeros((len(set1) + 1, len(set2)))
		for i in range(len(set1)):
			for j in range(len(set2)):
				m[i, j] = min(set1[i], set2[j])  # MIN

		# find the output index of each rule value
		# then create a vector of possible maximum membership value from output
		# membership class
		v_out = np.zeros(np.max(rules_index))
		h, w = rules_index.shape
		for i in range(h):
			for j in range(w):
				#v_out[rules_index[i, j]] = max(v_out[rules_index[i, j]-1], m[i, j])  # MAX
				v_out[rules_index[i, j] - 1] = max(v_out[rules_index[i, j] - 1], m[i, j])  # MAX
		mat_rules_val = m
		v_out_th = v_out
		return mat_rules_val, v_out_th

	def calculate_output(self, pos_err, hed_err):
		# TFLC control
		fuz_dist = self.check_distance_membership(pos_err)
		fuz_orie = self.check_err_angle_membership(hed_err)

		mat_TFLC_L_rel, v_TFLC_L_th = self.relate_TFLC(fuz_dist, fuz_orie, self.rules_TFLC_L)
		mat_TFLC_R_rel, v_TFLC_R_th = self.relate_TFLC(fuz_dist, fuz_orie, self.rules_TFLC_R)

		centeroid_TFLC_L = self.calculate_centroid(v_TFLC_L_th)
		centeroid_TFLC_R = self.calculate_centroid(v_TFLC_R_th)

		centeroid_TFLC_L = centeroid_TFLC_L / 2
		centeroid_TFLC_R = centeroid_TFLC_R / 2

		return centeroid_TFLC_L, centeroid_TFLC_R

	def calculate_centroid(self, v_TFLC_th):
		dt = 0.01
		t = np.arange(-1, 13, dt)
		out = []
		for i in range(len(t)):
			ret = self.check_velocity_membership(t[i])
			ret = np.minimum(v_TFLC_th, ret)
			out.append(np.max(ret))
		centeroid = np.sum(np.array(out) * t) / np.sum(out)
		return centeroid

	def run_TFLC_control(self, pos_err, hed_err):
		centeroid_TFLC_L, centeroid_TFLC_R = self.calculate_output(pos_err, hed_err)
		# Somehow I need to flip R and L to make it work, if not it'd be positive feedback
		return centeroid_TFLC_L, centeroid_TFLC_R