import numpy as np

def triangle(input, start, middle, stop, str_val, mid_val, sto_val):
	if input < start:
		out = str_val
	elif input >= start and input < middle:
		v = (mid_val - str_val) / (middle - start)
		out = str_val + v * (input - start)
	elif input == middle:
		out = mid_val
	elif input > middle and input <= stop:
		v = (sto_val - mid_val) / (stop - middle)
		out = mid_val + v * (input - middle)
	elif input > stop:
		out = sto_val
	return out

def trapezoid(input, start, middle1, middle2, stop, str_val, mid1_val, mid2_val, sto_val):
	if input < start:
		out = str_val
	elif input >= start and input < middle1:
		v = (mid1_val - str_val) / (middle1 - start)
		out = str_val + v * (input - start)
	elif input == middle1:
		out = mid1_val
	elif input > middle1 and input < middle2:
		v = (mid2_val - mid1_val) / (middle2 - middle1)
		out = mid1_val + v * (input - middle1)
	elif input == middle2:
		out = mid2_val
	elif input > middle2 and input <= stop:
		v = (sto_val - mid2_val) / (stop - middle2)
		out = mid2_val + v * (input - middle2)
	elif input > stop:
		out = sto_val
	return out

class TFLCcontrol(object):
	"""docstring for TFLCcontrol"""
	def __init__(self):
		super(TFLCcontrol, self).__init__()

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
		m = np.zeros((len(set1), len(set2)))
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
				v_out[rules_index[i, j]] = max(v_out[rules_index[i, j]], m[i, j])  # MAX

		mat_rules_val = m
		v_out_th = v_out
		return mat_rules_val, v_out_th

	def calculate_output(self, pos_err, hed_err):

		# TFLC control
		# Fuzzification
		fuz_dist = self.check_distance_membership(pos_err)
		fuz_orie = self.check_err_angle_membership(hed_err)

		# Inference
		mat_TFLC_L_rel, v_TFLC_L_th = self.relate_TFLC(fuz_dist, fuz_orie, self.rules_TFLC_L)
		mat_TFLC_R_rel, v_TFLC_R_th = self.relate_TFLC(fuz_dist, fuz_orie, self.rules_TFLC_R)

		# Calculating centroid TFLC L
		dt = 0.01
		t = np.arange(-1, 13, dt)  # Output data range
		out = []
		for i in range(len(t)):
			ret = self.check_velocity_membership(t[i])  # Output fuzzification
			ret = np.minimum(v_TFLC_L_th, ret)
			# Max value / real value of the inference
			out.append(np.max(ret))  # MAX
		centeroid_TFLC_L = np.sum(np.array(out) * t) / np.sum(out)

		# Calculating centroid TFLC R
		out = []
		for i in range(len(t)):
			ret = self.check_velocity_membership(t[i])  # Output fuzzification
			ret = np.minimum(v_TFLC_R_th, ret)
			# Max value / real value of the inference
			out.append(np.max(ret))  # MAX
		centeroid_TFLC_R = np.sum(np.array(out) * t) / np.sum(out)

		# Actuate the wheel
		centeroid_TFLC_L = centeroid_TFLC_L / 2
		centeroid_TFLC_R = centeroid_TFLC_R / 2

		# Somehow I need to flip R and L to make it work, if not it'd be positive feedback
		return centeroid_TFLC_L, centeroid_TFLC_R		
		pass


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
        mat_rules_val, v_OAFLC_L_th, v_OAFLC_R_th = self.relate_OAFLC(fuz_s1, fuz_s2, fuz_s3)

        # Calculating centroid OAFLC R
        dt = 0.01
        t = np.arange(-10, 15, dt)  # Output data range
        centroid_OAFLC_R = self.calculate_centroid(t, v_OAFLC_R_th)

        # Calculating centroid OAFLC L
        centroid_OAFLC_L = self.calculate_centroid(t, v_OAFLC_L_th)

        return centroid_OAFLC_R, centroid_OAFLC_L



		
		