from image_process import *

class purePursuit:
	def __init__(self, pose, points, lhd):
		self.pose   = pose
		self.points = points
		self.lhd  = lhd
		self.pt = np.array((0, 0))

		(self.img, self.factor, self.shift) = get_img_and_para(self.points)

	def process(self):
		theta, radius, d_img = self.calculate_angle()
		return theta, radius, d_img

	def calculate_angle(self):
		Phi = 7
		R = 1

		self.pt, d_img = get_intersection(self.img, self.factor, self.shift, self.pose[0:2], self.lhd)
		for pt in self.pt:
			if pt is None:
				return (None, None, d_img)
			else:
				x, y = pt[0], pt[1]

			X, Y = x - self.pose[0], y - self.pose[1]

			theta = self.pose[2] - np.pi/2

			x =  np.cos(theta)*X + np.sin(theta)*Y
			y = -np.sin(theta)*X + np.cos(theta)*Y

			if x == 0:
				d = np.inf
			else:
				d = (x**2 - y**2) / (2*x)

			r = -(d + x)

			if x < 0:
				phi = self.tan_inv(d, y)
			else:
				phi = -self.tan_inv(-d, y)

			if abs(phi) < abs(Phi):
				Phi = phi
				R = r
				pt = pt*self.factor + self.shift
				temp_d_img = cv2.circle(d_img.copy(), (int(pt[0]), int(pt[1])), 5, (0, 1, 0), -1)

		return Phi, R, temp_d_img

	def tan_inv(self, x, y):
		if x == 0:
			ang = np.pi/2
		else:
			ang = np.arctan(y/x)

		if x < 0:
			ang += np.pi
		elif y < 0:
			ang += 2*np.pi

		return ang