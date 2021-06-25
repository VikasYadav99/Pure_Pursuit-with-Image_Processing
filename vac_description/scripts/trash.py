
# def get_img_and_para(arr):
# 	act_size = np.array((1200, 1200))
# 	img_size = np.array((np.max(arr[:, 1]) - np.min(arr[:, 1]),
# 	                     np.max(arr[:, 0]) - np.min(arr[:, 0]), 3))
	
# 	factors = (act_size / img_size[0:2])[::-1].reshape((-1, 2))
# 	factors[0, 1] = -factors[0, 1] 
# 	shift = np.array(((0, act_size[0]/2)))
	
# 	# x = factors[1] * arr[:, 0] + 100
# 	# y = factors[0] * (-arr[:, 1]) + act_size[0]
# 	# points = np.array((x, y), dtype = np.int16).T

# 	points = np.array(factors * arr, dtype=np.int16) + shift

# 	if(points.any() < 0):
# 		print("Something wrong")

# 	img = np.zeros((1000, 1200, 3))
# 	f = 256.0**3 / points.shape[0]

# 	for i in range(points.shape[0]-1):
# 		p = int(i * f)
# 		cv2.line(img, tuple(points[i]), 
# 			tuple(points[i+1]), 
# 			[(p//(256**2))/256.0, ((p/256)%256)/256.0, (p%256)/256.0], 
# 			2)

# 	cv2.imshow("img", img)
# 	return (img, factors, shift)

# x = np.arange(0, 10, 0.1)
	# y = np.sin(x)
	# y[:] = 1
	# y[0:15] = 0
	# y[30:45] = 0

	# for i in range(0, 100, 4):
	# 	y[i:i+2] = 0.1