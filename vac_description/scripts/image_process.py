import numpy as np
import cv2

def get_img_and_para(arr):
	height = 600.0
	pad = 50

	img_size = {'h': np.max(arr[:, 1]) - np.min(arr[:, 1]),
	            'w': np.max(arr[:, 0]) - np.min(arr[:, 0])}

	f = height / img_size['h']
	f = (f, f)
	
	s = (0+pad/2, pad/2 - f[0]*np.min(arr[:, 1]))

	points = np.array(f*arr + s, dtype=np.int16)

	img = np.zeros((int(pad + height), int(pad + (img_size['w']*height)/img_size['h']), 3))
	col_fact = 256.0**3 / points.shape[0]

	for i in range(points.shape[0]-1):
		p = int((i+1) * col_fact)
		cv2.line(img,
				tuple(points[i]), 
				tuple(points[i+1]), 
				[(p//(256**2))/256.0, ((p/256)%256)/256.0, (p%256)/256.0], 
				4)
		
	return (img, f, s)


def get_intersection(img, f, s, pt, r):
	pt = np.array(pt)
	pt = np.array(pt*f + s, dtype=np.int16)
	
	r = int(r*f[0])
	cir_img = np.zeros_like(img)
	cv2.circle(cir_img, tuple(pt), r, (1, 1, 1), 4)
	b_img = cv2.circle(np.zeros_like(img), tuple(pt), 10, (1, 0, 0), -1)

	new_origin = np.array((max(pt[0]-2*r, 0), max(pt[1]-2*r, 0)))

	cr_cir_img = cir_img[
					max(pt[1]-2*r, 0):min(pt[1]+2*r, img.shape[0]), 
					max(pt[0]-2*r, 0):min(pt[0]+2*r, img.shape[1])]
	cr_img     = img[
					max(pt[1]-2*r, 0):min(pt[1]+2*r, img.shape[0]), 
					max(pt[0]-2*r, 0):min(pt[0]+2*r, img.shape[1])]

	d_img = (b_img+img)

	intersection_img = ((cv2.bitwise_and(cr_img, cr_cir_img) > 0)*255).astype(np.uint8)
	intersection_img = cv2.cvtColor(intersection_img, cv2.COLOR_BGR2GRAY)

	im, cont, h = cv2.findContours(intersection_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	h_pix = -1
	hv_pt = [(0, 0)]
	for c in cont:
		m = cv2.moments(c)
		x, y = int(m["m10"]/(m["m00"] + 10**(-5))), int(m["m01"]/(m["m00"]+10**(-5)))
		pix = cr_img[y, x]
		pix = pix[0]*(256**2) + pix[1]*256 + pix[2]

		if pix > h_pix:
			hv_pt = [(x, y)]
			h_pix = pix
		elif pix == h_pix:
			hv_pt.append((x, y))

	hv_pt = [(pt[0]+new_origin[0], new_origin[1]+pt[1]) for pt in hv_pt]

	# for pt in hv_pt:
	# 	d_img = cv2.circle(d_img, (int(pt[0]), int(pt[1])), 5, (0, 1, 0), -1)

	# d_img = d_img[::-1]

	# cv2.imshow("d_img", d_img)
	hv_pt = [(np.array(pt) - s)/f  for pt in hv_pt]
	
	if h_pix == -1:
		return [None], d_img
	else:
		return hv_pt, d_img



if __name__ == "__main__":
	# x = np.arange(0, 10, 0.01)
	# y = np.sin(x)
	# y = x*0 + 1
	# y[-1] = 0

	x = (0, 0.50, 0.68, 1.99, 2.57, 7.17)
	y = (0, 0.77, 0.85, 2.55, 3.54, 4.26)

	arr = np.array((x, y)).T
	img, f, s = get_img_and_para(arr)
	cv2.imshow("img", img[::-1])
	cv2.waitKey(0)
	get_intersection(img, f, s, (0, 0), 0.5)

	cv2.waitKey(0)
	cv2.destroyAllWindows()