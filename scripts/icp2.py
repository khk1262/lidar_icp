import numpy as np 
from scipy.spatial import KDTree

class Align2D:
	def __init__(self, source_points, target_points, initial_T=np.identity(3)):
		self.source = source_points
		self.target = target_points
		self.init_T = initial_T
		self.target_tree = KDTree(target_points[:,:2])
		self.transform = self.AlignICP(20, 1.0e-5)

	def AlignICP(self, max_iter, min_delta_err):

		mean_sq_error = 1.0e6 # initialize error as large number
		delta_err = 1.0e6    # change in error (used in stopping condition)
		T = self.init_T
		num_iter = 0         # number of iterations
		tf_source = self.source

		while delta_err > min_delta_err and num_iter < max_iter:

			# find correspondences via nearest-neighbor search
			matched_trg_pts,matched_src_pts,indices = self.FindCorrespondences(tf_source)

			# find alingment between source and corresponding target points via SVD
			# note: svd step doesn't use homogeneous points
			new_T = self.AlignSVD(matched_src_pts, matched_trg_pts)

			# update transformation between point sets
			T = np.dot(T,new_T)

			# apply transformation to the source points
			tf_source = np.dot(self.source,T.T)

			# find mean squared error between transformed source points and target points
			new_err = 0
			for i in range(len(indices)):
				if indices[i] != -1:
					diff = tf_source[i,:2] - self.target[indices[i],:2]
					new_err += np.dot(diff,diff.T)

			new_err /= float(len(matched_trg_pts))

			# update error and calculate delta error
			delta_err = abs(mean_sq_error - new_err)
			mean_sq_error = new_err

			num_iter += 1

		return T


	def FindCorrespondences(self,src_pts):

		# get distances to nearest neighbors and indices of nearest neighbors
		matched_src_pts = src_pts[:,:2]
		dist,indices = self.target_tree.query(matched_src_pts)

		# remove multiple associatons from index list
		# only retain closest associations
		unique = False
		while not unique:
			unique = True
			for i in range(len(indices)):
				if indices[i] == -1:
					continue
				for j in range(i+1,len(indices)):
					if indices[i] == indices[j]:
						if dist[i] < dist[j]:
							indices[j] = -1
						else:
							indices[i] = -1
							break
		# build array of nearest neighbor target points
		# and remove unmatched source points
		point_list = []
		src_idx = 0
		for idx in indices:
			if idx != -1:
				point_list.append(self.target[idx,:])
				src_idx += 1
			else:
				matched_src_pts = np.delete(matched_src_pts,src_idx,axis=0)

		matched_pts = np.array(point_list)

		return matched_pts[:,:2],matched_src_pts,indices


	def AlignSVD(self, source, target):

		# first find the centroids of both point clouds
		src_centroid = self.GetCentroid(source)
		trg_centroid = self.GetCentroid(target)

		# get the point clouds in reference to their centroids
		source_centered = source - src_centroid
		target_centered = target - trg_centroid

		# get cross covariance matrix M
		M = np.dot(target_centered.T,source_centered)

		# get singular value decomposition of the cross covariance matrix
		U,W,V_t = np.linalg.svd(M)

		# get rotation between the two point clouds
		R = np.dot(U,V_t)

		# get the translation (simply the difference between the point clound centroids)
		t = np.expand_dims(trg_centroid,0).T - np.dot(R,np.expand_dims(src_centroid,0).T)

		# assemble translation and rotation into a transformation matrix
		T = np.identity(3)
		T[:2,2] = np.squeeze(t)
		T[:2,:2] = R

		return T

	def GetCentroid(self, points):
		point_mean = np.mean(points,axis=0)
		return point_mean