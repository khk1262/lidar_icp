import sys
import numpy as np
import math

'''
P : moved data
Q : origin data
'''

def get_correspondence_indices(P, Q):
    """For each point in P find closest one in Q."""
    p_size = P.shape[0]
    q_size = Q.shape[0]
    correspondences = []
    distances = []
    for i in range(p_size):
        p_point = P[i, :]
        min_dist = sys.maxsize
        chosen_idx = -1
        for j in range(q_size):
            q_point = Q[j, :]
            dist = (q_point[0] - p_point[0]) * (q_point[0] - p_point[0]) + (q_point[1] - p_point[1]) * (q_point[1] - p_point[1])
            # dist = np.linalg.norm(q_point - p_point)
            if dist < min_dist:
                min_dist = dist
                chosen_idx = j
        distances.append(min_dist)
        correspondences.append((i, chosen_idx))

    return distances, correspondences


def center_data(data):
    center = np.mean(data, axis=0)
    return center, data - center


def compute_cross_covariance(P, Q, correspondences):
    cov = np.zeros((2, 2))
    for i, j in correspondences:
        p_point = P[[i], :]
        q_point = Q[[j], :]
        cov += q_point.T.dot(p_point)
    return cov


def compute_R_t(data, center_P, center_Q):
    m = center_P.shape[0]

    U, S, V_T = np.linalg.svd(data)
    R_found = np.dot(U, V_T)
    t_found = center_Q.T - R_found.dot(center_P.T)

    # T = np.identity(m+1)
    # T[:m, :m] = R_found
    # T[:m, m] = t_found
    return R_found, t_found

def icp_svd(P, Q, iterations=20, min_error=1.0e-4):
    
    center_of_Q, Q_centered = center_data(Q)
    P_copy = P.copy() # deep copy
    prev_error = 0

    for i in range(iterations):
        center_of_P, P_centered = center_data(P_copy)

        dist, correspondences = get_correspondence_indices(P_centered, Q_centered)

        cov = compute_cross_covariance(P_centered, Q_centered, correspondences)
        R, t = compute_R_t(cov, center_of_P, center_of_Q)

        P_copy = R.dot(P_copy.T).T + t

        mean_error = np.linalg.norm(center_of_P - center_of_Q)
        if mean_error < min_error:
            break


    return R, t