import numpy as np
from numpy import sin, cos
import scipy.linalg    # you may find scipy.linalg.block_diag useful
import sys

class EKF(object):

    def __init__(self, x0, P0, Q):
        self.x = x0    # Gaussian belief mean
        self.P = P0    # Gaussian belief covariance
        self.Q = Q     # Gaussian control noise covariance (corresponding to dt = 1 second)

    # Updates belief state given a discrete control step (Gaussianity preserved by linearizing dynamics)
    # INPUT:  (u, dt)
    #       u - zero-order hold control input
    #      dt - length of discrete time step
    # OUTPUT: none (internal belief state (self.x, self.P) should be updated)
    def transition_update(self, u, dt):
        g, Gx, Gu = self.transition_model(u, dt)
        self.x = g 
        if u:
            self.P = np.matmul(np.matmul(Gx, self.P), Gx.transpose()) + dt * np.matmul(np.matmul(Gu,self.Q) , Gu.transpose())
        else:
            # if there's no control dependency u is none
            self.P = np.matmul(np.matmul(Gx, self.P), Gx.transpose())


    # Propagates exact (nonlinear) state dynamics; also returns associated Jacobians for EKF linearization
    # INPUT:  (u, dt)
    #       u - zero-order hold control input
    #      dt - length of discrete time step
    # OUTPUT: (g, Gx, Gu)
    #      g  - result of belief mean self.x propagated according to the system dynamics with control u for dt seconds
    #      Gx - Jacobian of g with respect to the belief mean self.x
    #      Gu - Jacobian of g with respect to the control u
    def transition_model(self, u, dt):
        # g(x, u) => x_(t+1)
        # x = x_old + u_t*dt 
        #TODO
        return (g, Gx, Gu)


    # Updates belief state according to a given measurement (with associated uncertainty)
    # INPUT:  (rawZ, rawR)
    #    rawZ - raw measurement mean
    #    rawR - raw measurement uncertainty
    # OUTPUT: none (internal belief state (self.x, self.P) should be updated)
    def measurement_update(self, rawZ, rawR):
        z, R, H = self.measurement_model(rawZ, rawR)
        if z is None:    # don't update if measurement is invalid (e.g., no line matches for line-based EKF localization)
            return

        #### TODO ####
        # update self.x, self.P
        ##############
        sig = np.matmul(np.matmul(H, self.P), H.transpose()) + R 
        K = np.matmul(np.matmul(self.P, H.transpose()), np.linalg.inv(sig))
        self.x = self.x + np.matmul(K, z)
        self.P = self.P - np.matmul(np.matmul(K, sig), K.transpose())
    # Converts raw measurement into the relevant Gaussian form (e.g., a dimensionality reduction);
    # also returns associated Jacobian for EKF linearization
    # INPUT:  (rawZ, rawR)
    #    rawZ - raw measurement mean
    #    rawR - raw measurement uncertainty
    # OUTPUT: (z, R, H)
    #       z - measurement mean (for simple measurement models this may = rawZ)
    #       R - measurement covariance (for simple measurement models this may = rawR)
    #       H - Jacobian of z with respect to the belief mean self.x
    def measurement_model(self, rawZ, rawR):
        raise NotImplementedError("measurement_model must be overriden by a subclass of EKF")


# for single detected object
class MeasurementObjectEKF(EKF):
    def __init__(self, x0, P0, Q):
        self.x_cam = None
        self.y_cam = None

    def transition_model(self):
        g = self.x
        x_dim = self.x.shape
        Gx = np.zeros((x_dim, x_dim))
        Gu = None
        return g, Gx, Gu

    def measurement_model(self, rawZ, rawR):
        cam2object = self.x-cam_loc #(xcam -xobj, ycam-yobj)
        alpha_predicted = np.linalg.norm(cam2object)
        r_predicted = np.arctan2(cam2object[0], cam2object[1])
        h = np.array([alpha_predicted, r_predicted])
        Hx = np.array([
            [-0.5*np.sqrt(r_predicted) * (cam2object[0]), -0.5*np.sqrt(r_predicted) * (cam2object[1])],
            [-cam2object[1]/(r_predicted**2), -cam2object[1]/(r_predicted**2)]])

    # This must be called before calling measurement update
    def update_cam_loc(self, x_cam, y_cam):
        self.x_cam = None
        self.y_cam = None


# class Localization_EKF(EKF):

#     def __init__(self, x0, P0, Q, map_lines, tf_base_to_camera, g):
#         self.map_lines = map_lines                    # 2xJ matrix containing (alpha, r) for each of J map lines
#         self.tf_base_to_camera = tf_base_to_camera    # (x, y, theta) transform from the robot base to the camera frame
#         self.g = g                                    # validation gate
#         super(self.__class__, self).__init__(x0, P0, Q)

#     # Unicycle dynamics (Turtlebot 2)
#     def transition_model(self, u, dt):
#         v, w = u
#         x, y, th = self.x
#         if abs(w) > 1e-3:
#             th_new = th + w * dt 
#             x_new = x + v/w*(np.sin(th_new)-sin(th))
#             y_new = y - v/w*(np.cos(th_new)-cos(th))
#             g = np.array([x_new, y_new, th_new])
#             Gx = np.array([[1, 0, -v*np.sin(th)*dt],[0,1,v*np.cos(th)*dt],[0,0,1]])
#             Gu = np.array([
#                 [(np.sin(th_new)-sin(th))/w, 
#                     -v/(w**2)*(sin(th_new)-sin(th))+v/w*(cos(th_new)*dt)],
#                 [-(np.cos(th_new)-cos(th))/w, 
#                     v/(w**2)*(cos(th_new)-cos(th))-v/w*(-sin(th_new)*dt)], 
#                 [0, dt]])
#         else:
#             th_new = th + w * dt 
#             x_new = x + v * np.cos(th_new)*dt
#             y_new = y + v * np.sin(th_new)*dt
#             g = np.array([x_new, y_new, th_new])
#             Gx = np.array([[1, 0, -v*np.sin(th)*dt],[0,1,v*np.cos(th)*dt],[0,0,1]])
#             Gu = np.array(
#                 [[np.cos(th)*dt, -v/2 *(sin(th_new)*dt**2)],
#                 [np.sin(th)*dt, v/2*cos(th_new)*dt**2], 
#                 [0, dt]])
#         #### TODO ####
#         # compute g, Gx, Gu
#         ##############

#         return g, Gx, Gu

#     # Given a single map line m in the world frame, outputs the line parameters in the scanner frame so it can
#     # be associated with the lines extracted from the scanner measurements
#     # INPUT:  m = (alpha, r)
#     #       m - line parameters in the world frame
#     # OUTPUT: (h, Hx)
#     #       h - line parameters in the scanner (camera) frame
#     #      Hx - Jacobian of h with respect to the the belief mean self.x
#     def map_line_to_predicted_measurement(self, m):
#         alpha, r = m
#         x_bot, y_bot, th_bot = self.x
#         x_cam, y_cam, th_cam = self.tf_base_to_camera
#         alpha_new = alpha - th_bot - th_cam
#         r_new = r - (x_bot*cos(alpha) + y_bot*sin(alpha)) - (x_cam*cos(alpha-th_bot) + y_cam*sin(alpha-th_bot))

#         h = (alpha_new, r_new)
#         Hx = np.array([[0,0, -1],
#                     [-cos(alpha), -sin(alpha), -(x_cam*sin(alpha-th_bot) - y_cam*cos(alpha-th_bot))]])
#         #### TODO ####
#         # compute h, Hx
#         ##############

#         flipped, h = normalize_line_parameters(h)
#         if flipped:
#             Hx[1,:] = -Hx[1,:]

#         return h, Hx

#     # Given lines extracted from the scanner data, tries to associate to each one the closest map entry
#     # measured by Mahalanobis distance
#     # INPUT:  (rawZ, rawR)
#     #    rawZ - 2xI matrix containing (alpha, r) for each of I lines extracted from the scanner data (in scanner frame)
#     #    rawR - list of I 2x2 covariance matrices corresponding to each (alpha, r) column of rawZ
#     # OUTPUT: (v_list, R_list, H_list)
#     #  v_list - list of at most I innovation vectors (predicted map measurement - scanner measurement)
#     #  R_list - list of len(v_list) covariance matrices of the innovation vectors (from scanner uncertainty)
#     #  H_list - list of len(v_list) Jacobians of the innovation vectors with respect to the belief mean self.x
#     def associate_measurements(self, rawZ, rawR):

#         #### TODO ####
#         # compute v_list, R_list, H_list
#         v_list = [] 
#         R_list = [] 
#         H_list = []
#         _, n_line = self.map_lines.shape
#         _, n_measure = rawZ.shape
#         map_cam_list = [self.map_line_to_predicted_measurement(self.map_lines[:,m]) for m in range(n_line)]
#         for j in range(n_measure):
#             z = rawZ[:,j]
#             R = rawR[j]
#             d_min = sys.maxint
#             idx_min = -1
#             for i in range(n_line):
#                 h, Hx = map_cam_list[i]
#                 v = z - h
#                 S = np.matmul(np.matmul(Hx, self.P), Hx.transpose()) + R
#                 d = np.matmul(np.matmul(v.transpose(), np.linalg.inv(S)), v)
#                 if d < d_min:
#                     d_min = d
#                     v_min = v
#                     idx_min = i
#             if d_min <= self.g**2:
#                 v_list.append(v_min)
#                 R_list.append(R)
#                 H_list.append(map_cam_list[idx_min][1])
#         ##############

#         return v_list, R_list, H_list

#     # Assemble one joint measurement, covariance, and Jacobian from the individual values corresponding to each
#     # matched line feature
#     def measurement_model(self, rawZ, rawR):
#         v_list, R_list, H_list = self.associate_measurements(rawZ, rawR)
#         if not v_list:
#             print "Scanner sees", rawZ.shape[1], "line(s) but can't associate them with any map entries"
#             return None, None, None

#         #### TODO ####
#         # compute z, R, H
#         ##############
#         v_len = len(v_list)
#         r_dim, _ = R_list[0].shape
#         R = np.zeros((v_len *r_dim, v_len*r_dim))
#         for i in range(v_len):
#             R[i*r_dim:(i+1)*r_dim, i*r_dim:(i+1)*r_dim] = R_list[i]
#         z = np.hstack(v_list).transpose()
#         H = np.vstack(H_list)
#         return z, R, H


# class SLAM_EKF(EKF):

#     def __init__(self, x0, P0, Q, tf_base_to_camera, g):
#         self.tf_base_to_camera = tf_base_to_camera    # (x, y, theta) transform from the robot base to the camera frame
#         self.g = g                                    # validation gate
#         super(self.__class__, self).__init__(x0, P0, Q)

#     # Combined Turtlebot + map dynamics
#     # Adapt this method from Localization_EKF.transition_model.
#     def transition_model(self, u, dt):
#         v, w = u
#         x, y, th = self.x[:3]
#         m = self.x[3:]
#         x_dim = self.x.shape[0]

#         if abs(w) > 1e-3:
#             th_new = th + w * dt 
#             x_new = x + v/w*(np.sin(th_new)-sin(th))
#             y_new = y - v/w*(np.cos(th_new)-cos(th))
#             # landmark variable remains the same
#             g = np.hstack([np.array([x_new, y_new, th_new]), m])

#             Gx = np.diagflat(np.hstack([[0,0,0],np.ones(x_dim-3)]))
#             Gx[0:3,0:3] = np.array([[1, 0, -v*np.sin(th)*dt],[0,1,v*np.cos(th)*dt],[0,0,1]])

#             Gu = np.zeros((x_dim, 2))
#             Gu[:3, :2] = np.array([
#                 [(np.sin(th_new)-sin(th))/w, 
#                     -v/(w**2)*(sin(th_new)-sin(th))+v/w*(cos(th_new)*dt)],
#                 [-(np.cos(th_new)-cos(th))/w, 
#                     v/(w**2)*(cos(th_new)-cos(th))-v/w*(-sin(th_new)*dt)], 
#                 [0, dt]])
#         else:
#             th_new = th + w * dt 
#             x_new = x + v * np.cos(th_new)*dt
#             y_new = y + v * np.sin(th_new)*dt
#             g = np.hstack([np.array([x_new, y_new, th_new]), self.x[3:]])

#             Gx = np.diagflat(np.hstack([[0,0,0],np.ones(x_dim-3)]))
#             Gx[:3, :3] = np.array([[1, 0, -v*np.sin(th)*dt],[0,1,v*np.cos(th)*dt],[0,0,1]])

#             Gu = np.zeros((x_dim, 2))
#             Gu[:3, :2] = np.array(
#                 [[np.cos(th)*dt, -v/2 *(sin(th_new)*dt**2)],
#                 [np.sin(th)*dt, v/2*cos(th_new)*dt**2], 
#                 [0, dt]])



#         #### TODO ####
#         # compute g, Gx, Gu (some shape hints below)
#         # g = np.copy(self.x)
#         # Gx = np.eye(self.x.size)
#         # Gu = np.zeros((self.x.size, 2))
#         ##############

#         return g, Gx, Gu

#     # Combined Turtlebot + map measurement model
#     # Adapt this method from Localization_EKF.measurement_model.
#     #
#     # The ingredients for this model should look very similar to those for Localization_EKF.
#     # In particular, essentially the only thing that needs to change is the computation
#     # of Hx in map_line_to_predicted_measurement and how that method is called in
#     # associate_measurements (i.e., instead of getting world-frame line parameters from
#     # self.map_lines, you must extract them from the state self.x)
#     def measurement_model(self, rawZ, rawR):
#         v_list, R_list, H_list = self.associate_measurements(rawZ, rawR)
#         if not v_list:
#             print "Scanner sees", rawZ.shape[1], "line(s) but can't associate them with any map entries"
#             return None, None, None

#         #### TODO ####
#         # compute z, R, H (should be identical to Localization_EKF.measurement_model above)
#         ##############
#         v_len = len(v_list)
#         r_dim, _ = R_list[0].shape
#         R = np.zeros((v_len *r_dim, v_len*r_dim))
#         for i in range(v_len):
#             R[i*r_dim:(i+1)*r_dim, i*r_dim:(i+1)*r_dim] = R_list[i]
#         z = np.hstack(v_list).transpose()
#         H = np.vstack(H_list)
#         return z, R, H

#     # Adapt this method from Localization_EKF.map_line_to_predicted_measurement.
#     #
#     # Note that instead of the actual parameters m = (alpha, r) we pass in the map line index j
#     # so that we know which components of the Jacobian to fill in.
#     def map_line_to_predicted_measurement(self, j):
#         alpha, r = self.x[(3+2*j):(3+2*j+2)]    # j is zero-indexed! (yeah yeah I know this doesn't match the pset writeup)

#         #### TODO ####
#         # compute h, Hx (you may find the skeleton for computing Hx below useful)

#         alpha, r = self.x[3+j*2 : 3+j*2+2]
#         x_bot, y_bot, th_bot = self.x[:3]
#         x_cam, y_cam, th_cam = self.tf_base_to_camera
#         alpha_new = alpha - th_bot - th_cam
#         r_new = r - (x_bot*cos(alpha) + y_bot*sin(alpha)) - (x_cam*cos(alpha-th_bot) + y_cam*sin(alpha-th_bot))
#         h = (alpha_new, r_new)

#         Hx = np.zeros((2,self.x.size))
#         Hx[:,:3] = np.array([[0,0, -1],
#                     [-cos(alpha), -sin(alpha), -(x_cam*sin(alpha-th_bot) - y_cam*cos(alpha-th_bot))]])
#         # First two map lines are assumed fixed so we don't want to propagate any measurement correction to them
#         if j > 1:
#             Hx[0, 3+2*j] = 1 # d alpha/d alphaa 
#             Hx[1, 3+2*j] = -(-x_bot*sin(alpha) + y_bot*cos(alpha)) - (x_cam*cos(alpha - th_bot) + y_cam*sin(alpha-th_bot))
#                              # d r / d alpha
#             Hx[0, 3+2*j+1] = 0 # d alpha/ d r 
#             Hx[1, 3+2*j+1] = 1 # d r /d r

#         ##############

#         flipped, h = normalize_line_parameters(h)
#         if flipped:
#             Hx[1,:] = -Hx[1,:]

#         return h, Hx

#     # Adapt this method from Localization_EKF.associate_measurements.
#     def associate_measurements(self, rawZ, rawR):

#         #### TODO ####
#         # compute v_list, R_list, H_list
#         ##############
#         v_list = [] 
#         R_list = [] 
#         H_list = []
#         n_line = (len(self.x) - 3)/2
#         _, n_measure = rawZ.shape
#         map_cam_list = [self.map_line_to_predicted_measurement(j) for j in range(n_line)]

#         for j in range(n_measure):
#             z = rawZ[:,j]
#             R = rawR[j]
#             d_min = sys.maxint
#             idx_min = -1
#             for i in range(n_line):
#                 h, Hx = map_cam_list[i]
#                 v = z - h
#                 S = np.matmul(np.matmul(Hx, self.P), Hx.transpose()) + R
#                 d = np.matmul(np.matmul(v.transpose(), np.linalg.inv(S)), v)
#                 if d < d_min:
#                     d_min = d
#                     v_min = v
#                     idx_min = i
#             if d_min <= self.g**2:
#                 v_list.append(v_min)
#                 R_list.append(R)
#                 H_list.append(map_cam_list[idx_min][1])
#         return v_list, R_list, H_list
