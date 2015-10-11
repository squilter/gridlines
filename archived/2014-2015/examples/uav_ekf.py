#!/usr/bin/env python

# uav_ekf.py
# Implements EKF class to track UAV position

"""
 # State vector
 #    x =     [ pos_x,
 #              pos_y ]
 #
 # Control signal
 #    u =     [ vel_x,
 #              vel_y ]
 #
 # Measurement
 #    z =     [ pos_x_meas,
 #              pos_y_meas ]
 #
 # The Model
 #    x_pred_k+1 = g(x_k, u_k) = x_k + dt*u
"""

import rospy
import numpy as np
import params

from uav_msgs.msg import OpticalFlowPost
from uav_msgs.msg import GridLinePosPost
from uav_msgs.msg import GridIntersectList
from uav_msgs.msg import UavEstState


class UAV_EKF:

    # Propogated time
    prop_time = rospy.Time()

    # State vector
    x_shape = (2, 1)
    x = np.zeros(x_shape)

    # Covariance matrix
    P = np.zeros((2,2))

    # M_base = base control covariance TUNABLE
    _M_base = np.diag([1, 1])

    # R = process model covariance TUNABLE
    R = np.diag([1, 1])

    # last used control signal and M
    _last_u = np.zeros((2,1))
    _last_M = _M_base


    def __init__(self, init_x, time_start):

        # Check passed state vector dimensions
        if init_x.shape != self.x_shape:
            exc = Exception
            exc.message = 'Invalid dimensions for init_x argument'

        # Init state vector
        self.x = init_x

        # Init covariance
        self.P = np.diag([1,1])

        # Init reference time
        self.prop_time = time_start

        # Init publisher
        self.pub = rospy.Publisher('uav_est_state', UavEstState, queue_size=1)
        self._publish()

        return

    def __str__(self):
        return 'EKF: \n   Time = ' + str(self.prop_time) + '\n   Mean = ' + str(self.x) + '\n   Covariance = ' + str(self.P)

    def _publish(self):

        msg = UavEstState()
        msg.timestamp = self.prop_time

        # Copy state variables
        msg.state.append(self.x[0][0])
        msg.state.append(self.x[1][0])

        # Copy covariance
        for i in range(self.P.shape[0]):
            for j in range(self.P.shape[1]):
                msg.covariance.append(self.P[i][j])

        print 'Publishing UAV estimate.'
        self.pub.publish(msg)

        return

    def _process_model(self, dt, u):
        """
        :param dt: Time difference since last measurement [float in seconds]
        :param u: Control input, in our case the (x,y) velocity measurement from optical flow [Numpy array of dim(2,1)]
        :return: Evaluation of our process model g(x_k, u_k, dt) [Numpy array of dim(2,1)]
        """

        # initialize output vector
        g = np.zeros(self.x.shape)

        # pos_xal
        g[0] = self.x[0] + dt*u[0]
        # pos_y
        g[1] = self.x[1] + dt*u[1]

        return g

    def _propagate(self, dt, u=None, M=None):
        """
        :param dt: Duration to propagate forward (float in sec)
        :param u: Control vector (if present)
        :param M: Covariance matrix of the control signal
        :return: tuple (x_next, P_next)
        """

        # prepare control input and covariance matrix
        if u == None:
            u = self._last_u
        if M == None:
            M = self._last_M

        # verify parameters
        if not isinstance(M, np.ndarray):
            print 'EKF._propagate_covariance: invalid object passed in parameter M.'
            raise Exception
        expected_shape = (u.shape[0], u.shape[0])
        if M.shape != expected_shape:
            print 'EKF._propagate_covariance: invalid shape of matrix M with respect to u. Expected', expected_shape
            raise Exception

        # store last u and M
        self._last_u = u
        self._last_M = M

        # calculate new mean
        x_next = self._process_model(dt, u)

        # prepare G
        # G is the Jacobian of the process model with respect to the current state,
        # the partial derivative of process model at x_k and u_k with respect to x
        G = np.array([[1, 0], [0, 1]])

        # prepare V
        # V is the Jacobian of the process model with respect to the control signal
        V = np.array([[dt, 0], [0, dt]])

        P_next = G.dot(self.P).dot(G.transpose()) + ( V).dot(M.dot(V.transpose())) + self.R

        return (x_next, P_next)

    def prediction(self, of_post_meas):

        # verify type of of_post_meas
        if not isinstance(of_post_meas, OpticalFlowPost):
            print 'EKF.prediction: invalid object type passed in parameter of_post_meas'
            raise Exception

        # calculate dt
        dt = (of_post_meas.timestamp - self.prop_time).to_sec()
        if dt < 0:
            rospy.logerr('Attempting to propagate backwards in time. Failure.')
            raise Exception
        if dt > 0.1:
            rospy.logwarn('EKF: Longer than expected time since last prediction step. dt = %d.', dt)

        # prepare u vector
        u = np.array([[of_post_meas.x_vel],
                       [of_post_meas.y_vel]])

        # prepare measurement uncertainty M
        M = of_post_meas.uncertainty * self._M_base

        # predict next mean and covariance
        (x_next, P_next) = self._propagate(dt, u, M)

        # TODO: verify results

        # now store results
        self.x = x_next
        self.P = P_next
        self.prop_time = of_post_meas.timestamp

        # publish state
        self._publish()

        return

    def _measurement_model(self):

        # Computes h(x_k) -- the measurement we expect to see given our estimated state
        h = np.array([[self.x[0][0]],
                      [self.x[1]][0]])
        return h

    def _calc_updates(self, z, Q):
        """
        :param z: measurement vector
        :return: tuple (x_next, P_next)
        """

        # verify parameters
        if not isinstance(z, np.ndarray):
            print 'EKF._calc_updates: Invalid object type passed in parameter z.'
            raise Exception
        if not isinstance(Q, np.ndarray):
            print 'EKF._calc_updates: Invalid object type passed in parameter Q.'
            raise Exception

        # get predicted measurement
        z_pred = self._measurement_model()

        # prepare H
        H = np.array([[1, 0],
                      [0, 1]])

        # calculate K
        m = H.dot(self.P).dot(H.transpose()) + Q
        m_inv = np.linalg.inv(m)
        K = self.P.dot(H.transpose()).dot(m_inv)

        # compute next x
        innovation = z - z_pred
        x_next = self.x + K.dot(innovation)

        # compute next P
        P_next = (np.identity(2) - K.dot(H)).dot(self.P)

        return (x_next, P_next)

    def update(self, gl_pos_post_meas):

        # verify type of gridline message
        if not isinstance(gl_pos_post_meas, GridLinePosPost):
            print 'EKF.update: Invalid object type passed as gridline post-processed msg.'
            raise Exception

        # TODO: What do we do if gridline and optical flow measurements come out-of-order?

        # calculate dt
        dt = (gl_pos_post_meas.timestamp - self.prop_time).to_sec()
        if dt < 0:
            rospy.logwarn('EKF: Attempting to propagate backwards in time. Skipping.')
        if dt > 10:
            rospy.logwarn('EKF: Longer than expected time since last propagation. dt = %d.', dt)
        else:
            # propagate filter to timestamp of this gridline measurement
            # we don't have new control input so it will just use the last inputs
            self._propagate(gl_pos_post_meas.timestamp)
            self.prop_time = gl_pos_post_meas.timestamp

        # prepare Q matrix
        Q = 0.1*np.diag([1,1])

        # prepare z vector
        z = np.array([[gl_pos_post_meas.position.x],
                      [gl_pos_post_meas.position.y]])

        # compute updates
        (x_next, P_next) = self._calc_updates(z, Q)

        # TODO: verify results

        # store results
        self.x = x_next
        self.P = P_next

        # publish results
        self._publish()

        return

    # def update_gridpoints(self, gridpoint_list):
    #
    #     if not isinstance(gridpoint_list, GridIntersectList):
    #         print 'EKF.update_gridpoints: Invalid object type passed as GridIntersectList msg.'
    #         raise Exception
    #
    #
    #     # predict list of corners
    #     cell_width = params.field_width/params.grid_num_cols
    #     cell_height = params.field_height/params.grid_num_rows
    #
    #     # start from bottom right corner
    #     (x,y) = (-params.field_width/2, -params.field_height/2)
    #     while x <=


"""
Initialize EKF and subscribers
"""
print 'state_estimator'
rospy.init_node('state_estimator')

print 'Initializing EKF for UAV state estimation.'
init_x = np.array([[4],[4]])
ekf = UAV_EKF(init_x, rospy.get_rostime())

print 'Subscribing to of_meas_post and gridline_pos_post topics.'
sub_of = rospy.Subscriber('of_meas_post', OpticalFlowPost, ekf.prediction, queue_size=1)
sub_gl = rospy.Subscriber('gridline_pos_post', GridLinePosPost, ekf.update, queue_size=1)

# Loop until termination
count = 0
while not rospy.is_shutdown():
    ekf._publish()
    rospy.sleep(1.0)

