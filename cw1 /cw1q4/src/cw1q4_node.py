#!/usr/bin/env python3

from urllib import response
import rospy
import numpy as np

# TODO: Include all the required service classes
# your code starts here -----------------------------
from cw1q4_srv.srv import quat2rodrigues
from cw1q4_srv.srv import quat2zyx


from cw1q4_srv.srv import quat2rodriguesResponse
from cw1q4_srv.srv import quat2zyxResponse
from cw1q4_srv.srv import quat2rodriguesRequest
from cw1q4_srv.srv import quat2zyxRequest
# your code ends here ------------------------------

def convert_quat2zyx(request):
    # TODO complete the function
    """Callback ROS service function to convert quaternion to Euler z-y-x representation

    Args:
        request (quat2zyxRequest): cw1q4_srv service message, containing
        the quaternion you need to convert.
    
    Returns:
        quat2zyxResponse: cw1q4_srv service response, in which 
        you store the requested euler angles 
    """
    assert isinstance(request, quat2zyxRequest)

    # Your code starts here ----------------------------

    q_w = request.q.x
    q_x = request.q.x
    q_y = request.q.y
    q_z = request.q.z


    q_xx = q_x*q_x
    q_yy = q_y*q_y
    q_zz = q_z*q_z

    response = quat2zyxResponse() #same variable as srv file

    response.x.data = np.arctan2(2*(q_w*q_x + q_y*q_z) , 1- 2*(q_xx + q_yy))
    response.y.data = np.arcsin(2*(q_w*q_y - q_z*q_x))
    response.z.data = np.arctan2(2*(q_w*q_z + q_x*q_y) , 1- 2*(q_yy+ q_zz))


        # Your code ends here ------------------------------
    
    assert isinstance(response, quat2zyxResponse)
    return response


def convert_quat2rodrigues(request):
    # TODO complete the function

    """Callback ROS service function to convert quaternion to rodrigues representation
    
    Args:
        request (quat2rodriguesRequest): cw1q4_srv service message, containing
        the quaternion you need to convert
   

    Returns:
        quat2rodriguesResponse: cw1q4_srv service response, in which 
        you store the requested rodrigues representation 
    """
    assert isinstance(request, quat2rodriguesRequest)

    # Your code starts here ----------------------------

    response = quat2rodriguesResponse()
    qx = request.q.x
    qy = request.q.y
    qz = request.q.z
    qw = request.q.w

    norm = qx * qx + qy * qy + qz * qz
    
    if norm > 0: 
        sin_theta = np.sqrt(norm)
        factor = 2*np.arctan2(sin_theta , qw)/sin_theta
        x_v = qx *  factor 
        y_v = qy *  factor 
        z_v = qz *  factor
    else: 
        factor = 2
        x_v = qx * factor 
        y_v = qy * factor 
        z_v = qz*  factor
        
    response.x.data = x_v

    response.y.data = y_v

    response.z.data = z_v
    # Your code ends here ------------------------------

    assert isinstance(response, quat2rodriguesResponse)
    return response

def rotation_converter():
    rospy.init_node('rotation_converter')

    #Initialise the services
    rospy.Service('quat2rodrigues', quat2rodrigues, convert_quat2rodrigues)
    rospy.Service('quat2zyx', quat2zyx, convert_quat2zyx)

    rospy.spin()


if __name__ == "__main__":
    rotation_converter()
