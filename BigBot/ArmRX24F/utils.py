from numpy import *

# Length of links in cm
l1= 8.85
l2 = 5.2
l3 = 9.9

def returnAngleFromXYAndAngle(px, py, phi):
    '''px = -15
    py = 5.895+3.1
    phi = 180'''

    phi = deg2rad(phi)

    # Equations for Inverse kinematics
    x2 = px - l3*cos(phi)
    y2 = py - l3*sin(phi)

    delta = x2**2 + y2**2
    c2 = ( delta -l1**2 -l2**2)/(2*l1*l2)
    s2 = sqrt(1-c2**2)  # elbow down
    theta_2 = arctan2(s2, c2)

    s1 = ((l1+l2*c2)*y2 - l2*s2*x2)/delta
    c1 = ((l1+l2*c2)*x2 + l2*s2*y2)/delta

    theta_1 = arctan2(s1,c1)
    theta_3 = phi-theta_1-theta_2

    dtheta_1 = round(rad2deg(theta_1), 2)
    dtheta_2 = round(rad2deg(theta_2), 2)
    dtheta_3 = round(rad2deg(theta_3), 2)

    return [dtheta_1, dtheta_2, dtheta_3]

    '''print('Real theta_1:', dtheta_1)
    print('Real theta_2:', dtheta_2)
    print('Real theta_3:', dtheta_3)
    print('Total theta :', dtheta_1 + dtheta_2 + dtheta_3)
    print("")
    print('Serv theta_1:', 240- dtheta_1)
    print('Serv theta_2:', 150+ dtheta_2)
    print('Serv theta_3:', 150- dtheta_3)
    print("")
    print('Fusi theta_1:', dtheta_1-90)
    print('Fusi theta_2:', dtheta_2)
    print('Fusi theta_3:', dtheta_3)'''