import tf
from sympy import *

## DH parameter table constants

DH = {
  'th1': 0,     'a0': 0,     'd1': 0.75,  'r0': 0,
  'th2': -pi/2, 'a1': -pi/2, 'd2': 0,     'r1': 0.35,
  'th3': 0,     'a2': 0,     'd3': 0,     'r2': 1.25,
  'th4': 0,     'a3': -pi/2, 'd4': 1.5,   'r3': -0.054,
  'th5': 0,     'a4': pi/2,  'd5': 0,     'r4': 0,
  'th6': 0,     'a5': -pi/2, 'd6': 0,     'r5': 0,
  'th7': 0,     'a6': 0,     'd7': 0.303, 'r6': 0,
}

th1, th2, th3, th4, th5, th6, th7 = symbols('th1:8') # joint variables
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # twist angles
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # link offests
r0, r1, r2, r3, r4, r5, r6 = symbols('r0:7') # link lenghts

s = {
  th1: th1 + DH['th1'], a0: DH['a0'], d1: DH['d1'], r0: DH['r0'],
  th2: th2 + DH['th2'], a1: DH['a1'], d2: DH['d2'], r1: DH['r1'],
  th3: th3 + DH['th3'], a2: DH['a2'], d3: DH['d3'], r2: DH['r2'],
  th4: th4 + DH['th4'], a3: DH['a3'], d4: DH['d4'], r3: DH['r3'],
  th5: th5 + DH['th5'], a4: DH['a4'], d5: DH['d5'], r4: DH['r4'],
  th6: th6 + DH['th6'], a5: DH['a5'], d6: DH['d6'], r5: DH['r5'],
  th7: th7 + DH['th7'], a6: DH['a6'], d7: DH['d7'], r6: DH['r6'],
}

# build a homogeneous transform
def htm(th, a, d, r):
    return Matrix([[cos(th), -sin(th), 0, r],
                   [sin(th)*cos(a), cos(th)*cos(a), -sin(a), -sin(a)*d],
                   [sin(th)*sin(a), cos(th)*sin(a), cos(a), cos(a)*d],
                   [0, 0, 0, 1]])

# build the homogeneous transformations for each join
T0_1 = htm(th1, a0, d1, r0).subs(s)
T1_2 = htm(th2, a1, d2, r1).subs(s)
T2_3 = htm(th3, a2, d3, r2).subs(s)
T3_4 = htm(th4, a3, d4, r3).subs(s)
T4_5 = htm(th5, a4, d5, r4).subs(s)
T5_6 = htm(th6, a5, d6, r5).subs(s)
T6_G = htm(th7, a6, d7, r6).subs(s)

# compose transformations between joints
T0_2 = simplify(T0_1 * T1_2)
T0_3 = simplify(T0_2 * T2_3)
T0_4 = simplify(T0_3 * T3_4)
T0_5 = simplify(T0_4 * T4_5)
T0_6 = simplify(T0_5 * T5_6)
T0_G = simplify(T0_6 * T6_G)


def calculate_IK(pose):
    px = pose.position.x
    py = pose.position.y
    pz = pose.position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w])

    # Calculate the end effector rotation matrix
    r, p, y = symbols('r p y')
    ROT_x = Matrix([[1, 0, 0],
                    [0, cos(r), -sin(r)],
                    [0, sin(r), cos(r)]])

    ROT_y = Matrix([[cos(p), 0, sin(p)],
                    [0, 1, 0],
                    [-sin(p), 0, cos(p)]])

    ROT_z = Matrix([[cos(y), -sin(y), 0],
                    [sin(y), cos(y), 0],
                    [0, 0, 1]])
    ROT_EE = ROT_z * ROT_y * ROT_x
    Rot_Error = ROT_z.subs(y, pi) * ROT_y.subs(p, -pi/2)
    ROT_EE = ROT_EE * Rot_Error
    ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

    EE = Matrix([[px], [py], [pz]])
    # calculate the wrist center
    WC = EE - (0.303) * ROT_EE[:, 2]

    # get an angle based on the three sides of a triangle
    #  using the law of cosines
    def loc(a, b, c):
        rad = acos((a**2 + b**2 - c**2) / (2 * a * b))
        return rad
    # the height of the wrist center
    wc_z = WC[2] - DH['d1']
    # the distance of the WC on the xy plane
    wc_xy = sqrt(WC[0]**2 + WC[1]**2) - DH['r1']
    # the distance of the WC in space
    wc_distance = sqrt(wc_xy ** 2 + (wc_z) ** 2)

    theta1 = atan2(WC[1],WC[0])

    alpha_a = atan2(wc_z, wc_xy)
    alpha_b = loc(DH['r2'], wc_distance, DH['d4'])
    theta2 = -((alpha_a + alpha_b) + DH['th2'])

    theta3 = -(loc(DH['d4'], DH['r2'], wc_distance) - pi/2)

    # extract the rotation matrice from 0-3
    R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
    # subtitute theta1-3
    R0_3 = R0_3.evalf(subs={th1: theta1, th2: theta2, th3: theta3})
    # get the rotation matric from 3-6
    R3_6 = R0_3.inv("LU") * ROT_EE

    # calculate theta4-6 using the methods described in lession11/8
    #  Euler angles from Rotation Matrix
    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])

    return theta1, theta2, theta3, theta4, theta5, theta6
