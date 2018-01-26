from sympy import *
from time import time
from mpmath import radians
import tf
import numpy as np
from sympy.matrices import Matrix
from sympy import atan2, sqrt

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()

    ########################################################################################
    ##

    ## Insert IK code here!

    theta1 = test_case[2][0]
    theta2 = test_case[2][1]
    theta3 = test_case[2][2]
    theta4 = test_case[2][3]
    theta5 = test_case[2][4]
    theta6 = test_case[2][5]

    ##
    ########################################################################################

    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    th1, th2, th3, th4, th5, th6, th7 = symbols('th1:8') # joint variables
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # twist angles
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # link offests
    r0, r1, r2, r3, r4, r5, r6 = symbols('r0:7') # link lenghts

    s = {
      th1: theta1,        a0: 0,     d1: 0.75,  r0: 0,
      th2: theta2 - pi/2, a1: -pi/2, d2: 0,     r1: 0.35,
      th3: theta3,        a2: 0,     d3: 0,     r2: 1.25,
      th4: theta4,        a3: -pi/2, d4: 1.5,   r3: -0.054,
      th5: theta5,        a4: pi/2,  d5: 0,     r4: 0,
      th6: theta6,        a5: -pi/2, d6: 0,     r5: 0,
      th7: 0,             a6: 0,     d7: 0.303, r6: 0,
    }

    def htm(th, a, d, r):
        return Matrix([[cos(th), -sin(th), 0, r],
                       [sin(th)*cos(a), cos(th)*cos(a), -sin(a), -sin(a)*d],
                       [sin(th)*sin(a), cos(th)*sin(a), cos(a), cos(a)*d],
                       [0, 0, 0, 1]])

    T0_1 = htm(th1, a0, d1, r0).subs(s)
    T1_2 = htm(th2, a1, d2, r1).subs(s)
    T2_3 = htm(th3, a2, d3, r2).subs(s)
    T3_4 = htm(th4, a3, d4, r3).subs(s)
    T4_5 = htm(th5, a4, d5, r4).subs(s)
    T5_6 = htm(th6, a5, d6, r5).subs(s)
    T6_G = htm(th7, a6, d7, r6).subs(s)

    T0_2 = simplify(T0_1 * T1_2)
    T0_3 = simplify(T0_2 * T2_3)
    T0_4 = simplify(T0_3 * T3_4)
    T0_5 = simplify(T0_4 * T4_5)
    T0_6 = simplify(T0_5 * T5_6)
    T0_G = simplify(T0_6 * T6_G)

    R_z = Matrix([[cos(pi), -sin(pi), 0, 0],
                  [sin(pi),  cos(pi), 0, 0],
                  [0,  0, 1, 0],
                  [0,  0, 0, 1]])
    R_y = Matrix([[cos(-pi/2), 0, sin(-pi/2), 0],
                  [0,  1, 0, 0],
                  [-sin(-pi/2),  0, cos(-pi/2), 0],
                  [0,  0, 0, 1]])
    R_corr = simplify(R_z, R_y)

    T_total = simplify(T0_G * R_corr)

    subs = {
        th1: 0,
        th2: 0,
        th3: 0,
        th4: 0,
        th5: 0,
        th6: 0,
    }
    print('T0_1', T0_1.evalf(subs=subs))
    print('T0_2', T0_2.evalf(subs=subs))
    print('T0_3', T0_3.evalf(subs=subs))
    print('T0_4', T0_4.evalf(subs=subs))
    print('T0_5', T0_5.evalf(subs=subs))
    print('T0_6', T0_6.evalf(subs=subs))
    print('T0_G', T0_G.evalf(subs=subs))
    print('total', T_total.evalf(subs=subs))

    EE = T_total.evalf(subs=subs)
    alpha = atan2(EE[1,0], EE[0,0])
    beta = atan2(EE[2,0], sqrt(EE[1,0]+EE[0,0]))
    gamma = atan2(EE[2,1], EE[2,2])
    print('alpha', alpha)
    print('beta', beta)
    print('gamma', gamma)

    offset = 0.303
    print('offset', offset)
    wc_x = EE[0,3] - offset * EE[0,2]
    wc_y = EE[1,3] - offset * EE[1,2]
    wc_z = EE[2,3] - offset * EE[2,2]
    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [wc_x, wc_y, wc_z] # <--- Load your calculated WC values in this array
    your_ee = [EE[0,3],EE[1,3],EE[2,3]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f (%04.8f - %04.8f)" % (wc_x_e, your_wc[0], test_case[1][0]))
        print ("Wrist error for y position is: %04.8f (%04.8f - %04.8f)" % (wc_y_e, your_wc[1], test_case[1][1]))
        print ("Wrist error for z position is: %04.8f (%04.8f - %04.8f)" % (wc_z_e, your_wc[2], test_case[1][2]))
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f (%04.8f - %04.8f)" % (ee_x_e, your_ee[0], test_case[0][0][0]))
        print ("End effector error for y position is: %04.8f (%04.8f - %04.8f)" % (ee_y_e, your_ee[1], test_case[0][0][1]))
        print ("End effector error for z position is: %04.8f (%04.8f - %04.8f)" % (ee_z_e, your_ee[2], test_case[0][0][2]))
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 2

    test_code(test_cases[test_case_number])
