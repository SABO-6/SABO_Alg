#!/usr/bin/python

import sys
sys.path.append('/home/hresvelgr/Downloads/legged/src/leggedcontrol_go2/legged_controllers/scripts')
sys.path.append('/home/hresvelgr/Downloads/legged/src/leggedcontrol_go2/legged_controllers/scripts/RPOL_env')

print("当前版本为：" + sys.version)

from kpkd_collection.RPOL_and_CKB import RPOL

import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy.optimize import minimize
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import Matern
from scipy.stats import norm
import math
import matplotlib.pyplot as plt
import pickle
import os

def kp_kd_total_diff_callback(msg):
    global kp, kd, new_data_received, obj, vio, threshold, Alg, ls, kp_upper, kd_upper, kp_lower, kd_lower

    if msg.data[0] == kp and msg.data[1] == kd and msg.data[2] == obj:
        vio = max(msg.data[3] - threshold, 0)
        new_data_received = False
        rospy.loginfo("Repeated data.")
    else:
        kp = msg.data[0]
        kd = msg.data[1]
        obj = msg.data[2]
        vio = msg.data[3] 
        rospy.loginfo("Optimizer received kp: %f, kd: %f, obj: %f, vio: %f", kp, kd, obj, vio)
        new_data_received = True
    # result = [[kp, kd, obj, vio]]

    # with open(f'/home/hresvelgr/Downloads/legged/src/leggedcontrol_go2/legged_controllers/scripts/Comparison/Flying_trot/Result_of_Flying_Trot_Upper_{kp_upper}_Lengthscale_{ls}_Threshold_{threshold}.txt', 'a') as file:
    #     for row in result:
    #         file.write("\t".join(map(str, row)) + "\n")


def main():
    global kp, kd, obj, vio, new_data_received, threshold, Alg, ls, kp_upper, kd_upper, kp_lower, kd_lower
    kp, kd, obj, threshold = 1, 1, 5, 0.3
    ls = 0.1
    kp_upper, kd_upper = 9, 9
    kp_lower, kd_lower = 0.5, 0.5
    Alg = "RPOL"
    global kp_stack, kd_stack
    kp_stack = []
    kd_stack = []
    
    rospy.init_node('kp_kd_optimizer', anonymous=False)
    rospy.Subscriber("/controllers/legged_controller/kp_kd_total_diff_topic", Float64MultiArray, kp_kd_total_diff_callback)
    kp_kd_pub = rospy.Publisher("/controllers/legged_controller/kp_kd_opt", Float64MultiArray, queue_size=10)
    rospy.loginfo("Optimizer initialized with kp: %f, kd: %f", kp, kd)
    pub_count = 0

    for index in range(2, 11):
    
        x = np.arange(kp_lower, kp_upper, ls)
        y = np.arange(kd_lower, kd_upper, ls)

        opt_obj_location = "/home/hresvelgr/Downloads/legged/src/leggedcontrol_go2/legged_controllers/scripts/kpkd_collection/flying_trot.kpkd"

        # if os.path.exists(opt_obj_location) and os.path.getsize(opt_obj_location):
        #     print("读取了！")
        #     bayes_opt = pickle.load(open(opt_obj_location, 'rb'))
        # else:
        #     print("没读取！")
        #     bayes_opt = RPOL(np.meshgrid(x, y))
        # bayes_opt = CKB(np.meshgrid(x, y))

        bayes_opt = RPOL(np.meshgrid(x, y))

        while not rospy.is_shutdown():

            next_point = bayes_opt.decision()
            print(next_point)
            next_kp, next_kd = next_point[0], next_point[1]
            # next_kp, next_kd = next_point[0, 0], next_point[0, 1]

            msg = Float64MultiArray()
            # msg.data = [next_kp, next_kd]
            msg.data = [5, 7.5]
            kp_kd_pub.publish(msg)
            new_data_received = False
            rospy.loginfo(f"{Alg} Optimizer published next kp: {next_kp} and next kd: {next_kd}")

            while not new_data_received and not rospy.is_shutdown():
                kp_kd_pub.publish(msg)
                rospy.sleep(5)
            
            X_next = np.array([kp, kd])
            Y_next = np.array([obj])

            bayes_opt.update(-obj, vio)
            pickle.dump(bayes_opt, open(opt_obj_location, 'wb'))
            result = [[kp, kd, obj, vio]]
            with open(f"/home/hresvelgr/Downloads/legged/src/leggedcontrol_go2/legged_controllers/scripts/Comparison/Flying_trot/Real_0ms_RPOL_Trot_Upper_{kp_upper}_Lengthscale_{ls}_Threshold_{threshold}_Index_{index}.txt", "a") as file:
                for row in result:
                    file.write("\t".join(map(str, row)) + "\n")
                pub_count += 1
                if pub_count == 600:
                    pub_count = 0
                    break




    

# Plot
    # obj_list = []
    # vio_list = []
    # Alg_list = ['RPOL']
    # upper_list = [3, 10]
    # ls_list = [0.1, 1]
    # threshold_list = [0.5, 0.3]
    # while not rospy.is_shutdown():
    #     for threshold in threshold_list:
    #         for upper in upper_list:
    #             for ls in ls_list:
    #                 x = np.arange(0, upper, ls)
    #                 y = np.arange(0, upper, ls)
    #                 for Alg in Alg_list:
    #                     if Alg == 'RPOL':
    #                         bayes_opt = RPOL(np.meshgrid(x, y))
    #                     elif Alg == 'CKB':
    #                         bayes_opt = CKB(np.meshgrid(x, y))
    #                     for _ in range(50):
    #                         next_point = bayes_opt.decision()
    #                         print(next_point)
    #                         next_kp, next_kd = next_point[0], next_point[1]
    #                         # next_kp, next_kd = next_point[0, 0], next_point[0, 1]

    #                         msg = Float64MultiArray()
    #                         msg.data = [next_kp, next_kd]
    #                         kp_kd_pub.publish(msg)
    #                         new_data_received = False
    #                         rospy.loginfo(f"{Alg} Optimizer published next kp: {next_kp} and next kd: {next_kd}")

    #                         while not new_data_received and not rospy.is_shutdown():
    #                             kp_kd_pub.publish(msg)
    #                             # rospy.loginfo("Keep publishing")
    #                             rospy.sleep(5)
                            
    #                         X_next = np.array([kp, kd])
    #                         Y_next = np.array([obj])
    #                         # print("X_next, Y_next: ", X_next, Y_next)

    #                         # bayes_opt.update(X_next, Y_next)
    #                         vio = max(vio - threshold, 0)
    #                         bayes_opt.update(-obj, vio)
    #                         obj_list.append(obj)
    #                         vio_list.append(vio)

    #                         result = [[kp, kd, obj, vio]]
    #                         with open(f"/home/hresvelgr/Downloads/legged/src/leggedcontrol_go2/legged_controllers/scripts/Comparison/Flying_trot/Result_of_Flying_Trot_Upper_{upper}_Lengthscale_{ls}_Threshold_{threshold}.txt", "a") as file:
    #                             for row in result:
    #                                 file.write("\t".join(map(str, row)) + "\n")



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
