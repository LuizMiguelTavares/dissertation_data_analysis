#!/usr/bin/python3

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

LQG_data = np.array(pd.read_csv("LQG/LQG_data.csv"))
LQR_data = np.array(pd.read_csv("LQR/LQR_data.csv"))
PID_data_fail = np.array(pd.read_csv("PID/1-PID_fail/PID_data.csv"))
PID_data_sucess = np.array(pd.read_csv("PID/2-PID/PID_data.csv"))
PID_no_noise_data = np.array(pd.read_csv("PID_no_noise/PID_data_no_noise.csv"))

# LQG
# [A.pPos.Xd A.pPos.X thrust thrust_feito A.pPos.dX A.pPos.Z([2 5 8]) cmd cmd_raw A.pPos.X_sujo A.pPos.T dummy_var_pose.pPos.X t_atual]
#     1-12    13-24     25        26       27 -- 38     39 40 41     42-45  46-49     50 51 52    53-58        59 -- 70          71

# PID
# [A.pPos.Xd  A.pPos.X  thrust  thrust_feito  A.pPos.dX  A.pPos.Z([2 5 8])    cmd   cmd_raw   A.pPos.X_sujo  A.pPos.Xi  A.pPos.dXi  dummy_var_pose.pPos.X  t_atual];
#     1-12      13-24     25         26       27 -- 38       39 40 41        42-45   46-49       50 51 52     53 54 55    56 57 58         59-70             71

time_LQG = LQG_data[:, -1]
time_LQR = LQR_data[:, -1]
time_PID_fail = PID_data_fail[:, -1]
time_PID_sucess = PID_data_sucess[:, -1]
time_PID_no_noise = PID_no_noise_data[:, -1]

pose_LQG = LQG_data[:, 58:64]
pose_LQR = LQR_data[:, 58:64]
pose_PID_fail = PID_data_fail[:, 58:64]
pose_PID_sucess = PID_data_sucess[:, 58:64]
pose_PID_no_noise = PID_no_noise_data[:, 58:64]
