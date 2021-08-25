## This file creates the contact plan for different gaits in an MPC fashion
## Author : Avadesh Meduri
## Date : 6/05/2021

import time
import numpy as np
from numpy.lib.arraysetops import isin
import pinocchio as pin
from inverse_kinematics_cpp import InverseKinematics
from py_biconvex_mpc.motion_planner.cpp_biconvex import BiConvexMP
from gait_planner_cpp import GaitPlanner

from matplotlib import pyplot as plt

class SoloMpcGaitGen:

    def __init__(self, robot, r_urdf, dt, weight_abstract, x_reg, planning_time, q0, height_map = None):
        """
        Input:
            robot : robot model
            r_urdf : urdf of robot
            dt : discretization
            weight_abstract : weight abstract class with params for desired motion
            x_reg : joint config about which regulation is done
            plan_freq : planning frequency in seconds
        """

        ## Note : only creates plan for horizon of 2*st
        self.rmodel = robot.model
        self.rdata = robot.data
        self.r_urdf = r_urdf
        self.dt = dt
        self.foot_size = 0.018
        self.params = weight_abstract
        self.step_height = self.params.step_ht

        #TODO: DEPRECATE THIS...
        #Use for a fixed frequency planning time
        self.planning_time = planning_time

        self.eff_names = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
        self.hip_names = ["FL_HFE", "FR_HFE", "HL_HFE", "HR_HFE"]
        pin.forwardKinematics(self.rmodel, self.rdata, q0, np.zeros(self.rmodel.nv))
        pin.updateFramePlacements(self.rmodel, self.rdata)
        com_init = pin.centerOfMass(self.rmodel, self.rdata, q0, np.zeros(self.rmodel.nv))

        self.offsets = np.zeros((len(self.eff_names), 2))
        self.ee_frame_id = []
        for i in range(len(self.eff_names)):
            self.ee_frame_id.append(self.rmodel.getFrameId(self.eff_names[i]))
            self.offsets[i] = self.rdata.oMf[self.rmodel.getFrameId(self.hip_names[i])].translation[0:2] - com_init[0:2].copy()
            self.offsets[i] = np.round(self.offsets[i], 3)

        # Contact-planning offsets
        self.offsets[0][0] -= 0.00 #Front Left_X
        self.offsets[0][1] += 0.04 #Front Left_Y
        
        self.offsets[1][0] -= 0.00 #Front Right_X
        self.offsets[1][1] -= 0.04 #Front Right_Y
        
        self.offsets[2][0] += 0.00 #Hind Left_X
        self.offsets[2][1] += 0.04 #Hind Left_Y
        
        self.offsets[3][0] += 0.00 #Hind Right X
        self.offsets[3][1] -= 0.04 #Hind Right Y
        self.apply_offset = True

        #Current Contact
        self.current_contact = np.zeros(4)

        self.swing_wt = self.params.swing_wt # swing foot cost
        self.cent_wt = self.params.cent_wt # centeroidal cost
        
        self.state_wt = self.params.state_wt
        self.ctrl_wt = self.params.ctrl_wt

        self.x_reg = x_reg
        self.reg_wt = self.params.reg_wt

        # --- Set up gait parameters ---
        self.gait_period = self.params.gait_period
        self.stance_percent = self.params.stance_percent
        self.gait_dt = self.params.gait_dt
        self.phase_offset = self.params.phase_offset
        self.gait_planner = GaitPlanner(self.gait_period, np.array(self.stance_percent), \
                                        np.array(self.phase_offset), self.step_height)

        #Different horizon parameterizations; only self.gait_horizon works for now
        self.gait_horizon = self.params.gait_horizon
        self.horizon = int(round(self.gait_horizon*self.gait_period/self.gait_dt))
        
        # --- Set up Inverse Kinematics ---
        self.ik_horizon = 0.5*self.gait_horizon*self.gait_period
        self.ik = InverseKinematics(r_urdf, self.gait_dt, self.ik_horizon)

        # --- Set up Dynamics ---
        self.m = pin.computeTotalMass(self.rmodel)
        self.rho = self.params.rho # penalty on dynamic constraint violation
        self.mp = BiConvexMP(self.m, self.gait_dt, self.gait_horizon*self.gait_period, len(self.eff_names), rho = self.rho)

        # Set up Weights & Matrices for Dynamics
        self.W_X = self.params.W_X
        self.W_X_ter = self.params.W_X_ter
        self.W_F = self.params.W_F
        self.X_nom = np.zeros((9*self.horizon))
        self.nom_ht = self.params.nom_ht

        # Set up constraints for Dynamics
        self.bx = 0.45
        self.by = 0.45
        self.bz = 1.0
        self.fx_max = 16.5
        self.fy_max = 16.5
        self.fz_max = 75

        # --- Set up other variables ---
        # For interpolation (should be moved to the controller)
        self.xs_int = np.zeros((len(self.x_reg), int(self.dt/0.001)))
        self.us_int = np.zeros((self.rmodel.nv, int(self.dt/0.001)))
        self.f_int = np.zeros((4*len(self.eff_names), int(self.dt/0.001)))

        # For plotting
        self.com_traj = []
        self.xs_traj = []
        self.q_traj = []
        self.v_traj = []

        #Height Map (for contacts)
        self.height_map = height_map

    def update_params(self, swing_wt = None, cent_wt = None, nom_ht = None, W_X = None, W_X_ter = None, X_nom = None):
        """
        updates parameters
        """
        if not swing_wt == None:
            self.swing_wt = swing_wt
        if not cent_wt == None:
            self.cent_wt = cent_wt
        if not nom_ht == None:
            self.nom_ht == None
        if isinstance(W_X, np.ndarray):
            self.W_X = W_X
        if isinstance(W_X_ter, np.ndarray):
            self.W_X_ter = W_X_ter
        if isinstance(X_nom, np.ndarray):
            self.X_nom = X_nom
    
    def create_cnt_plan(self, q, v, t, v_des):
        pin.forwardKinematics(self.rmodel, self.rdata, q, v)
        pin.updateFramePlacements(self.rmodel, self.rdata)

        com = np.round(pin.centerOfMass(self.rmodel, self.rdata, q, v)[0:2], 3)
        vcom = np.round(v[0:2], 3)

        vtrack = v_des[0:2] # this effects the step location (if set to vcom it becomes raibert)
        # vtrack[1] = vcom[1]
        # vtrack = vcom[0:2]

        self.cnt_plan = np.zeros((self.horizon, len(self.eff_names), 4))
        # This array determines when the swing foot cost should be enforced in the ik
        self.swing_time = np.zeros((self.horizon, len(self.eff_names)))
        self.prev_cnt = np.zeros((len(self.eff_names), 3))
        self.curr_cnt = np.zeros(len(self.eff_names))
        # Contact Plan Matrix: horizon x num_eef x 4: The '4' gives the contact plan and location:
        # i.e. the last vector should be [1/0, x, y, z] where 1/0 gives a boolean for contact (1 = contact, 0 = no cnt)

        for i in range(self.horizon):
            for j in range(len(self.eff_names)):
                ft = np.round(t + i*self.gait_dt,3)
                if self.gait_planner.get_phase(ft, j) == 1:
                    self.cnt_plan[i][j][0] = 1
                    self.cnt_plan[i][j][1:4] = np.round(self.rdata.oMf[self.ee_frame_id[j]].translation, 3)
                    self.cnt_plan[i][j][3]  = self.foot_size
                    self.prev_cnt[j] = self.cnt_plan[i][j][1:4]
                else:
                    self.cnt_plan[i][j][0] = 0
                    self.cnt_plan[i][j][1:4] = np.round(self.rdata.oMf[self.ee_frame_id[j]].translation, 3)
                    #     self.cnt_plan[i][j][1:3] += self.offsets[j]

        return self.cnt_plan

    def create_costs(self, q, v, v_des, t):
        """
        Input:
            q : joint positions at current time
            v : joint velocity at current time
            v_des : desired velocity of center of mass
            t : time within the step
        """

        self.x0 = np.hstack((q,v))

        # --- Set Up IK --- #
        #Right now this is only setup to go for the *next* gait period only
        for i in range(int(round(self.ik_horizon/self.gait_dt))):
            for j in range(len(self.eff_names)):
                if self.cnt_plan[i][j][0] == 1:
                    self.ik.add_position_tracking_task_single(self.ee_frame_id[j], self.cnt_plan[i][j][1:4], self.swing_wt[0],
                                                              "cnt_" + str(0) + self.eff_names[j], i)
                # elif self.swing_time[i][j] == 1:
                #     pos = self.cnt_plan[i][j][1:4].copy()
                #     pos[2] = self.step_height
                #     self.ik.add_position_tracking_task_single(self.ee_frame_id[j], pos, self.swing_wt[1],
                #                                               "via_" + str(0) + self.eff_names[j], i)

        self.ik.add_state_regularization_cost(0, self.ik_horizon, self.reg_wt[0], "xReg", self.state_wt, self.x_reg, False)
        self.ik.add_ctrl_regularization_cost_2(0, self.ik_horizon, self.reg_wt[1], "uReg", self.ctrl_wt, np.zeros(self.rmodel.nv), False)

        self.ik.add_state_regularization_cost(0, self.ik_horizon, self.reg_wt[0], "xReg", self.state_wt, self.x_reg, True)
        self.ik.add_ctrl_regularization_cost_2(0, self.ik_horizon, self.reg_wt[1], "uReg", self.ctrl_wt, np.zeros(self.rmodel.nv), True)

        self.ik.setup_costs()

        # --- Setup Dynamics --- #
        # initial and terminal state
        self.X_init = np.zeros(9)
        pin.computeCentroidalMomentum(self.rmodel, self.rdata)
        self.X_init[0:3] = pin.centerOfMass(self.rmodel, self.rdata, q.copy(), v.copy())
        self.X_init[3:] = np.array(self.rdata.hg)
        self.X_init[3:6] /= self.m

        # if (np.any(abs(self.X_init[6:]) > 0.15)) :
        #     np.savez("./dat_file/debug.npz", q = q, v = v, t = t)
        #     assert False

        self.X_nom[0::9] = self.X_init[0]
        for i in range(1, self.horizon):
            self.X_nom[9*i+0] = self.X_nom[9*(i-1)+0] + v_des[0]*self.dt
            self.X_nom[9*i+1] = self.X_nom[9*(i-1)+1] + v_des[1]*self.dt

        self.X_nom[2::9] = self.nom_ht
        self.X_nom[3::9] = v_des[0]
        self.X_nom[4::9] = v_des[1]
        self.X_nom[5::9] = v_des[2]

        amom = self.compute_ori_correction(q, np.array([0,0,0,1])) #Changed to 0 for bounding
        self.X_nom[6::9] = amom[0]*self.params.ori_correction[0] 
        self.X_nom[7::9] = amom[1]*self.params.ori_correction[1]
        self.X_nom[8::9] = amom[2]*self.params.ori_correction[2]

        X_ter = np.zeros_like(self.X_init)
        X_ter[0:2] = self.X_init[0:2] + (self.gait_horizon*self.gait_period*v_des)[0:2] #Changed this
        X_ter[2] = self.nom_ht
        X_ter[3:6] = v_des
        X_ter[6:] = amom

        # Setup dynamic optimization
        self.mp.create_contact_array_2(np.array(self.cnt_plan))
        self.mp.create_bound_constraints_2(self.bx, self.by, self.bz, self.fx_max, self.fy_max, self.fz_max)
        self.mp.create_cost_X(self.W_X, self.W_X_ter, X_ter, self.X_nom)
        self.mp.create_cost_F(self.W_F)

        #Shift costs & constraints (Assumes shift of one knot point for now...)
        # TODO: Make update_dynamics take in the time
        # self.mp.update_dynamics()

    def compute_ori_correction(self, q, des_quat):
        """
        This function computes the AMOM required to correct for orientation
        q : current joint configuration
        des_quat : desired orientation
        """
        pin_quat = pin.Quaternion(np.array(q[3:7]))
        pin_des_quat = pin.Quaternion(np.array(des_quat))

        omega = pin.log3((pin_des_quat*(pin_quat.inverse())).toRotationMatrix())

        return omega

    def optimize(self, q, v, t, v_des, step_height, current_contact, X_wm = None, F_wm = None, P_wm = None):

        #TODO: Move to C++
        t1 = time.time()
        self.step_height = step_height
        self.current_contact = current_contact
        self.create_cnt_plan(q, v, t, v_des)

        #Creates costs for IK and Dynamics
        self.create_costs(q, v, v_des, t)
        self.q_traj.append(q)
        self.v_traj.append(v)


        # --- Dynamics optimization ---

        t2 = time.time()
        com_opt, F_opt, mom_opt = self.mp.optimize(self.X_init, 85, X_wm, F_wm, P_wm)
        t3 = time.time()
        com_tmp = pin.centerOfMass(self.rmodel, self.rdata, self.q_traj[-1], self.v_traj[-1])

        self.com_traj.append(com_opt)

        # --- IK Optimization ---
        # Add tracking costs from Dynamic optimization
        self.ik.add_centroidal_momentum_tracking_task(0, self.ik_horizon, mom_opt[0:int(round(self.ik_horizon/self.gait_dt))], self.cent_wt[1], "mom_track", False)
        self.ik.add_centroidal_momentum_tracking_task(0, self.ik_horizon, mom_opt[int(round(self.ik_horizon/self.gait_dt))], self.cent_wt[1], "mom_track", True) #Final state

        self.ik.add_com_position_tracking_task(0, self.ik_horizon, com_opt[0:int(round(self.ik_horizon/self.gait_dt))], self.cent_wt[0], "com_track_cost", False)
        self.ik.add_com_position_tracking_task(0, self.ik_horizon, com_opt[int(round(self.ik_horizon/self.gait_dt))], self.cent_wt[0], "com_track_cost", True) #Final State

        t4 = time.time()
        self.ik.optimize(np.hstack((q,v)))
        t5 = time.time()

        xs = self.ik.get_xs()
        us = self.ik.get_us()

        self.xs_traj.append(xs)
        # opt_mom, opt_com = self.compute_optimal_com_and_mom(xs, us)
        # self.mp.add_ik_com_cost(opt_com)
        # self.mp.add_ik_momentum_cost(opt_mom)
        print("cost", t2 - t1)
        print("dyn", t3 - t2)
        print("ik", t5 - t4)
        print("total", t5 - t1)
        print("------------------------")

        n_eff = 3*len(self.eff_names)
        ind = int(self.planning_time/self.dt) + 1 # 1 is to account for time lag
        for i in range(ind):
            if i == 0:
                self.f_int = np.linspace(F_opt[i*n_eff:n_eff*(i+1)], F_opt[n_eff*(i+1):n_eff*(i+2)], int(self.dt/0.001))
                self.xs_int = np.linspace(xs[i], xs[i+1], int(self.dt/0.001))
                self.us_int = np.linspace(us[i], us[i+1], int(self.dt/0.001))

                self.com_int = np.linspace(com_opt[i], com_opt[i+1], int(self.dt/0.001))
                self.mom_int = np.linspace(mom_opt[i], mom_opt[i+1], int(self.dt/0.001))
            else:
                self.f_int =  np.vstack((self.f_int, np.linspace(F_opt[i*n_eff:n_eff*(i+1)], F_opt[n_eff*(i+1):n_eff*(i+2)], int(self.dt/0.001))))
                self.xs_int = np.vstack((self.xs_int, np.linspace(xs[i], xs[i+1], int(self.dt/0.001))))
                self.us_int = np.vstack((self.us_int, np.linspace(us[i], us[i+1], int(self.dt/0.001))))

                self.com_int = np.vstack((self.com_int, np.linspace(com_opt[i], com_opt[i+1], int(self.dt/0.001))))
                self.mom_int = np.vstack((self.mom_int, np.linspace(mom_opt[i], mom_opt[i+1], int(self.dt/0.001))))

        return self.xs_int, self.us_int, self.f_int

    def compute_optimal_com_and_mom(self, xs, us):
        """
        This function computes the optimal momentum based on the solution
        """

        opt_mom = np.zeros((len(xs), 6))
        opt_com = np.zeros((len(xs), 3))
        m = pin.computeTotalMass(self.rmodel)
        for i in range(len(xs)):
            q = xs[i][:self.rmodel.nq]
            v = xs[i][self.rmodel.nq:]
            pin.forwardKinematics(self.rmodel, self.rdata, q, v)
            pin.computeCentroidalMomentum(self.rmodel, self.rdata)
            opt_com[i] = pin.centerOfMass(self.rmodel, self.rdata, q, v)
            opt_mom[i] = np.array(self.rdata.hg)
            opt_mom[i][0:3] /= m

        return opt_com, opt_mom


    def reset(self):
        self.ik = InverseKinematics(self.r_urdf, self.gait_dt, self.ik_horizon)
        self.mp = BiConvexMP(self.m, self.gait_dt, self.gait_horizon*self.gait_period, len(self.eff_names), rho = self.rho)

    def plot(self, com_real=None):
        """
        This function plots the iterative mpc plans for the COM and Forces
        """
        self.com_traj = np.array(self.com_traj)
        self.q_traj = np.array(self.q_traj)
        self.v_traj = np.array(self.v_traj)
        x = self.dt*np.arange(0, len(self.com_traj[1]) + int((self.planning_time/self.dt))*len(self.com_traj), 1)
        # com plots
        fig, ax = plt.subplots(3,1)
        for i in range(0, len(self.com_traj)):
            st_hor = i*int(self.planning_time/self.dt)

            if i == 0:
                com = pin.centerOfMass(self.rmodel, self.rdata, self.q_traj[i], self.v_traj[i])
                ax[0].plot(x[st_hor], com[0], "o", label = "real com x")
                ax[1].plot(x[st_hor], com[1], "o", label = "real com y")
                ax[2].plot(x[st_hor], com[2], "o", label = "real com z")


                ax[0].plot(x[st_hor:st_hor + len(self.com_traj[i])], self.com_traj[i][:,0], label = "com x")
                ax[1].plot(x[st_hor:st_hor + len(self.com_traj[i])], self.com_traj[i][:,1], label = "com y")
                ax[2].plot(x[st_hor:st_hor + len(self.com_traj[i])], self.com_traj[i][:,2], label = "com z")

            else:
                com = pin.centerOfMass(self.rmodel, self.rdata, self.q_traj[i], self.v_traj[i])
                ax[0].plot(x[st_hor], com[0], "o")
                ax[1].plot(x[st_hor], com[1], "o")
                ax[2].plot(x[st_hor], com[2], "o")
        
                ax[0].plot(x[st_hor:st_hor + len(self.com_traj[i])], self.com_traj[i][:,0])
                ax[1].plot(x[st_hor:st_hor + len(self.com_traj[i])], self.com_traj[i][:,1])
                ax[2].plot(x[st_hor:st_hor + len(self.com_traj[i])], self.com_traj[i][:,2])

        if isinstance(com_real, np.ndarray):  
            com_real = np.array(com_real)[::int(self.dt/0.001)]
            ax[0].plot(x[:len(com_real)], com_real[:,0], "--",  label = "com real_x")
            ax[1].plot(x[:len(com_real)], com_real[:,1], "--",  label = "com real_y")
            ax[2].plot(x[:len(com_real)], com_real[:,2], "--",  label = "com real_z")

        ax[0].grid()
        ax[0].legend()

        ax[1].grid()
        ax[1].legend()

        ax[2].grid()
        ax[2].legend()

        plt.show()

    def plot_joints(self):
        self.xs_traj = np.array(self.xs_traj)
        self.xs_traj = self.xs_traj[:,:,:self.rmodel.nq]
        self.q_traj = np.array(self.q_traj)
        x = self.dt*np.arange(0, len(self.xs_traj[1]) + len(self.xs_traj), 1)
        # com plots
        fig, ax = plt.subplots(3,1)
        for i in range(len(self.xs_traj)):
            st_hor = i*int(self.planning_time/self.dt)
            ax[0].plot(x[st_hor], self.q_traj[i][10], 'o')
            ax[0].plot(x[st_hor:st_hor + len(self.xs_traj[i])], self.xs_traj[i][:,10])

        plt.show()

    def plot_plan(self):
        xs = self.ik.get_xs()
        opt_mom = np.zeros((len(xs), 6))
        opt_com = np.zeros((len(xs), 3))
        for i in range(len(xs)):
            q = xs[i][:self.rmodel.nq]
            v = xs[i][self.rmodel.nq:]
            pin.forwardKinematics(self.rmodel, self.rdata, q, v)
            pin.computeCentroidalMomentum(self.rmodel, self.rdata)
            opt_com[i] = pin.centerOfMass(self.rmodel, self.rdata, q, v)
            opt_mom[i] = np.array(self.rdata.hg)
            opt_mom[i][0:3] /= self.m

        self.mp.add_ik_com_cost(opt_com)
        self.mp.add_ik_momentum_cost(opt_mom) 
    
        self.mp.stats()