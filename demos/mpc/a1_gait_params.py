## Contains A1 gait params
## Author : Paarth shah
## Date : 7/7/21

import numpy as np
from weight_abstract import BiconvexMotionParams
from robot_properties_a1.config import A1Config

pin_robot = A1Config.buildRobotWrapper()
urdf_path = A1Config.urdf_path

#### Stand Still #########################################
still = BiconvexMotionParams("a1", "Stand")

# Cnt
still.gait_period = 0.5
still.stance_percent = [1.0, 1.0, 1.0, 1.0]
still.gait_dt = 0.05
still.phase_offset = [0.0, 0.4, 0.4, 0.0]

# IK
still.state_wt = np.array([0., 0, 0] + [10.0] * 3 + [50.0] * (pin_robot.model.nv - 6) \
                          + [0.00] * 3 + [0.01] * 3 + [100.0] *(pin_robot.model.nv - 6))

still.ctrl_wt = [0, 0, 0.0] + [0.0, 0.0, 0.0] + [0.0] *(pin_robot.model.nv - 6)

still.swing_wt = [1e4, 1e4]
still.cent_wt = [5e+5, 5e+4]
still.step_ht = 0.02
still.nom_ht = 0.300
still.reg_wt = [5e-4, 9e-6]

# Dyn
still.W_X =        np.array([1e+3, 1e+3, 1e+6, 1e+1, 1e+1, 2e+3, 1e+4, 1e+4, 1e4])
still.W_X_ter = 10*np.array([1e+3, 1e+3, 1e+6, 1e+1, 1e+1, 2e+3, 1e+5, 1e+5, 1e+5])
still.W_F = np.array(4*[1e+0, 1e+0, 1e+0])
still.rho = 5e+4
still.ori_correction = [0.4, 0.5, 0.4]
still.gait_horizon = 2.0

# Gains
still.kp = 7.0
still.kd = 0.4

#### Gallop Still #########################################
gallop = BiconvexMotionParams("a1", "Gallop")

# Cnt
gallop.gait_period = 0.5
gallop.stance_percent = [0.45, 0.45, 0.45, 0.45] #FL, FR, HL, HR
gallop.gait_dt = 0.05
gallop.phase_offset = [0.0, 0.20, 0.30, 0.50]

# IK
gallop.state_wt = np.array([0.0, 0.0, 10.0] + [5000] * 3 + [0.0, 0.0, 50.0] * 4 \
                           + [0.0, 0.0, 0.0]  + [1000] * 3 + [30.0, 60.0, 60.0] * 4)

gallop.ctrl_wt = [0, 0, 1000] + [5e2, 5e2, 5e2] + [1.0] *(pin_robot.model.nv - 6)

gallop.swing_wt = [1e6, 1e4]
gallop.cent_wt = [5e+1, 5e+2]
gallop.step_ht = 0.08
gallop.nom_ht = 0.26
gallop.reg_wt = [5e-2, 1e-5]

# Dyn
gallop.W_X =        np.array([1e-5, 1e-5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+5, 1e+5, 1e4])
gallop.W_X_ter = 10*np.array([1e+5, 1e+5, 1e+5, 1e+1, 1e+1, 2e+2, 1e+5, 1e+5, 1e+5])
gallop.W_F = np.array(4*[1e+1, 1e+1, 1e+1])
gallop.rho = 5e+4
gallop.ori_correction = [0.4, 0.7, 0.4]
gallop.gait_horizon = 2.0

# Gains
gallop.kp = 5.0
gallop.kd = 0.2


#### Trot #########################################
trot = BiconvexMotionParams("a1", "Trot")

# Cnt
trot.gait_period = 0.4
trot.stance_percent = [0.65, 0.65, 0.65, 0.65]
trot.gait_dt = 0.04
trot.phase_offset = [0.0, 0.5, 0.5, 0.0]

# IK
trot.state_wt = np.array([0., 0, 100.0] + [10.0] * 3 + [10.0] * (pin_robot.model.nv - 6) \
                         + [0.00] * 3 + [.01] * 3 + [500.0] *(pin_robot.model.nv - 6))

trot.ctrl_wt = [0, 0, 1000] + [5e2, 5e2, 5e2] + [1.0] *(pin_robot.model.nv - 6)

trot.swing_wt = [1e5, 1e4]
trot.cent_wt = [5e5, 5e+2]
trot.step_ht = 0.25
trot.nom_ht = 0.30
trot.reg_wt = [5e-4, 9e-6]

# Dyn
trot.W_X =       np.array([1e5, 1e5, 1e+6, 1e+1, 1e+1, 2e+3, 1e+5, 1e+5, 1e4])
trot.W_X_ter = 10*np.array([1e+5, 1e+5, 1e+6, 1e+1, 1e+1, 2e+3, 1e+5, 1e+5, 1e+5])
trot.W_F = np.array(4*[1e-1, 1e-1, 1e-1])
trot.rho = 5e+4
trot.ori_correction = [0.6, 0.6, 0.4]
trot.gait_horizon = 2.0

# Gains
trot.kp = 8.0
trot.kd = 0.3

#### Walking #######################################
# walk = BiconvexMotionParams("a1", "walk")

# # Cnt
# walk.gait_period = 0.6
# walk.stance_percent = [0.8, 0.8, 0.8, 0.8]
# walk.gait_dt = 0.05
# walk.phase_offset = [0.6, 0.0, 0.2, 0.8]

# # IK
# walk.state_wt = np.array([0., 0, 10] + [1e3] * 3 + [50.0] * (pin_robot.model.nv - 6) \
#                         + [0.00] * 3 + [50] * 3 + [1e-2] *(pin_robot.model.nv - 6))

# walk.ctrl_wt = [0, 0, 2] + [10, 10, 20] + [5e-3] *(pin_robot.model.nv - 6)

# walk.swing_wt = [1e4,1e4]
# walk.cent_wt = [5e+1, 1e+2]
# walk.step_ht = 0.06
# walk.reg_wt = [5e-3, 7e-3]

# # Dyn
# walk.W_X =        0.05*np.array([1e-5, 1e-5, 1e+5, 1e2, 1e2, 2e+2, 1e+3, 4e+3, 4e3])
# walk.W_X_ter = 100*np.array([1e-5, 1e-5, 1e+4, 1e2, 1e2, 1e+4, 4e+3, 4e+3, 4e+3])
# walk.W_F = np.array(4*[5e+1, 5e+1, 5e+1])
# walk.nom_ht = 0.22
# walk.rho = 5e+4
# walk.ori_correction = [0.0, 0.0, 0.0]
# walk.gait_horizon = 5.0

# # Gains
# walk.kp = 10.0
# walk.kd = 0.15

walk = BiconvexMotionParams("a1", "walk")

# Cnt
walk.gait_period = 0.6
walk.stance_percent = [0.8, 0.8, 0.8, 0.8]
walk.gait_dt = 0.05
walk.phase_offset = [0.6, 0.0, 0.2, 0.8]

# IK
walk.state_wt = np.array([0., 0, 1000] + [1e3] * 3 + [0.5] * (pin_robot.model.nv - 6) \
                         + [0.00] * 3 + [50] * 3 + [1e-2] *(pin_robot.model.nv - 6))

walk.ctrl_wt = [1, 1, 10] + [10, 10, 20] + [5e-3] *(pin_robot.model.nv - 6)

walk.swing_wt = [1e4,1e4]
walk.cent_wt = [5e+1, 5e+2]
walk.step_ht = 0.05
walk.reg_wt = [5e-3, 7e-3]

# Dyn
walk.W_X =        np.array([1e-5, 1e-5, 1e+5, 1e2, 1e2, 1e+2, 5e+3, 5e+3, 5e3])
walk.W_X_ter = 10*np.array([1e-5, 1e-5, 1e+5, 1e2, 1e2, 1e+2, 1e+3, 1e+3, 1e+3])
walk.W_F = np.array(4*[1e+1, 1e+1, 1e+1])
walk.nom_ht = 0.24
walk.rho = 5e+4
walk.ori_correction = [0.2, 0.4, 0.5]
walk.gait_horizon = 0.5

# Gains
walk.kp = 3.5
walk.kd = 0.15

#### Bound #######################################
# bound = BiconvexMotionParams("a1", "bound")

# # Cnt
# bound.gait_period = 0.3
# bound.stance_percent = [0.5, 0.5, 0.5, 0.5]
# bound.gait_dt = 0.05
# bound.phase_offset = [0.0, 0.0, 0.5, 0.5]

# # IK
# bound.state_wt = np.array([0., 0, 1e3] + [10, 10, 10] + [50.0] * (pin_robot.model.nv - 6) \
#                         + [0.00] * 3 + [100, 10, 100] + [0.5] *(pin_robot.model.nv - 6))

# bound.ctrl_wt = [0.5, 0.5, 0.5] + [1, 1, 1] + [0.5] *(pin_robot.model.nv - 6)

# bound.swing_wt = [1e4, 1e4]
# bound.cent_wt = [5e+1, 5e+2]
# bound.step_ht = 0.07
# bound.reg_wt = [7e-3, 7e-5]

# # Dyn
# bound.W_X =        np.array([1e-5, 1e-5, 5e+4, 1e1, 1e1, 1e+3, 5e+3, 1e+4, 5e+3])
# bound.W_X_ter = 10*np.array([1e-5, 1e-5, 5e+4, 1e1, 1e1, 1e+3, 1e+4, 1e+4, 1e+4])
# bound.W_F = np.array(4*[1e+1, 1e+1, 1e+1])
# bound.nom_ht = 0.25
# bound.rho = 5e+4
# bound.ori_correction = [0.2, 0.8, 0.8]
# bound.gait_horizon = 2.0

# # Gains
# bound.kp = 3.0
# bound.kd = 0.05


bound = BiconvexMotionParams("a1", "bound")

# Cnt
bound.gait_period = 0.5
bound.stance_percent = [0.70, 0.70, 0.70, 0.70]
bound.gait_dt = 0.05
bound.phase_offset = [0.0, 0.0, 0.5, 0.5]

# IK
bound.state_wt = np.array([0., 0, 1000] + [10, 10, 10] + [10.0] * (pin_robot.model.nv - 6) \
                          + [0.00] * 3 + [.01, .01, .01] + [500.0] *(pin_robot.model.nv - 6))

bound.ctrl_wt = [0.0, 0.0, 0.0] + [1, 1, 1] + [1.0] *(pin_robot.model.nv - 6)

bound.swing_wt = [1e5, 1e4]
bound.cent_wt = [5e+2, 5e+4]
bound.step_ht = 0.23
bound.reg_wt = [5e-4, 9e-4]

# Dyn
bound.W_X =        np.array([1e+3, 1e+3, 1e+6, 1e+1, 1e+1, 2e+3, 1e+4, 1e+6, 1e4])
bound.W_X_ter = 10*np.array([1e+5, 1e+5, 1e+6, 1e+1, 1e+1, 2e+3, 1e+5, 1e+6, 1e+5])
bound.W_F = np.array(4*[1e+0, 1e+0, 1e+0])
bound.nom_ht = 0.28
bound.rho = 5e+4
bound.ori_correction = [0.6, 0.8, 0.8]
bound.gait_horizon = 2.0

# Gains
bound.kp = 9.0
bound.kd = 0.4