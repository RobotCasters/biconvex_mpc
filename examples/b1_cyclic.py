import time
import numpy as np
import pinocchio as pin
import pybullet as p
from pdb import set_trace

from robot_properties_b1.config import B1Config
from robot_properties_b1.b1wrapper import B1Robot
from mpc.b1_cyclic_gen import B1MpcGaitGen
from motions.cyclic.b1_trot import trot
from motions.cyclic.b1_jump import jump


from envs.pybullet_env import PyBulletEnv
from controllers.robot_id_controller import InverseDynamicsController

## robot config and init
pin_robot = B1Config.buildRobotWrapper()
urdf_path = B1Config.urdf_path

n_eff = 4
q0 = np.array(B1Config.initial_configuration)
q0[0:2] = 0.0
q0[2] = 0.4

v0 = pin.utils.zero(pin_robot.model.nv)
x0 = np.concatenate([q0, pin.utils.zero(pin_robot.model.nv)])
f_arr = ["LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT"]

v_des = np.array([0.5, 0.0, 0.0])
w_des = 0.01

plan_freq = 0.05  # sec
update_time = 0.02  # sec (time of lag)

sim_t = 0.0
sim_dt = 0.001
index = 0
pln_ctr = 0

## Motion
gait_params = trot
lag = int(update_time / sim_dt)
gg = B1MpcGaitGen(pin_robot, urdf_path, x0, plan_freq, q0, None)

gg.update_gait_params(gait_params, sim_t)

robot = PyBulletEnv(B1Robot, q0, v0)
robot_id_ctrl = InverseDynamicsController(pin_robot, f_arr)
robot_id_ctrl.set_gains(gait_params.kp, gait_params.kd)

plot_time = 0  # Time to start plotting

solve_times = []

p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
# robot.start_recording("b1_slow_stepping.mp4")

for o in range(int(1000 * (plan_freq / sim_dt))):
    # this bit has to be put in shared memory
    q, v = robot.get_state()

    R = pin.Quaternion(np.array(q[3:7])).toRotationMatrix()
    rpy_vector = pin.rpy.matrixToRpy(R)
    rpy_vector[2] = 0.0
    fake_quat = pin.Quaternion(pin.rpy.rpyToMatrix(rpy_vector))

    q[3] = fake_quat[0]
    q[4] = fake_quat[1]
    q[5] = fake_quat[2]
    q[6] = fake_quat[3]

    contact_configuration = robot.get_current_contacts()

    if o == int(100 * (plan_freq / sim_dt)):
        gait_params = trot
        gg.update_gait_params(gait_params, sim_t)
        robot_id_ctrl.set_gains(gait_params.kp, gait_params.kd)

    if pln_ctr == 0:
        contact_configuration = robot.get_current_contacts()

        pr_st = time.time()
        xs_plan, us_plan, f_plan = gg.optimize(q, v, np.round(sim_t, 3), v_des, w_des)
        # gg.plot(q,v)
        # Plot if necessary
        # if sim_t >= plot_time:
        # gg.plot_plan(q, v)
        # gg.save_plan("trot")

        pr_et = time.time()
        solve_times.append(pr_et - pr_et)

    # first loop assume that trajectory is planned
    if o < int(plan_freq / sim_dt) - 1:
        xs = xs_plan
        us = us_plan
        f = f_plan

    # second loop onwards lag is taken into account
    elif pln_ctr == lag and o > int(plan_freq / sim_dt) - 1:
        # Not the correct logic
        # lag = int((1/sim_dt)*(pr_et - pr_st))
        lag = 0
        xs = xs_plan[lag:]
        us = us_plan[lag:]
        f = f_plan[lag:]
        index = 0

    tau = robot_id_ctrl.id_joint_torques(
        q,
        v,
        xs[index][: pin_robot.model.nq].copy(),
        xs[index][pin_robot.model.nq :].copy(),
        us[index],
        f[index],
        contact_configuration,
    )
    # tau = robot_id_ctrl.id_joint_torques(q, v, q0, v0, v0, np.zeros(12), contact_configuration)
    robot.send_joint_command(tau)

    time.sleep(0.0005)
    sim_t += sim_dt
    pln_ctr = int((pln_ctr + 1) % (plan_freq / sim_dt))
    index += 1


# robot.stop_recording()
