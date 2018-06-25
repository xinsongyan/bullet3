import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally

p.setGravity(0, 0, -9.81)
planeId = p.loadURDF("plane.urdf")
robotStartPos = [0, 0, 0]
robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF("/home/xin/codes/bullet3/pybullet/new_examples/model/rrbot.urdf", robotStartPos,
                     robotStartOrientation)
print("-" * 50)
print("robotId: ", robotId)
N = p.getNumJoints(robotId)
n = N - 1
print("NumJoints: ", n)

for i in range(p.getNumJoints(robotId)):
    print(p.getJointInfo(robotId, i))

jointIds = [1, 2]
p.setJointMotorControlArray(robotId, jointIds, p.VELOCITY_CONTROL, forces=[0.0, 0.0])
p.setJointMotorControlArray(robotId, jointIds, p.TORQUE_CONTROL, forces=[0.0, 0.0])

while True:
    # sense robot state
    robotPos, robotOrn = p.getBasePositionAndOrientation(robotId)
    print("robot state: ", robotPos, robotOrn)
    joint_states = p.getJointStates(robotId, jointIds)
    print("joint state: ", joint_states)

    q = [joint_state[0] for joint_state in joint_states]
    qd = [joint_state[1] for joint_state in joint_states]
    print("q: ", q)
    print("qd: ", qd)

    # controller
    qd_des = [0] * n  # gravity compensation
    torque_cmd = p.calculateInverseDynamics(robotId, q, qd, qd_des)
    p.setJointMotorControlArray(robotId, jointIds, p.TORQUE_CONTROL, forces=torque_cmd)

    # sim one step
    p.stepSimulation()

    time.sleep(1. / 240.)
