import pybullet as p
import pybullet_data
import time


class RRbotEnv:
    def __init__(self, urdf_path="/home/xin/codes/bullet3/pybullet/new_examples/model/rrbot.urdf",
                 ini_pos=[0, 0, 0],
                 ini_quat=[0, 0, 0, 1]):
        # set env
        physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        p.setGravity(0, 0, -9.81)
        planeId = p.loadURDF("plane.urdf")

        # set robot
        self.robotId = p.loadURDF(urdf_path, ini_pos, ini_quat)
        self._print_robot_inf(self.robotId)

        self.jointIds = [1, 2]
        p.setJointMotorControlArray(self.robotId, self.jointIds, p.VELOCITY_CONTROL, forces=[0.0, 0.0])
        p.setJointMotorControlArray(self.robotId, self.jointIds, p.TORQUE_CONTROL, forces=[0.0, 0.0])

    def _print_robot_inf(self, robotId):
        print("-" * 100)
        print("RobotInfo")
        print("Number of Joints: ", p.getNumJoints(robotId))
        print("JointsInfo:")
        print("jointIndex", "jointName", "jointType", "qIndex", "uIndex", "flags", "jointDamping", "jointFriction",
              "jointLowerLimit", "jointUpperLimit", "jointMaxForce", "jointMaxVelocity", "linkName", "jointAxis",
              "parentFramePos", "parentFrameOrn", "parentIndex")
        for i in range(p.getNumJoints(robotId)):
            print(p.getJointInfo(robotId, i))

    def _get_obs(self):
        # sense robot state
        robotPos, robotOrn = p.getBasePositionAndOrientation(self.robotId)
        # print("robot state: ", robotPos, robotOrn)
        joint_states = p.getJointStates(self.robotId, self.jointIds)
        print("joint state: ", joint_states)

        q = [joint_state[0] for joint_state in joint_states]
        qd = [joint_state[1] for joint_state in joint_states]
        # print("q: ", q)
        # print("qd: ", qd)

        state = {'q': q, 'qd': qd}
        return state

    def reset(self):
        return self._get_obs()

    def render(self):
        pass

    def step(self, action):
        p.setJointMotorControlArray(self.robotId, self.jointIds, p.TORQUE_CONTROL, forces=action['torque'])
        p.stepSimulation()
        time.sleep(1. / 240.)
        return self._get_obs()


def main():
    # reset env
    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    p.setGravity(0, 0, -9.81)
    planeId = p.loadURDF("plane.urdf")

    # reset robot
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

    # start simulation
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


if __name__ == "__main__":

    env = RRbotEnv()
    state = env.reset()
    while True:
        action = {'torque': p.calculateInverseDynamics(env.robotId, state['q'], state['qd'], [0] * 2)}
        state = env.step(action)
