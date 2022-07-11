from time import sleep
import pybullet as p
import pybullet_data
import rospkg

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('spot_mini_ros')
urdf_path = pkg_path + '/urdf/spot.urdf'

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0.5]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
spotId = p.loadURDF(urdf_path, cubeStartPos, cubeStartOrientation)

j1 = p.addUserDebugParameter('j1', -3, 3, 0)
j2 = p.addUserDebugParameter('j2', -3, 3, 0)
j3 = p.addUserDebugParameter('j3', -3, 3, 0)
j4 = p.addUserDebugParameter('j4', -3, 3, 0)
# j2 = p.addUserDebugParameter('', -3, 3, 0)
# j2 = p.addUserDebugParameter('', -3, 3, 0)
# j2 = p.addUserDebugParameter('', -3, 3, 0)
# j2 = p.addUserDebugParameter('', -3, 3, 0)
# j2 = p.addUserDebugParameter('', -3, 3, 0)
# j2 = p.addUserDebugParameter('', -3, 3, 0)
# j2 = p.addUserDebugParameter('', -3, 3, 0)

i = 0
while i < 10000:

    j1r = p.readUserDebugParameter(j1)
    j2r = p.readUserDebugParameter(j2)
    j3r = p.readUserDebugParameter(j3)
    j4r = p.readUserDebugParameter(j4)
    p.stepSimulation()
    # cubePos, cubeOrn = p.getBasePositionAndOrientation(spotId)
    # number_of_joints = p.getNumJoints(spotId)
    # print(cubePos, number_of_joints)
    p.setJointMotorControl2(spotId, 1, p.POSITION_CONTROL, targetPosition=j1r)
    p.setJointMotorControl2(spotId, 2, p.POSITION_CONTROL, targetPosition=j2r)
    p.setJointMotorControl2(spotId, 3, p.POSITION_CONTROL, targetPosition=j3r)
    p.setJointMotorControl2(spotId, 4, p.POSITION_CONTROL, targetPosition=j4r)
    sleep(0.001)
    i += 1 
# p.disconnect()