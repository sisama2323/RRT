import pybullet as p
import math
from time import sleep
path= "D:/bullet3-master/data/"

p.connect(p.GUI)
p.resetSimulation()
p.setGravity(0,0,-9.8)
table=p.loadURDF(path+"table/table.urdf")
kukaId = p.loadURDF(path+"kuka_lwr/kuka.urdf",[0,0,0.6],useFixedBase=True)
kukaJoint=p.getNumJoints(kukaId)
ballId = p.loadURDF(path+"sphere_small.urdf",[0.3,-0.3,0.7],useFixedBase=True)


gripperId= p.loadSDF(path+"/gripper/wsg50_one_motor_gripper_new_free_base.sdf")[0]
gripperJoint=p.getNumJoints(gripperId)
const=p.createConstraint(kukaId,6,gripperId,0,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,0])
GRIPPER_CLOSED = [0.000000,-0.011130,-0.206421,0.205143,0.05,0.000000,0.05,0.000000]
GRIPPER_OPEN = [0.000000,-0.011130,-0.206421,0.205143,-0.01,0.000000,-0.01,0.000000]

for i in range (1,10):
    p.stepSimulation()


for i in range(kukaJoint):
    p.setJointMotorControl2(bodyIndex=kukaId, jointIndex=i, controlMode=p.POSITION_CONTROL, targetPosition=math.pi / 3)
    for j in range(100):
        p.stepSimulation()
        sleep(0.01)


for i in range(gripperJoint):
    p.setJointMotorControl2(bodyIndex=gripperId, jointIndex=i, controlMode=p.POSITION_CONTROL, targetPosition=GRIPPER_CLOSED[i])
    for j in range(5):
        p.stepSimulation()
        sleep(0.01)

for i in range(gripperJoint):
    p.setJointMotorControl2(bodyIndex=gripperId, jointIndex=i, controlMode=p.POSITION_CONTROL, targetPosition=GRIPPER_OPEN[i])
    for j in range(5):
        p.stepSimulation()
        sleep(0.01)


for i in range(1,100):
    p.stepSimulation()
    sleep(0.01)


