from ur_kinematics import Kinematics
kin = Kinematics('ur5') # or ur10
a = kin.forward([0]*6)
print a

b = kin.inverse(a)
print b