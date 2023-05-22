# Notes
## Table of Contents

## Engineering

### Quaternions
https://www.3dgep.com/understanding-quaternions/
Quaternion is vector with 4 elements representing either [w,x,y,z] or [q0,q1,q2,q3]
- w: scalar pf quaternion is angle of rotation
- x,y,z: quaternion (similar to 3d complex number where in addition of i there is j and k) is the axis around which rotation is performed

We can concert between w,x,y,z and q0,1,2,3:
q[q0,q1,q2,q3]  =[cos(w/2), x sin(w/2), y sin(w/2), z sin(w/2)]
w = 2 inv_cos(qo)
x,y,z = q1/sin(w/2), q2/cos(w/2), q3/sin(w/2)

Since quaternion is representing a rotation we can also convert q to [3x3] rotation matrix

Typical mathamtical operations: +,-,*,/,conjugate,norm

### 
q1 * (q2 * q3) = (q1 * q2) * 13
q1 * q2 != q2 8 q1 

#### Euler angels u,v,w = roll, pitch, yaw
We can convert between q and RPY, UVW
- q.conjugate = [w,-x,-y,-z]
- q.inverse = q.conjugate / q.norm()^2

#### norm
norm = sqrt(w^2 + x^2, y^2 + z^2)

#### dot product q1,q2
Represents angle between two quaternions

#### Attitude
Attitude is quanternion representing rotation of device frame from world frame
Attitude unit should be 1.

#### Rotating Gravity to device frame
    g = Quaternion([0, 0, 0, -1])
    pose = p 
    g * p

#### Rotating Vector in device fame to world frame
Vexctor [x,y,z] on sensor to world frame with pose q
-1) create p = [0,x,y,z]
-2) quaternion of p in the world frame p_world = q * p * q.conjugate()
-3) [x,y,z] in world frame is x,y,z component p_world

### python AHRS  https://ahrs.readthedocs.io/en/latest/quaternion/classQuaternion.html

Quaternion(from, vector(3d), q(4d), rpy, dcm)
- Math: =,-,*,/
- Attributes
   - q.w,x,y,z,v (vector part)
   - conjugate or conj, inverse, eponention or exp, logarithm or log
- Methods
   - is_pure(), is_real(), is_vector(), is_identity()
   - normalize(), q1.product(q2), mult_L(), mult_R(), rotate()
   - to_array(), to_list(), to_axang(), to_angles(), to_DCM()
- Methods to create q
    - from_DCM(), from_rpy() or from_angles(),
    - ode, random

### Testing AHRS
'''
import numpy as np
from ahrs import q_conj, Quaternion, DEG2RAD, RAD2DEG
g = Quaternion(np.array([0,0,0,-1])) # gravity
# RYP
p = Quaternion.from_rpy(Quaternion, (90*DEG2RAD,0,0))
'''

### AHRS
#### Roll Pitch Yaw or phi theta psi

q in w,x,y,z

This matches Wiki and has rotation order Z,Y,X

https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

-w = cos(Y/2)*cos(P/2)*cos(R/2) + sin(Y/2) * sin(P/2) * sin(R/2)
-x = cos(Y/2)*cos(P/2)*cos(R/2) - sin(Y/2) * sin(P/2) * sin(R/2)
-y = cos(Y/2)*sin(P/2)*cos(R/2) + sin(Y/2) * cos(P/2) * sin(R/2)
-z = sin(Y/2)*cos(P/2)*cos(R/2) - cos(Y/2) * sin(P/2) * sin(R/2)

RPY +90,   0,  0 => q = [0.71, 0,    0,    0.71]

RPY -90,   0,  0 => q = [0.71, 0,    0,   -0.71]

RPY   0,  90,  0 => q = [0.71, 0,    0.71, 0]

RPY   0, -90,  0 => q = [0.71, 0,   -0.71, 0]

RPY   0,   0, 90 => q = [0.71, 0.71, 0,    0]

RPY   0,   0,-90 => q = [0.71,-0.71, 0,    0]

#### Numpy Quaternions
q in w,x,y,z
quaternion.as_euler_angles
quaternion.from_euler_angles

YRP

alpha, beta, gamma first beta around z then alpha around y then gamma around z
1) right-handed rotation about the z axis through an angle gamma (yaw)
2) right-handed rotation about the (non-rotated) y axis through an angle beta (pitch) 
3) right-handed rotation about the (non-rotated) z axis through an angle alpha.

https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
This does not match Wiki
R                       [q0,q1,q2,q3]
RPY +90,   0,  0 => q = [0.71, 0.71, 0,    0]
RPY -90,   0,  0 => q = [0.71,-0.71, 0,    0]
Y
RPY   0,   0, 90 => q = [0.71, 0,    0, 0.71]
RPY   0,   0,-90 => q = [0.71, 0,    0,-0.71]
P
RPY   0,  90,  0 => q = [0.71, 0, 0.71,    0]
RPY   0, -90,  0 => q = [0.71, 0,-0.71,    0]



