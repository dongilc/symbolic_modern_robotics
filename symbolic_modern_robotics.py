# Modern Robotics Python Symbolic Calculation Library, BSD License.
# Written by Dongil Choi, MyongJi University, South Korea. dongilc@mju.ac.kr
# Theoretical Background (Textbook) : Modern Robotics, Kevin M. Lynch and Frank C. Park
# Symbolic Calculation Functions : Skew Symmatric
# 2021 / 04 / 05
# How to use : 
#       import intelligent_robotics as ir
#       dir(ir)
#       help(ir.DH)

import sympy as s

###### Rotation Matrix
# Calculate so3 from 3-vector
def VecToso3(w):
    o1 = w[0]
    o2 = w[1]
    o3 = w[2]
    skew_w = s.Matrix([[0,-o3,o2],
                       [o3,0,-o1],
                       [-o2,o1,0]])
    return skew_w

###### Rotation Matrix
# Calculate 3-vector from so3
def so3ToVec(so3mat):
    w = s.Matrix([[0],[0],[0]]);
    w[0] = -so3mat[1,2]
    w[1] = so3mat[0,2]
    w[2] = -so3mat[0,1]
    return w

###### Rotation Matrix
# Calculate unit axis of rotation(omega_hat) and theta from 3-vector
def AxisAng3(w):
    norm = w.norm()
    w_hat = w/norm
    theta = norm
    
    return w_hat, theta

###### Rotation Matrix
# Calculate SO3 from so3 by Matrix exponential
def MatrixExp3(so3mat):
    w = so3ToVec(so3mat)
    w_hat,theta = AxisAng3(w)
    skew_w = VecToso3(w_hat)
    I = s.Matrix([[1,0,0],[0,1,0],[0,0,1]])
    R = s.simplify(I + s.sin(theta)*skew_w + (1-s.cos(theta))*skew_w*skew_w)
    return R
        
###### Rotation Matrix
# Calculate so3 from SO3 by Matrix logarithm
def MatrixLog3(SO3):
    R = SO3
    R_T = R.T
    tr_R = s.simplify(s.Trace(R))
    theta = s.simplify(s.acos((tr_R-1)/2))
    w_hat_skew = s.simplify(1/(2*s.sin(theta))*(R-R_T))
    so3 = s.simplify(w_hat_skew*theta)
    return so3

###### Homogeneous Transform Matrix
# Calculate se3 from 6-vector twist
def VecTose3(V):
    w = s.Matrix(V[0:3])
    v = s.Matrix(V[3:6])
    skew_w = VecToso3(w)
    se3 = s.Matrix([[skew_w,v],
                    [0,0,0,0]])
    return se3

###### Homogeneous Transform Matrix
# Calculate screw-axis and angle from 6-vector
def AxisAng6(V):
    w = s.Matrix(V[0:3])
    v = s.Matrix(V[3:6])
    w_norm = w.norm()
    
    if w_norm == 0:
        norm = v.norm()
    else:
        norm = w.norm()
        
    S = s.simplify(V/norm)
    theta_dot = norm
    return S,theta_dot 

###### Homogeneous Transform Matrix
# Calculate 6-vector twist from se3
def se3ToVec(se3mat):
    V = s.Matrix([[0],[0],[0],[0],[0],[0]]);
    V[0] = -se3mat[1,2]
    V[1] = se3mat[0,2]
    V[2] = -se3mat[0,1]
    V[3] = se3mat[0,3]
    V[4] = se3mat[1,3]
    V[5] = se3mat[2,3]
    return V

###### Homogeneous Transform Matrix
# Calculate SE3 from se3 by Matrix exponential
def MatrixExp6(se3mat):
    V = se3ToVec(se3mat)
    w = s.Matrix(V[0:3])
    
    if w.norm() == 0:
        v = s.Matrix(V[3:6])
        I = s.Matrix([[1,0,0],[0,1,0],[0,0,1]])
        SE3 = s.Matrix([[I,v],
                        [0,0,0,1]])
    else:
        w_hat, theta = AxisAng3(w)
        v = s.Matrix(V[3:6])/theta
        so3_hat = VecToso3(w_hat)
        R = MatrixExp3(so3_hat*theta)
        I = s.Matrix([[1,0,0],[0,1,0],[0,0,1]])
        G_theta = I*theta + (1-s.cos(theta))*so3_hat + (theta-s.sin(theta))*so3_hat*so3_hat
        p = s.simplify(G_theta*v)
        SE3 = s.Matrix([[R,p],
                        [0,0,0,1]])
    #return V,w,w_hat,theta,v,so3_hat,R,p,SE3
    return SE3

###### Homogeneous Transform Matrix
# Calculate se3 from SE3 by Matrix logarithm
def MatrixLog6(T):
    R = T[0:3,0:3]
    p = T[0:3,3]
    
    I = s.Matrix([[1,0,0],[0,1,0],[0,0,1]])
    
    if R == I:
        w_zero = s.Matrix([[0,0,0],[0,0,0],[0,0,0]])
        se3 = s.Matrix([[w_zero,p],
                        [0,0,0,0]]) 
    else:
        so3 = MatrixLog3(R)
        w = so3ToVec(so3)
        w_hat,theta = AxisAng3(w)
        so3_hat = VecToso3(w_hat)
        I = s.Matrix([[1,0,0],[0,1,0],[0,0,1]])
        G_inv_theta = s.simplify(1/theta*I - 1/2*so3_hat + (1/theta - 1/2*s.cot(theta/2))*so3_hat*so3_hat)
        v = s.simplify(G_inv_theta*p)
        se3 = s.Matrix([[so3,v],
                        [0,0,0,0]])
    #return R, p, so3, w, w_hat, theta, so3_hat, G_inv_theta, v, se3 
    return se3

###### Space Form PoE
# Computes forward kinematics in the space frame
def FKinSpace(M, Slist, thetalist):
    T = M
    for i in range(len(thetalist) - 1, -1, -1):
       T = s.simplify(MatrixExp6(VecTose3(Slist[:, i]*thetalist[i]))@T)
    return T

###### Body Form PoE
# Computes forward kinematics in the body frame
def FKinBody(M, Blist, thetalist):
    T = M
    for i in range(len(thetalist)):
       T = s.simplify(T@MatrixExp6(VecTose3(Blist[:, i]*thetalist[i])))
    return T

##### Adjoint of T
# Computes Adjoint of T
def Adjoint(T):
    R = T[0:3,0:3]
    p = T[0:3,3]
    
    Ad_T = s.Matrix([[R, s.Matrix.zeros(3)],
                     [VecToso3(p)@R ,R]])
    return Ad_T

def TransInv(T):
    R = T[0:3,0:3]
    p = T[0:3,3]

    T_inv = s.Matrix([[R.T, -R.T@p],
                      [0,0,0,1]])
    return T_inv

###### Body Jacobian
# Computes Body Jacobian
def JacobianBody(Blist, thetalist):
    Jb = Blist.copy()
    T = s.Matrix.eye(4)
    for i in range(len(thetalist) - 2, -1, -1):
        T = s.simplify( T @ MatrixExp6(VecTose3(Blist[:, i+1]*-thetalist[i+1])) )
        Jb[:, i] = s.simplify(Adjoint(T)@Blist[:, i])
    return Jb

###### Space Jacobian
# Computes Space Jacobian
def JacobianSpace(Slist, thetalist):
    Js = Slist.copy()
    T = s.Matrix.eye(4)
    for i in range(1, len(thetalist)):
        T = s.simplify( T @ MatrixExp6(VecTose3(Slist[:, i-1]*thetalist[i-1])) )
        Js[:, i] = s.simplify(Adjoint(T)@Slist[:, i])
        #print(i, Js[:, i])
    return Js

###### Numerical Inverse Kinematics using Newton-Rapton Method
# Computes inverse kinematics in the body frame
def IKinBody(Blist, M, Tsd, thetalist0, eomg, ev):
    thetalist = thetalist0
    i = 0
    maxiterations = 20
    T_sb = FKinBody(M, Blist, thetalist)
    T_sb_inv = TransInv(T_sb)
    T_bd = s.simplify(T_sb_inv@Tsd)
    Vb = se3ToVec(MatrixLog6(T_bd))
    #err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
    #while err and i < maxiterations:
    #    thetalist = thetalist + np.dot(np.linalg.pinv(JacobianBody(Blist, thetalist)), Vb)
    #    i = i + 1
    #    Vb = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(M, Blist, thetalist)), T)))
    #    err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
    #return (thetalist, not err)
    return Vb