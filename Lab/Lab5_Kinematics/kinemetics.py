import numpy as np
from scipy.spatial.transform import Rotation

def get_T(q,d,a,alpha):
    
    cq = np.cos(q)
    sq = np.sin(q)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    matrix = np.array([[cq, -sq*ca, sq*sa, a*cq],
                    [sq, cq*ca, -cq*sa, a*sq],
                    [0, sa, ca, d],
                    [0, 0, 0, 1]])
    return matrix

PI = np.pi
ALPHA=np.array([PI/2,PI,PI/2,PI/2,PI/2,0])
A=np.array([0,280.0,0,0,0,0])
D=np.array([128.3+115.0, 30.0, 20.0, 140.0+105.0, 28.5+28.5, 105.0+130.0])
BASE = np.array([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]]) #See ZERO: rotated 90 degrees about z-axis

def transform(q_list: np.array)->np.array:
    # D-H parameters
    q = q_list/180*PI + np.array([0,PI/2,PI/2,PI/2,PI,PI/2])
    
    Ts = []
    for i in range(6):
        T = get_T(q[i], D[i], A[i], ALPHA[i])
        Ts.append(T)
        
    Ts.append(np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.1300],[0,0,0,1]])) #Frame 6 to gripper

    T = Ts[0]@Ts[1]@Ts[2]@Ts[3]@Ts[4]@Ts[5]@Ts[6]

    return T@BASE

input_names = ['HOME', 'ZERO', 'RETRACT', 'PACKAGING', 'PICK']

def kinemetics(inputs):
    outputs = []
    transformed_mats = []
    
    for i in range(5):
        transformed_mat = transform(inputs[i])
        transformed_mats.append(transformed_mat)

        transformed_mat_rounded = np.round(transformed_mat, decimals=1)
        print(f'{input_names[i]}:\n{transformed_mat}')

        px, py, pz = np.round(transformed_mat[:3,3], 1)
        print(f'Position: {px, py, pz}')

        rotation_mat = transformed_mat[:3,:3]
        r = Rotation.from_matrix(rotation_mat)
        euler_angles = r.as_euler('xyz', degrees=True)
        qx, qy, qz = np.round(euler_angles, 1)
        print(f'Euler angles: {qx, qy, qz}')
        print('--------------------------------')

        outputs.append([px, py, pz, qx, qy, qz])
    
    return outputs

inputs = np.array([[0,345,75,0,300,0],[0,0,0,0,0,0],[357,21,150,272,320,273],[270,148,148,270,140,0],[20.5,313.5,100,265.5,327,57]])

outputs = kinemetics(inputs)

for i in range(5):
    print(f'{outputs[i]}\n')
