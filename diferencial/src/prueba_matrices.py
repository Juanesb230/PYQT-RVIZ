import numpy as np
import math

p_ref_ant=np.array([[0.0],[0.0]])
p_ref=np.array([[4.0],[4.0]])
x=0.0
y=0.0
theta=0.0

def Control(angle,x_pos,y_pos,pos_before,pos_ref,K_sin):
    Jr=np.array([[math.cos(angle) ,-0.1 * math.sin(angle)],[math.sin(angle) ,0.1 * math.cos(angle)]])
    Jr_inv=np.linalg.inv(Jr)
    K=np.array([[K_sin[0], 0],[0, K_sin[1]]])
    posp_d=pos_ref - pos_before
    pos=np.array([[x_pos],[y_pos]])
    pos_error=pos_ref - pos
    return np.dot(Jr_inv, (10 * posp_d + np.dot(K,pos_error)))

c=Control(0.7, 2, 3, p_ref_ant, p_ref, [0.1, 0.1])
linear = c[0, 0]
print(linear)