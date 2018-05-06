import numpy as np
from pyquaternion import Quaternion
import matplotlib.pyplot as plt
from track_orientation import *

def load_data(path):
    position_path = 'position_data.npy'
    quaternion_path = 'quaternion_data.npy'
    position_data = np.load(path+position_path)
    quaternion_data = np.load(path+quaternion_path)
    return position_data, quaternion_data

def main():
    '''
    loading data and set configurations
    '''
    position_data, quaternion_data = load_data('data/')
    print(position_data[0], quaternion_data[0])
    N = len(position_data)
    # two satelites at (0,0,20200*1000) and (1e7,1e7,20200*1000)
    satelite_a = Satelite(0,0)
    satelite_b = Satelite(1e7,1e7)
    satelite_a.line_of_sight(position_data[0])
    satelite_b.line_of_sight(position_data[0])
    wavelength = satelite_a.wavelength
    # the line_of_sight unit vector is approximate as constant vector
    diff_los = satelite_a.los-satelite_b.los
    rx_num = 4
    arm_len = 0.2 # 0.2 meter from the center to the rx antenna
    '''
    simulating phase based on the data generated
    '''
    base_drone = Drone(location=(0,0,0),quaternion=(1,0,0,0))
    base_drone.rotate(arm_length=arm_len)
    base_matrix = np.array([base_drone.rx[i]-base_drone.rx[0] for i in range(1,rx_num)])
    phase = np.zeros((N,rx_num,2))
    drone_stat = []
    # each drone instance represents a time slot
    for i in range(N):
        drone = Drone(position_data[i],quaternion_data[i])
        drone.rotate(arm_len)
        phase[i,:,0] = drone.phase_calculate(satelite_a)
        phase[i,:,1] = drone.phase_calculate(satelite_b)
        drone_stat.append(drone)
    # system model, calculating all differentials
    model = Model(N, rx_num)
    model.diff_calculate(phase)
    for i in range(1,2):
        for j in range(rx_num-1):
            H = np.inner(base_matrix[j],np.cross(drone_stat[i-1].quaternion.rotation_matrix,diff_los))
            yy = np.inner(np.inner(base_matrix[j],drone_stat[i-1].quaternion.rotation_matrix),diff_los)
            y = wavelength*model.doubleDiffRxSate[j]
            # wavelength*model.doubleDiffRxTime[i] =

if __name__ == "__main__":
    main()

# y and quaternion

for i in range(N):
    for j in range(rx_num-1):
     quaternion_measurement[i] = N[i][j]*satelite_a.wavelength+satelite_a.wavelength*Model.doubleDiffRxSate[i][j]
  
# Karman filter 
     from numpy import dot
 
#X : The mean state estimate of the previous step ( k −1). P : The state covariance of previous step ( k −1).
#A : The transition n × n matrix.
#Q : The process noise covariance matrix.
#B : The noise effect matrix. U : The control noise.    
def kf_predict(X, P, A, Q, B, U):
    X = dot(A, X) + dot(B, U)
    P = dot(A, dot(P, A.T)) + Q
    return(X,P)

#K : the Kalman Gain matrix
#IM : the Mean of predictive distribution of Y
#IS : the Covariance or predictive mean of Y
#LH : the Predictive probability (likelihood) of measurement which is computed using the Python function gauss_pdf
from numpy import dot, sum, tile, linalg
from numpy.linalg import inv

def kf_update(X, P, Y, H, R): 
    IM = dot(H, X)
    IS = R + dot(H, dot(P, H.T)) 
    K = dot(P, dot(H.T, inv(IS))) 
    X = X + dot(K, (Y-IM))
    P = P - dot(K, dot(IS, K.T)) 
    LH = gauss_pdf(Y, IM, IS) 
    return (X,P,K,IM,IS,LH)

def gauss_pdf(X, M, S): 
    if M.shape()[1] == 1:
        DX = X - tile(M, X.shape()[1])
        E = 0.5 * sum(DX * (dot(inv(S), DX)), axis=0)
        E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S)) 
        P = exp(-E)
    elif X.shape()[1] == 1:
        DX = tile(X, M.shape()[1])- M
        E = 0.5 * sum(DX * (dot(inv(S), DX)), axis=0)
        E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S))
        P = exp(-E)
    else:
        DX = X-M
        E = 0.5 * dot(DX.T, dot(inv(S), DX))
        E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S)) 
        P = exp(-E)
    return (P[0],E[0])


#particle filter
    tau=100
def particle(tau): 
    x = np.zeros(tau) #initialize space
    y = np.zeros(tau)
    x[1] =0
    ##add y[1]=x[1]+noise 
    for i in range(2,tau):
        #x[i]=x[i-1] state formular
        #y[i]=x[i]#measurement formular
    
    #Begin particle filter
    N=500
    x_pf = np.zeros((tau+1,N))
    #initialize
    x_pf[0] = numpy.random.normal(size=N)
    m = np.zeros(tau)
    for t in range(2,(tau+1)):
        #add x_pf[t] =x[t-1] state formular
    
    from scipy.stats import norm
    #likelihood
        w_tlide = norm.pdf(y[t-1],loc=,scale=wavelength**2)
     #normalize
    
    w= w_tlide/sum(w_tlide)   
    #estimate of state
    m[t-1]=sum(w*x_pf[t])
    
    #resample step
      s = numpy.random.choice(range(N), size=N, replace=True, p=w) 
      x_pf =x_pf[s]
 return(m)       

   
 #  use Karman or particle
 import math
 
    if math.cos(2*math.pi*sum(N))> 5.6:
    #kalman filter
else:
    # particle filter


     