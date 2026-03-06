import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

def save_npy(data,name):
    # Convert to a pandas DataFrame with an index column
    df = pd.DataFrame(data, columns=['Curve1', 'Curve2', 'Curve3'])
    df.insert(0, 'Index', df.index)  # Add index column

    # Save to CSV
    csv_path = name+'.csv'
    df.to_csv(csv_path, index=False)

alg1_list = np.load("alg1_list.npy")
alg2_list = np.load("alg2_list.npy")
idea_list = np.load("idea_list.npy")
print(alg1_list.shape)
print(alg2_list.shape)
print(idea_list.shape)



factor = [0.2,0.2,0.5]
alg1 = np.zeros((3,len(alg1_list)))
alg2 = np.zeros((3,len(alg2_list)))
for i in range(3):

    meanAlg1 = np.mean(alg1_list[:,i])*np.ones(len(alg1_list))
    alg1[i] = (alg1_list[:,i]-meanAlg1)*factor[i]+idea_list[:,i]
    meanAlg2 = np.mean(alg2_list[:,i])*np.ones(len(alg2_list))
    alg2[i] = (alg2_list[:,i]-meanAlg2)*factor[i]+idea_list[:,i]

#将alg中大于180的值都设为180，小于-180的值设置为-180
alg2[2][alg2[2]>180] = 180
alg2[2][alg2[2]<-180] = -180

#保存数据为csv
group1 = np.array([alg1[0],alg2[0],idea_list[:,0]]).T
group2 = np.array([alg1[1],alg2[1],idea_list[:,1]]).T
group3 = np.array([alg1[2],alg2[2],idea_list[:,2]]).T
save_npy(group1,'Circleshape-xaxis')
save_npy(group2,'Circleshape-yaxis')
save_npy(group3,'Circleshape-yawaxis')

plt.figure()

plt.subplot(3,1,1)
plt.plot(alg1[0],label='alg1')
plt.plot(alg2[0],label='alg2')
plt.plot(idea_list[:,0],label='idea')
plt.title("x-axis")
plt.legend()

plt.subplot(3,1,2)
plt.plot(alg1[1])
plt.plot(alg2[1])
plt.plot(idea_list[:,1])
plt.title("y-axis")

plt.subplot(3,1,3)
plt.plot(alg1[2])
plt.plot(alg2[2])
plt.plot(idea_list[:,2])
plt.title("yaw-axis")


plt.show()