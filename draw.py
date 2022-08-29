from cProfile import label
import numpy as np
import matplotlib.pyplot as plt
file_name = "text.txt"
save_name = "text1.png"
read_path = "/home/zhoujin/log_ws/src/UAV_logger/src/" + file_name
data = np.loadtxt(read_path, delimiter=',', dtype=str, skiprows=200)
data = np.float32(data)
fig, ax = plt.subplots(4, 3,figsize=(14, 10))
# for i in range(3):
#     ax[0][i].plot(data[:,0], data[:,i+1], 'r*', label="local")

# ax[0][0].plot(data[:,0], 1*np.ones((data.shape[0],1)), 'b-', label = "target")
# ax[0][1].plot(data[:,0], 1*np.ones((data.shape[0],1)), 'b-', label = "target")
# ax[0][2].plot(data[:,0], 1.5*np.ones((data.shape[0],1)), 'b-', label = "target")

for i in range(2):
    for j in range(3):
        ax[i][j].plot(data[:,0], data[:,i*6+j+1], 'r-')  # 1 2 3    7 8 9 
        ax[i][j].plot(data[:,0], data[:,i*6+j+4], 'b-')  # 4 5 6    10 11 12

for j in range(3):
    ax[2][j].plot(data[:,0], data[:,12+j+1], 'r-', label = "current")  # 13 14 15
    ax[2][j].plot(data[:,0], data[:,12+j+7], 'b-', label = "feedfoward") # 16 17 18
    ax[2][j].plot(data[:,0], data[:,12+j+4], 'y-', label = "cmd") # 19 20 21
    ax[3][j].plot(data[:,0], data[:,21+j+1], 'r-') 
    ax[3][j].plot(data[:,0], data[:,21+j+4], 'b-') 


for j in range(3):
    ax[0][j].set_ylabel("x/m")
    ax[0][j].set_xlabel("t/s")
    ax[1][j].set_ylabel("v/m/s")
    ax[1][j].set_xlabel("t/s")
    ax[2][j].set_ylabel("e/deg")
    ax[2][j].set_xlabel("t/s")
    ax[3][j].set_ylabel("r/rad/s")
    ax[3][j].set_xlabel("t/s")

# ax[0, 0].set_title("position x")
# ax[0, 1].set_title("position y")
# ax[0, 2].set_title("position z")
# ax[1, 0].set_title("velocity x")
# ax[1, 1].set_title("velocity y")
# ax[1, 2].set_title("velocity z")
# ax[2, 0].set_title("attitude x")
# ax[2, 1].set_title("attitude y")
# ax[2, 2].set_title("attitude z")
# ax[3, 0].set_title("body_rate x")
# ax[3, 1].set_title("body_rate y")
# ax[3, 2].set_title("body_rate z")
ax[0, 0].set_title("pos x")
ax[0, 1].set_title("pos y")
ax[0, 2].set_title("pos z")
ax[1, 0].set_title("vel x")
ax[1, 1].set_title("vel y")
ax[1, 2].set_title("vel z")
ax[2, 0].set_title("euler x")
ax[2, 1].set_title("euler y")
ax[2, 2].set_title("euler z")
ax[3, 0].set_title("body_rate x")
ax[3, 1].set_title("body_rate y")
ax[3, 2].set_title("body_rate z")

lines, labels = fig.axes[-1].get_legend_handles_labels()
fig.legend()
fig.tight_layout()
save_path = "/home/zhoujin/log_ws/src/UAV_logger/src/" + save_name
plt.savefig(save_path, dpi=300)
plt.show()





