from pandas import read_csv
import matplotlib.pyplot as plt

file_name = "/home/michael/HRI/ros2_ws/src/cpp_pubsub/data_logging/csv_logs/part0/trial13.csv"

df = read_csv(file_name, header=None)

hxs = df.iloc[:,0].tolist()
hys = df.iloc[:,1].tolist()
hzs = df.iloc[:,2].tolist()

rxs = df.iloc[:,3].tolist()
rys = df.iloc[:,4].tolist()
rzs = df.iloc[:,5].tolist()

txs = df.iloc[:,6].tolist()
tys = df.iloc[:,7].tolist()
tzs = df.iloc[:,8].tolist()

ax = plt.figure().add_subplot(projection='3d')

# ax.plot(hxs, hys, hzs, label="human_positions")
# ax.plot(rxs, rys, rzs, label="robot_positions")
# ax.plot(txs, tys, tzs, label="total_positions")

ax.legend()

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

plt.show()