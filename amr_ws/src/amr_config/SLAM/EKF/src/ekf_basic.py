# https://github.com/NekSfyris/EKF_Lidar_Odometry/blob/master/ekf.py
import pickle


with open("/home/dj/Mobile_robotics/amr_config/SLAM/data/data.pickle", "rb") as f:
    data = pickle.load(f)

# print(data)

time_stmap = data["t"]
v = data["v"]
om = data["om"]

b = data["b"]  # array of 2d angles [[angle1],[angle2], ..,[anglen]] angle=[th1, th_2, th_3, .., th_n]
r = data["r"]  # array of 2d distance [[ranges1],[ranges2], ..,[rangesn]] ranges=[d_1, d_2, d_3, .., d_n]

# print(len(time_stmap), len(v), len(om), len(b), len(r))

l = data["l"]
d = data["d"]


x_int = data["x_init"]
y_int = data["y_init"]
th_int = data["th_init"]

# print(x_int, y_int, th_int)
# print(d)


