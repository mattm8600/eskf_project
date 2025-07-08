import numpy as np
import pandas as pd
from IPython.display import display
import matplotlib.pyplot as plt

csv_file_path = "new_troubleshooting.csv"
df = pd.read_csv(csv_file_path)
input_df = pd.read_csv("all_sensors.csv")
mean_input_gyr = input_df[["gyrx","gyry","gyrz"]].mean()
mean_input_gyr = mean_input_gyr * (np.pi/180)
mean_input_acc = input_df[["ax","ay","az"]].mean()
display(mean_input_acc)
display(mean_input_gyr)
mean_acceleration = df[["ax_hat", "ay_hat", "az_hat"]].mean()
df["yaw_unwrapped"] = np.unwrap(df["yaw"].values)
df["pitch_unwrapped"] = np.unwrap(df["pitch"].values)
df["roll_unwrapped"] = np.unwrap(df["roll"].values)



# time,x_hat,y_hat,z_hat,Vx_hat,Vy_hat,Vz_hat,ax_hat,ay_hat,az_hat
fig, axes = plt.subplots(4, 1, figsize=(30, 20))
axes[0].plot(df["time"], df["x_hat"], label="x_hat")
axes[0].plot(df["time"], df["y_hat"], label="y_hat")
axes[0].plot(df["time"], df["z_hat"], label="z_hat")

axes[1].plot(df["time"], df["Vx_hat"], label="Vx_hat")
axes[1].plot(df["time"], df["Vy_hat"], label="Vy_hat")
axes[1].plot(df["time"], df["Vz_hat"], label="Vz_hat")

axes[2].plot(df["time"], df["ax_hat"], label="ax_hat")
axes[2].plot(df["time"], df["ay_hat"], label="ay_hat")
axes[2].plot(df["time"], df["az_hat"], label="az_hat")

axes[3].plot(df["time"], df["yaw"], label="yaw")

axes[0].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[1].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[2].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[3].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[0].set_title("Position vs Time (s)")
axes[1].set_title("Velocity vs Time (s)")
axes[2].set_title("Acceleration vs Time (s)")
axes[3].set_title("Yaw vs Time (s)")


axes[0].legend(fontsize=12)
axes[1].legend(fontsize=12)
axes[2].legend(fontsize=12)
axes[3].legend(fontsize=12)
plt.show()

# Compare acceleration bias and acceleration:
# fig, axes = plt.subplots(2, 1, figsize=(24, 20))
# axes[0].plot(df["time"], df["ax_bias"], label="ax_bias")
# axes[0].plot(df["time"], df["ay_bias"], label="ay_bias")
# axes[0].plot(df["time"], df["az_bias"], label="az_bias")
# axes[1].plot(df["time"], df["ax_hat"], label="ax_hat")
# axes[1].plot(df["time"], df["ay_hat"], label="ay_hat")
# axes[1].plot(df["time"], df["az_hat"], label="az_hat")
# axes[0].grid(True, which='both', linestyle='--', linewidth=0.5)
# axes[1].grid(True, which='both', linestyle='--', linewidth=0.5)
# axes[0].set_title("Acceleration Bias vs Time (s)")
# axes[1].set_title("Predicted Acceleration vs Time (s)")
# axes[0].legend(fontsize=12)
# axes[1].legend(fontsize=12)
# plt.show()

# Compare gyro bias and acceleration:
fig, axes = plt.subplots(2, 1, figsize=(24, 20))
axes[0].plot(df["time"], df["gx_bias"], label="gx_bias")
axes[0].plot(df["time"], df["gy_bias"], label="gy_bias")
axes[0].plot(df["time"], df["gz_bias"], label="gz_bias")
axes[1].plot(df["time"], df["gx_bias"], label="gx_bias")
axes[1].plot(df["time"], df["gy_bias"], label="gy_bias")
axes[1].plot(df["time"], df["gz_bias"], label="gz_bias")
axes[0].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[1].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[0].set_title("Acceleration Bias vs Time (s)")
axes[1].set_title("Predicted Acceleration vs Time (s)")
axes[0].legend(fontsize=12)
axes[1].legend(fontsize=12)
plt.show()


# Compare Yaw in vs Yaw out
# fig, axes = plt.subplots(2, 1, figsize=(24, 20))
# axes[0].plot(df["time"], df["yaw"], label="yaw_hat")
# axes[1].plot(df["time"], df["yaw_residual"], label="yaw_residual")
# axes[0].grid(True, which='both', linestyle='--', linewidth=0.5)
# axes[1].grid(True, which='both', linestyle='--', linewidth=0.5)
# axes[0].legend(fontsize=12)
# axes[1].legend(fontsize=12)
# plt.show()

# Roll, Pitch, Yaw
fig, axes = plt.subplots(3, 1, figsize=(24, 20))
axes[0].plot(df["time"], df["roll"], label="roll")
axes[1].plot(df["time"], df["pitch"], label="pitch")
axes[2].plot(df["time"], df["yaw"], label="yaw")

axes[0].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[1].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[2].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[0].set_title("Roll")
axes[1].set_title("Pitch")
axes[2].set_title("Yaw")


axes[0].legend(fontsize=12)
axes[1].legend(fontsize=12)
axes[2].legend(fontsize=12)
plt.show()