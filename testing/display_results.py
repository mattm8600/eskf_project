import numpy as np
import pandas as pd
from IPython.display import display
import matplotlib.pyplot as plt

csv_file_path = "square_flight_results.csv"
df = pd.read_csv(csv_file_path)
input_df = pd.read_csv("squareflightdataset.csv")
input_df = input_df.replace(-9999999.00, np.nan)
input_df['time'] = input_df['time']/(1e6)
input_df[["gyrx","gyry","gyrz"]] = input_df[["gyrx","gyry","gyrz"]]*(1/16.4)*(np.pi/180)
mean_input_acc = input_df[["ax","ay","az"]].mean()
display(mean_input_acc)
mean_acceleration = df[["ax_hat", "ay_hat", "az_hat"]].mean()
df['est_heading'] = df['est_heading'] * (180/np.pi)
df['yaw'] = df['yaw'] * (180/np.pi)
input_df['mag_norm'] = np.sqrt(input_df['mag_x']*input_df['mag_x'] + input_df['mag_y']*input_df['mag_y'] + input_df['mag_z']*input_df['mag_z'])
# pd.set_option('display.max_columns', None)
# row = df[df["time"] == 186686]

# print(row)

# time,x_hat,y_hat,z_hat,Vx_hat,Vy_hat,Vz_hat,ax_hat,ay_hat,az_hat
fig, axes = plt.subplots(3, 1, figsize=(30, 20))
axes[0].plot(df["time"], df["x_hat"], label="x_hat")
axes[0].plot(df["time"], df["y_hat"], label="y_hat")
axes[0].plot(df["time"], df["z_hat"], label="z_hat")

axes[1].plot(df["time"], df["Vx_hat"], label="Vx_hat")
axes[1].plot(df["time"], df["Vy_hat"], label="Vy_hat")
axes[1].plot(df["time"], df["Vz_hat"], label="Vz_hat")

axes[2].plot(df["time"], df["ax_hat"], label="ax_hat")
axes[2].plot(df["time"], df["ay_hat"], label="ay_hat")
axes[2].plot(df["time"], df["az_hat"], label="az_hat")

axes[0].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[1].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[2].grid(True, which='both', linestyle='--', linewidth=0.5)

axes[0].set_title("Position vs Time (s)")
axes[1].set_title("Velocity vs Time (s)")
axes[2].set_title("Acceleration vs Time (s)")


axes[0].legend(fontsize=12)
axes[1].legend(fontsize=12)
axes[2].legend(fontsize=12)
plt.show(block=False)
# Compare acceleration bias and acceleration:
fig, axes = plt.subplots(2, 1, figsize=(24, 20))
axes[0].plot(df["time"], df["ax_bias"], label="ax_bias")
axes[0].plot(df["time"], df["ay_bias"], label="ay_bias")
axes[0].plot(df["time"], df["az_bias"], label="az_bias")
axes[1].plot(input_df["time"], input_df["ax"], label="ax", marker='o',markersize=2,linestyle='None')
axes[1].plot(input_df["time"], input_df["ay"], label="ay", marker='o',markersize=2,linestyle='None')
axes[1].plot(input_df["time"], input_df["az"], label="az", marker='o',markersize=2,linestyle='None')
axes[0].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[1].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[0].set_title("Acceleration Bias vs Time (s)")
axes[1].set_title("Predicted Acceleration vs Time (s)")
axes[0].legend(fontsize=12)
axes[1].legend(fontsize=12)
plt.show(block=False)

# Compare gyro bias and gyro:
fig, axes = plt.subplots(3, 1, figsize=(24, 20))
axes[0].plot(df["time"], df["gx_bias"], label="gx_bias")
axes[0].plot(df["time"], df["gy_bias"], label="gy_bias")
axes[0].plot(df["time"], df["gz_bias"], label="gz_bias")
axes[1].plot(input_df["time"], input_df["gyrx"], label="gyr_x", marker='o',markersize=2,linestyle='None')
axes[1].plot(input_df["time"], input_df["gyry"], label="gyr_y", marker='o',markersize=2,linestyle='None')
axes[1].plot(input_df["time"], input_df["gyrz"], label="gyr_z", marker='o',markersize=2,linestyle='None')
# axes[1].plot(input_df["time"], df["filt_w_x"], label="filtered w_x")
# axes[1].plot(input_df["time"], df["filt_w_y"], label="filtered w_y")
# axes[1].plot(input_df["time"], df["filt_w_z"], label="filtered w_z")
axes[0].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[1].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[0].set_title("Gyro Bias vs Time (s)")
axes[1].set_title("Input gyro vs Time (s)")
axes[0].legend(fontsize=12)
axes[1].legend(fontsize=12)
plt.show(block=False)

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
axes[0].plot(df["time"], df["roll"], label="roll", marker='o',markersize=3)
axes[1].plot(df["time"], df["pitch"], label="pitch", marker='o',markersize=3)
axes[2].plot(df["time"], df["yaw"], label="yaw", marker='o',markersize=3)

axes[0].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[1].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[2].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[0].set_title("Roll")
axes[1].set_title("Pitch")
axes[2].set_title("Yaw")


axes[0].legend(fontsize=12)
axes[1].legend(fontsize=12)
axes[2].legend(fontsize=12)

in_region = False
start_time = None
time = df["time"]
mag_used = df["mag_correction"]

for i in range(1, len(time)):
    if mag_used.iloc[i] == 1 and not in_region:
        in_region = True
        start_time = time.iloc[i]
    elif mag_used.iloc[i] == 0 and in_region:
        in_region = False
        end_time = time.iloc[i]
        axes[2].axvspan(start_time, end_time, color='lightgreen', alpha=0.3)

# If still in a region at the end of the data
if in_region:
    axes[2].axvspan(start_time, time.iloc[-1], color='lightgreen', alpha=0.3)


in_region = False
start_time = None
time = df["time"]
pr_used = df["pr_correction"]

for i in range(1, len(time)):
    if pr_used.iloc[i] == 1 and not in_region:
        in_region = True
        start_time = time.iloc[i]
    elif pr_used.iloc[i] == 0 and in_region:
        in_region = False
        end_time = time.iloc[i]
        axes[0].axvspan(start_time, end_time, color='lightgreen', alpha=0.3)
        axes[1].axvspan(start_time, end_time, color='lightgreen', alpha=0.3)

# If still in a region at the end of the data
if in_region:
    axes[0].axvspan(start_time, time.iloc[-1], color='lightgreen', alpha=0.3)
    axes[1].axvspan(start_time, time.iloc[-1], color='lightgreen', alpha=0.3)

plt.show(block=False)


# INPUT:
fig, axes = plt.subplots(4, 1, figsize=(30, 20))
axes[0].plot(input_df["time"], input_df["gps_x"], '.', label="GPS x")
axes[0].plot(input_df["time"], input_df["gps_y"], '.', label="GPS y")
axes[0].plot(input_df["time"], input_df["gps_z"], '.', label="GPS z")
axes[0].plot(input_df["time"], input_df["baro"], label="Baro z")

axes[1].plot(input_df["time"], input_df["gyrx"], label="gyr_x")
axes[1].plot(input_df["time"], input_df["gyry"], label="gyr_y")
axes[1].plot(input_df["time"], input_df["gyrz"], label="gyr_z")

axes[2].plot(input_df["time"], input_df["ax"], label="ax")
axes[2].plot(input_df["time"], input_df["ay"], label="ay")
axes[2].plot(input_df["time"], input_df["az"], label="az")

axes[3].plot(input_df["time"], input_df["heading"], label="Mag heading")

axes[0].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[1].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[2].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[3].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[0].set_title("Position vs Time (s)")
axes[1].set_title("Angular Velocity vs Time (s)")
axes[2].set_title("Acceleration vs Time (s)")
axes[3].set_title("Heading vs Time (s)")


axes[0].legend(fontsize=12)
axes[1].legend(fontsize=12)
axes[2].legend(fontsize=12)
axes[3].legend(fontsize=12)
plt.show()

# Estimated Heading vs Model's Heading vs Mag Heading
# fig, axes = plt.subplots(4, 1, figsize=(30, 20))
# axes[0].plot(df["time"], df["yaw"], '.', label="Model Heading")
# axes[1].plot(df["time"], df["est_heading"], label="Estimated Mag heading")
# axes[2].plot(input_df["time"], input_df["heading"], label="mag heading")
# axes[3].plot(input_df["time"], input_df["mag_x"], label="Mag x")
# axes[3].plot(input_df["time"], input_df["mag_y"], label="Mag y")
# axes[3].plot(input_df["time"], input_df["mag_z"], label="Mag z")
# axes[3].plot(input_df["time"], input_df["mag_norm"], label="Mag Norm")
# axes[0].grid(True, which='both', linestyle='--', linewidth=0.5)
# axes[1].grid(True, which='both', linestyle='--', linewidth=0.5)
# axes[2].grid(True, which='both', linestyle='--', linewidth=0.5)
# axes[3].grid(True, which='both', linestyle='--', linewidth=0.5)
# axes[1].set_title("Estimated Heading vs Time (s)")
# axes[0].set_title("Model Heading vs Time (s)")
# axes[2].set_title("Mag Heading vs Time (s)")
# axes[3].set_title("Mag vector vs Time (s)")
# axes[0].legend(fontsize=12)
# axes[1].legend(fontsize=12)
# axes[2].legend(fontsize=12)
# axes[3].legend(fontsize=12)

# in_region = False
# start_time = None
# time = df["time"]
# mag_used = df["mag_correction"]

# for i in range(1, len(time)):
#     if mag_used.iloc[i] == 1 and not in_region:
#         in_region = True
#         start_time = time.iloc[i]
#     elif mag_used.iloc[i] == 0 and in_region:
#         in_region = False
#         end_time = time.iloc[i]
#         axes[0].axvspan(start_time, end_time, color='lightgreen', alpha=0.3)

# # If still in a region at the end of the data
# if in_region:
#     axes[0].axvspan(start_time, time.iloc[-1], color='lightgreen', alpha=0.3)

# plt.show()

# # Mag x,y,z and heading:
# plt.plot(input_df['time'], input_df[['mag_x','mag_y','mag_z']])
# plt.legend()
# plt.grid(True)
# plt.show(block=False)
plt.pause(999999)