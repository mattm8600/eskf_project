import numpy as np
import pandas as pd
from IPython.display import display
import matplotlib.pyplot as plt

csv_file_path = "tri_flight_2_results.csv"
df = pd.read_csv(csv_file_path)
input_df = pd.read_csv("tri_flight_2.csv")
input_df = input_df.replace(-9999999.00, np.nan)
input_df['time'] = input_df['time']/(1e6)
mean_input_acc = input_df[["ax","ay","az"]].mean()
display(mean_input_acc)
mean_acceleration = df[["ax_hat", "ay_hat", "az_hat"]].mean()
mask = input_df['time'] < 90.0
accuracyS_mask = input_df[['gps_hAcc','gps_vAcc','gps_sAcc','gps_cAcc']] > 100
input_df.loc[:, ['gps_hAcc','gps_vAcc','gps_sAcc','gps_cAcc']] = input_df.loc[:, ['gps_hAcc','gps_vAcc','gps_sAcc','gps_cAcc']].mask(accuracyS_mask, np.nan)

avg_bias = input_df.loc[mask, ["gyrx", "gyry", "gyrz"]].mean()
display(avg_bias)
input_df['mag_norm'] = np.sqrt(input_df['mag_x']*input_df['mag_x'] + input_df['mag_y']*input_df['mag_y'] + input_df['mag_z']*input_df['mag_z'])
# pd.set_option('display.max_columns', None)
# row = df[df["time"] == 186686]


plt.rcParams.update({
    "axes.titlesize": 22,
    "axes.labelsize": 14,
    "xtick.labelsize": 14,
    "ytick.labelsize": 14,
    "legend.fontsize": 18,
    "lines.linewidth": 2,
    "lines.markersize": 5
})



display(df["gps_yaw_correction"])

# time,x_hat,y_hat,z_hat,Vx_hat,Vy_hat,Vz_hat,ax_hat,ay_hat,az_hat
fig, axes = plt.subplots(3, 1, figsize=(24, 20), sharex=True, constrained_layout=True)
axes[0].plot(df["time"], df["x_hat"], '.', label="x_hat")
axes[0].plot(df["time"], df["y_hat"], '.', label="y_hat")
axes[0].plot(df["time"], df["z_hat"], '.', label="z_hat")

axes[1].plot(df["time"], df["Vx_hat"], '.', label="Vx_hat")
axes[1].plot(df["time"], df["Vy_hat"], '.', label="Vy_hat")
axes[1].plot(df["time"], df["Vz_hat"], '.', label="Vz_hat")

axes[2].plot(df["time"], df["ax_hat"], '.', label="ax_hat")
axes[2].plot(df["time"], df["ay_hat"], '.', label="ay_hat")
axes[2].plot(df["time"], df["az_hat"], '.', label="az_hat")


axes[0].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[1].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[2].grid(True, which='both', linestyle='--', linewidth=0.5)

axes[0].set_title("Position vs Time (s)")
axes[1].set_title("Velocity vs Time (s)")
axes[2].set_title("Acceleration vs Time (s)")


axes[0].legend(loc='upper left')
axes[1].legend(loc='upper left')
axes[2].legend(loc='upper left')
plt.show(block=False)


# Compare acceleration bias and acceleration:
# fig, axes = plt.subplots(2, 1, figsize=(24, 20))
# axes[0].plot(df["time"], df["ax_bias"], label="ax_bias")
# axes[0].plot(df["time"], df["ay_bias"], label="ay_bias")
# axes[0].plot(df["time"], df["az_bias"], label="az_bias")
# axes[1].plot(input_df["time"], input_df["ax"], label="ax", marker='o',markersize=2,linestyle='None')
# axes[1].plot(input_df["time"], input_df["ay"], label="ay", marker='o',markersize=2,linestyle='None')
# axes[1].plot(input_df["time"], input_df["az"], label="az", marker='o',markersize=2,linestyle='None')
# axes[0].grid(True, which='both', linestyle='--', linewidth=0.5)
# axes[1].grid(True, which='both', linestyle='--', linewidth=0.5)
# axes[0].set_title("Acceleration Bias vs Time (s)")
# axes[1].set_title("Predicted Acceleration vs Time (s)")
# axes[0].legend(fontsize=12)
# axes[1].legend(fontsize=12)
# plt.show(block=False)

# Compare gyro bias and gyro:
# fig, axes = plt.subplots(3, 1, figsize=(24, 20))
# axes[0].plot(df["time"], df["gx_bias"], label="gx_bias")
# axes[0].plot(df["time"], df["gy_bias"], label="gy_bias")
# axes[0].plot(df["time"], df["gz_bias"], label="gz_bias")
# axes[1].plot(input_df["time"], input_df["gyrx"], label="gyr_x", marker='o',markersize=2,linestyle='None')
# axes[1].plot(input_df["time"], input_df["gyry"], label="gyr_y", marker='o',markersize=2,linestyle='None')
# axes[1].plot(input_df["time"], input_df["gyrz"], label="gyr_z", marker='o',markersize=2,linestyle='None')
# # axes[1].plot(input_df["time"], df["filt_w_x"], label="filtered w_x")
# # axes[1].plot(input_df["time"], df["filt_w_y"], label="filtered w_y")
# # axes[1].plot(input_df["time"], df["filt_w_z"], label="filtered w_z")
# axes[0].grid(True, which='both', linestyle='--', linewidth=0.5)
# axes[1].grid(True, which='both', linestyle='--', linewidth=0.5)
# axes[0].set_title("Gyro Bias vs Time (s)")
# axes[1].set_title("Input gyro vs Time (s)")
# axes[0].legend(fontsize=12)
# axes[1].legend(fontsize=12)
# plt.show(block=False)

# Roll, Pitch, Yaw
fig, axes = plt.subplots(3, 1, figsize=(24, 20), sharex=True, constrained_layout=True)
axes[0].plot(df["time"], df["roll"], label="roll", marker='o',markersize=3)
axes[1].plot(df["time"], df["pitch"], label="pitch", marker='o',markersize=3)
axes[2].plot(df["time"], df["yaw"], label="yaw", marker='o',markersize=3)

axes[0].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[1].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[2].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[0].set_title("Roll")
axes[1].set_title("Pitch")
axes[2].set_title("Yaw")


axes[0].legend(loc='upper right')
axes[1].legend(loc='upper right')
axes[2].legend(loc='upper right')

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
        axes[2].axvspan(start_time, end_time, color='gold', alpha=0.3)

# If still in a region at the end of the data
if in_region:
    axes[2].axvspan(start_time, time.iloc[-1], color='gold', alpha=0.3)


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

in_region = False
start_time = None
time = df["time"]
gps = df["gps_yaw_correction"]

for i in range(1, len(time)):
    if gps.iloc[i] == 1 and not in_region:
        in_region = True
        start_time = time.iloc[i]
    elif gps.iloc[i] == 0 and in_region:
        in_region = False
        end_time = time.iloc[i]
        axes[2].axvspan(start_time, end_time, color='lightcoral', alpha=0.3)

# If still in a region at the end of the data
if in_region:
    axes[2].axvspan(start_time, time.iloc[-1], color='lightcoral', alpha=0.3)

plt.show(block=False)


# INPUT:
fig, axes = plt.subplots(2, 3, figsize=(24, 40), sharex=True, constrained_layout=True)

# Plot 1: GPS Position
axes[0][0].plot(input_df["time"], input_df["gps_x"], '.', label="GPS x")
axes[0][0].plot(input_df["time"], input_df["gps_y"], '.', label="GPS y")
axes[0][0].plot(input_df["time"], input_df["gps_z"], '.', label="GPS z")
axes[0][0].plot(input_df["time"], input_df["baro"], '.', label="Baro z")

# Plot 2: GPS Velocity
# axes[0][1].plot(input_df["time"], input_df["gps_veln"], '.', label="GPS Vel N")
axes[0][1].plot(input_df["time"], input_df["gps_vele"], '.', label="GPS Vel E")
axes[0][1].plot(input_df["time"], input_df["gps_veld"], '.', label="GPS Vel D")

# Plot 3: GPS Accuracy
axes[0][2].plot(input_df["time"], input_df["gps_hAcc"], '.', label="Gps Horiz. Acc.")
axes[0][2].plot(input_df["time"], input_df["gps_vAcc"], '.', label="Gps Vert. Acc.")
axes[0][2].plot(input_df["time"], input_df["gps_sAcc"], '.', label="Gps Speed Acc.")
axes[0][2].plot(input_df["time"], input_df["gps_cAcc"], '.', label="Gps Heading Acc.")

# Plot 4: Accelerometer
axes[1][0].plot(input_df["time"], input_df["ax"], '.', label="Ax")
axes[1][0].plot(input_df["time"], input_df["ay"], '.', label="Ay")
axes[1][0].plot(input_df["time"], input_df["az"], '.', label="Az")

# Plot 5: Gyro
axes[1][1].plot(input_df["time"], input_df["gyrx"], '.', label="Gyr x")
axes[1][1].plot(input_df["time"], input_df["gyry"], '.', label="Gyr y")
axes[1][1].plot(input_df["time"], input_df["gyrz"], '.', label="Gyr z")

# Plot 6: Heading
axes[1][2].plot(input_df["time"], input_df["heading"], '.', label="Mag heading")
axes[1][2].plot(input_df["time"], input_df["gps_heading"], '.', label="Gps heading")


axes[0][0].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[0][1].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[0][2].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[1][0].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[1][1].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[1][2].grid(True, which='both', linestyle='--', linewidth=0.5)
axes[0][0].set_title("Position vs Time (s)")
axes[0][1].set_title("GPS Velocity vs Time (s)")
axes[0][2].set_title("GPS Accuracy vs Time (s)")
axes[1][0].set_title("Body acceleration vs Time (s)")
axes[1][1].set_title("Angular Velocity vs Time (s)")
axes[1][2].set_title("Heading vs Time (s)")


axes[0][0].legend(loc='lower left')
axes[0][1].legend(loc='lower left')
axes[0][2].legend(loc='lower left')
axes[1][0].legend(loc='lower left')
axes[1][1].legend(loc='lower left')
axes[1][2].legend(loc='lower left')
plt.show()


plt.pause(999999)