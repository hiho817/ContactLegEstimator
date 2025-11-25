import numpy as np
import pandas as pd
import os
import sys
# Add parent directory to path
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, parent_dir)
from tools import *

FILENAME = "cpp_test"

observer_path = "output_data/" + FILENAME + "_observer_detail.csv"
raw_data_path = "data/" + FILENAME + ".csv"

check_file_exists(observer_path)
check_file_exists(raw_data_path)

observer_data = pd.read_csv(observer_path)
raw_data = pd.read_csv(raw_data_path)

start_index = 500
end_index = len(observer_data)  # Use full length of observer data

index = observer_data['Index'].values[start_index:end_index]
estimated_disturbance_beta_a = observer_data['estimated_disturbance_beta_a'].values[start_index:end_index]
estimated_disturbance_rm_a = observer_data['estimated_disturbance_rm_a'].values[start_index:end_index]
estimated_disturbance_beta_b = observer_data['estimated_disturbance_beta_b'].values[start_index:end_index]
estimated_disturbance_rm_b = observer_data['estimated_disturbance_rm_b'].values[start_index:end_index]
estimated_disturbance_beta_c = observer_data['estimated_disturbance_beta_c'].values[start_index:end_index]
estimated_disturbance_rm_c = observer_data['estimated_disturbance_rm_c'].values[start_index:end_index]
estimated_disturbance_beta_d = observer_data['estimated_disturbance_beta_d'].values[start_index:end_index]
estimated_disturbance_rm_d = observer_data['estimated_disturbance_rm_d'].values[start_index:end_index]

mapped_start_index = index[0]
end_index = len(raw_data)

# Check if the mapped indices equal the length of the observer data
if (end_index - mapped_start_index) != len(index):
    print("Error: Mismatch in data lengths between observer data and raw data.")
    exit()

print(f"Data length check passed: {end_index - mapped_start_index} == {len(index)}")

theta_a = raw_data['state_theta_a'].values[mapped_start_index:end_index]
beta_a = raw_data['state_beta_a'].values[mapped_start_index:end_index]
theta_b = raw_data['state_theta_b'].values[mapped_start_index:end_index]
beta_b = -raw_data['state_beta_b'].values[mapped_start_index:end_index]
theta_c = raw_data['state_theta_c'].values[mapped_start_index:end_index]
beta_c = -raw_data['state_beta_c'].values[mapped_start_index:end_index]
theta_d = raw_data['state_theta_d'].values[mapped_start_index:end_index]
beta_d = raw_data['state_beta_d'].values[mapped_start_index:end_index]

troque_R_a = raw_data['state_trq_r_a'].values[mapped_start_index:end_index]
troque_L_a = raw_data['state_trq_l_a'].values[mapped_start_index:end_index]
troque_R_b = raw_data['state_trq_r_b'].values[mapped_start_index:end_index]
troque_L_b = raw_data['state_trq_l_b'].values[mapped_start_index:end_index]
troque_R_c = raw_data['state_trq_r_c'].values[mapped_start_index:end_index]
troque_L_c = raw_data['state_trq_l_c'].values[mapped_start_index:end_index]
troque_R_d = raw_data['state_trq_r_d'].values[mapped_start_index:end_index]
troque_L_d = raw_data['state_trq_l_d'].values[mapped_start_index:end_index]

raw_rm_a, raw_beta_a = calculate_force_Motor2RP(theta_a, troque_L=troque_L_a, troque_R=troque_R_a)
raw_rm_b, raw_beta_b = calculate_force_Motor2RP(theta_b, troque_L=troque_L_b, troque_R=troque_R_b)
raw_rm_c, raw_beta_c = calculate_force_Motor2RP(theta_c, troque_L=troque_L_c, troque_R=troque_R_c)
raw_rm_d, raw_beta_d = calculate_force_Motor2RP(theta_d, troque_L=troque_L_d, troque_R=troque_R_d)

raw_beta_b = -raw_beta_b
raw_beta_c = -raw_beta_c



############### Plotting ###############
import matplotlib.pyplot as plt

# Compare estimated disturbance with raw values
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 10), sharex=True)

# Leg LF
ax1.plot(index, estimated_disturbance_rm_a, 'b-', label='Estimated Disturbance Rm LF', linewidth=1.5)
ax1.plot(index, -raw_rm_a, 'b--', label='Raw Rm LF', linewidth=1)
ax1.plot(index, estimated_disturbance_beta_a, 'g-', label='Estimated Disturbance Beta LF', linewidth=1.5)
ax1.plot(index, -raw_beta_a, 'g--', label='Raw Beta LF', linewidth=1)
ax1.set_title('Leg LF')
ax1.set_ylabel('Rm')
ax1.legend()
ax1.grid()

# Leg RF
ax2.plot(index, estimated_disturbance_rm_b, 'b-', label='Estimated Disturbance Rm RF', linewidth=1.5)
ax2.plot(index, -raw_rm_b, 'b--', label='Raw Rm RF', linewidth=1)
ax2.plot(index, estimated_disturbance_beta_b, 'g-', label='Estimated Disturbance Beta RF', linewidth=1.5)
ax2.plot(index, -raw_beta_b, 'g--', label='Raw Beta RF', linewidth=1)
ax2.set_title('Leg RF')
ax2.set_ylabel('Rm')
ax2.legend()
ax2.grid()

# Leg RH
ax3.plot(index, estimated_disturbance_rm_c, 'b-', label='Estimated Disturbance Rm RH', linewidth=1.5)
ax3.plot(index, -raw_rm_c, 'b--', label='Raw Rm RH', linewidth=1)
ax3.plot(index, estimated_disturbance_beta_c, 'g-', label='Estimated Disturbance Beta RH', linewidth=1.5)
ax3.plot(index, -raw_beta_c, 'g--', label='Raw Beta RH', linewidth=1)
ax3.set_title('Leg RH')
ax3.set_ylabel('Rm')
ax3.legend()
ax3.grid()

# Leg LH
ax4.plot(index, estimated_disturbance_rm_d, 'b-', label='Estimated Disturbance Rm LH', linewidth=1.5)
ax4.plot(index, -raw_rm_d, 'b--', label='Raw Rm LH', linewidth=1)
ax4.plot(index, estimated_disturbance_beta_d, 'g-', label='Estimated Disturbance Beta LH', linewidth=1.5)
ax4.plot(index, -raw_beta_d, 'g--', label='Raw Beta LH', linewidth=1)
ax4.set_title('Leg LH')
ax4.set_xlabel('Index')
ax4.set_ylabel('Rm')
ax4.legend()
ax4.grid()

plt.tight_layout()
plt.show()
