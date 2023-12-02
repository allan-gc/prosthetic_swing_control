import argparse
import os
import time
import math
import pandas as pd
import matplotlib.pyplot as plt

def load_dataset(data_file_name, data_dir):
    """
    This function loads the synthesized data provided in a csv file in the
    /data_dir directory.
    """

    data_path = os.path.join(data_dir, data_file_name)
    data_rows = open(data_path).read().strip().split("\n")

    return data_rows

data_dir = ''
data_file_name = "05ms.csv"

data_rows = load_dataset(data_file_name, data_dir)

single_pend_elbow_position = []
single_pend_elbow_torque = []
double_pend_elbow_position = []
double_pend_elbow_torque = []

for row in data_rows:
    position, torque = row.strip().split(",")

    single_pend_elbow_position.append(float(position[1:-1]))
    single_pend_elbow_torque.append(float(torque[1:-1]))
    # print(torque[1:-1])


elbow_pos = single_pend_elbow_position[:2000]
elbow_torque = single_pend_elbow_torque[:1000]
print(len(single_pend_elbow_torque))
print(len(elbow_torque))


plot_rate = 1/100
t_plot = []
input_torque_plot =[]
for i in range(len(elbow_torque)):
    t = i * plot_rate
    t_plot.append(t)
    input_torque_plot.append(elbow_torque[i])
    print(i)
    # time.sleep(plot_rate)


plt.plot(t_plot, input_torque_plot, c="blue", label="Input torque")
# plt.plot(t_plot, output_torque_plot, c="red", label="Output torque")
plt.ylabel('Torque [Nm]', fontsize=50)
plt.xlabel('Time [sec]', fontsize=50)
# plt.grid()
plt.title('Input & Output torque', fontsize=20)
plt.legend(fontsize=50)
plt.show()