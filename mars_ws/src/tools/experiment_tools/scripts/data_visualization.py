#!/usr/bin/env python3

import json
import matplotlib.pyplot as plt
import argparse
import numpy as np

########################################
# Simulation1 result : 14
# Simulation1 result : 15
########################################
def smooth_data(data, window_size):
    """Apply moving average smoothing to the data."""
    if len(data) < window_size:
        raise ValueError("Data length is shorter than the window size")
    return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

def plot_data(ax, data, key, ylabel, smooth=False, window_size=5):
    if key not in data:
        print(f"Warning: Key '{key}' not found in data")
        return
    
    raw_data = data[key]
    if raw_data is None:
        print(f"Warning: Data for key '{key}' is None")
        return
    
    try:
        raw_data = [float(value) for value in raw_data if value is not None]
    except ValueError:
        print(f"Warning: Data for key '{key}' contains non-numeric values")
        return

    if len(raw_data) == 0:
        print(f"Warning: Data for key '{key}' is empty or contains only None values")
        return

    if smooth:
        smoothed_data = smooth_data(raw_data, window_size)
        time_data = data['time'][:len(smoothed_data)]
        plot_data_values = smoothed_data
    else:
        time_data = data['time']
        plot_data_values = raw_data

    ax.plot(time_data[:len(plot_data_values)], plot_data_values, label=f"{'Smoothed ' if smooth else ''}{key}")
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(ylabel)
    ax.grid(True)

def main():
    parser = argparse.ArgumentParser(description='Plot navigation data.')
    parser.add_argument('file_number', type=int, help='The number of the navigation data file to open')
    args = parser.parse_args()

    file = f'/home/developer/lab/socially-store-robot/mars_ws/src/tools/experiment_tools/files/data/simulation1_data_{args.file_number}.json'
    
    with open(file, 'r') as f:
        data = json.load(f)
    
    fig, axs = plt.subplots(5, 2, figsize=(15, 20))
    fig.tight_layout(pad=5.0)
    
    # Plot data without smoothing
    smooth_window_size = 3
    plot_data(axs[0, 0], data, "navigability", "Navigability")
    plot_data(axs[1, 0], data, "speed_up_level", "Speed Up Level")
    plot_data(axs[2, 0], data, "robot_invisiable_level", "Robot Invisiable Level")
    plot_data(axs[3, 0], data, "right_side_level", "Right Side Level")
    plot_data(axs[4, 0], data, "pspace_level", "Pspace Level")
    plot_data(axs[0, 1], data, "weight_optimaltime", "Weight Optimal Time", smooth=True, window_size=smooth_window_size)
    plot_data(axs[1, 1], data, "weight_cc", "Weight CC", smooth=True, window_size=smooth_window_size)
    plot_data(axs[2, 1], data, "pspace_cov", "Pspace Overall Variance", smooth=True, window_size=smooth_window_size)
    plot_data(axs[3, 1], data, "pspace_r_ratio", "Pspace Right Ratio", smooth=True, window_size=smooth_window_size)
    plot_data(axs[4, 1], data, "use_external_prediction", "Use External Prediction")

    plt.show()

if __name__ == '__main__':
    main()
