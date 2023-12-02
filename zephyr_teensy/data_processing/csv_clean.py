
import csv
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from numpy.fft import fft, ifft
from scipy.fftpack import fftfreq


def truncate_csv(input_file, output_file, decimal_places=5):
    """
    Truncate decimal values in a CSV file to a specified number of decimal places.
    """
    with open(input_file, 'r') as infile, open(output_file, 'w', newline='') as outfile:
        reader = csv.reader(infile)
        writer = csv.writer(outfile)
        for row in reader:
            # Assuming your columns are in the first and second positions, adjust if needed
            truncated_col1 = format(float(row[0]), f".{decimal_places}f")
            # truncated_col2 = format(float(row[1]), f".{decimal_places}f")
            writer.writerow([truncated_col1])

    
    clean_csv(output_file)

def clean_csv(csv_file):
    with open(csv_file, 'r') as infile:
        content = infile.read().splitlines()

    # Add a comma after each line, except for the last line
    content_with_commas = '\n'.join([line + ','for line in content[:-1]] + [content[-1]])

    # Write the modified content to the output file
    with open(csv_file, 'w') as outfile:
        outfile.write(content_with_commas)


def plot_data(data):
    t_plot = []
    plot_rate  =1/100
    for i in range((len(data))):
        t = i * plot_rate
        t_plot.append(t)
    # Plot the data (replace 'column_name' with the actual column name you want to plot)
    plt.plot(t_plot, data)
    plt.title('Your Plot Title')
    plt.xlabel('X-axis Label')
    plt.ylabel('Y-axis Label')
    plt.show()


def fft_filter(signal, low_cutoff, high_cutoff):

    # sampling rate
    sr = 1/100
    # sampling interval
    t = np.arange(0,39.98,sr)

    x = signal

    # FFT the signal
    sig_fft = fft(x)
    # copy the FFT results
    sig_fft_filtered = sig_fft.copy()

    # obtain the frequencies using scipy function
    freq = fftfreq(len(x), d=1./100)

    
    cut_off_low =  low_cutoff
    cut_off_high = high_cutoff


    # sig_fft_filtered[np.abs(freq) > cut_off] = 0
    sig_fft_filtered[np.abs(freq) > cut_off_high] = 0
    sig_fft_filtered[np.abs(freq) < cut_off_low] = 0

    # sig_fft_filtered[np.abs(freq) > cut_off] = 0

    # get the filtered signal in time domain
    filtered = ifft(sig_fft_filtered) 

    # centered_data = filtered.real - np.mean(filtered.real)

    # plt.figure(figsize = (12, 6))
    # plt.subplot(221)
    # plt.stem(freq, np.abs(sig_fft), 'b', \
    #         markerfmt=" ", basefmt="-b")
    # plt.title('Before filtering')
    # plt.xlim(0, 50)
    # plt.xlabel('Frequency (Hz)')
    # plt.ylabel('FFT Amplitude')
    # plt.subplot(222)
    # plt.stem(freq, np.abs(sig_fft_filtered), 'b', \
    #         markerfmt=" ", basefmt="-b")
    # plt.title('After filtering')
    # plt.xlim(0, 50)
    # plt.xlabel('Frequency (Hz)')
    # plt.ylabel('FFT Amplitude')

    # ### Plot before and after filter
    # # plt.figure(figsize = (12, 6))
    # plt.subplot(223)
    # plt.plot(x, 'r')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Torque (Nm)')
    # plt.title('Torque Profile, No filter')
    # plt.subplot(224)
    # plt.plot(filtered)
    # plt.title('Torque Profile, 1.0 Hz Low Pass')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Torque (Nm)')
    # # plt.tight_layout()
    # # plt.show()
    # plt.show()

    plt.figure(figsize = (12, 6))
    plt.subplot(121)
    plt.plot(x, 'r')
    plt.xlabel('Time (s)')
    plt.ylabel('Torque (Nm)')
    plt.title('Torque Profile, No filter')
    plt.subplot(122)
    plt.plot(filtered)
    plt.title('Torque Profile, 1.0 Hz Low Pass')
    plt.xlabel('Time (s)')
    plt.ylabel('Torque (Nm)')
    # plt.tight_layout()
    # plt.show()
    plt.show()

    return filtered


def center_data(csv_file, output):
    with open(csv_file, 'r') as infile:
        content = infile.read().splitlines()

    # Convert the content to a NumPy array
    data_array = np.array([float(value) for value in content])

    # Center the values around zero
    centered_data = data_array - np.mean(data_array)

    # Write the centered data to the output file
    np.savetxt(output, centered_data, delimiter=',', fmt='%0.5f')

def calc_accel(pos_data):
    dt = 1/100
    t = np.arange(0,39.98,dt)
    
    velocity_data = np.diff(pos_data) / dt

    # Calculate acceleration (second derivative of position)
    acceleration_data = np.diff(velocity_data) / dt
    filtered_accel = fft_filter(acceleration_data, 0.0, 0.8)

    return filtered_accel



def main():

    base_csv = "05ms.csv"
    filtered_csv = "05ms_accel_filtered.csv"
    final_csv = "05ms_accel_clean.csv"
    # truncate_csv(input_csv, output_csv)

    df = pd.read_csv(base_csv, header=None)
    df.columns = ['Pos', 'Torque']

    # Torque Cut offs 0-1Hz, Pos cutoffs 0-1.0 Hz
    
    filtered_torque = fft_filter(df['Torque'], 0.0, 1.0)
    filtered_pos = fft_filter(df['Pos'], 0.0, 1.0)
    filtered_acceleration = calc_accel(filtered_pos)
    new_df = pd.DataFrame(filtered_acceleration.real)
    new_df.to_csv(filtered_csv, index=False, header=None)
    truncate_csv(filtered_csv, final_csv)

main()



