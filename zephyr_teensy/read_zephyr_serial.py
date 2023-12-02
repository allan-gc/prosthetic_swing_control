# chapter 28 in python

# sudo apt-get install python3-pip
# python3 -m pip install pyserial
# sudo apt-get install python3-matplotlib
# import ch28_read_plot_matrix as plot

import serial
import pandas as pd
import matplotlib.pyplot as plt
from numpy.fft import fft, ifft
from scipy.fftpack import fftfreq
import numpy as np

torque_samples = 3998

def clean_csv(input_filename, output_filename, desired_decimal_places):
    # Read the CSV file into a DataFrame
    df = pd.read_csv(input_filename, header=None)

    # Convert all values to numeric

    # Round each numeric value to the desired number of decimal places
    df = df.round(decimals=desired_decimal_places)

    # Write the cleaned data to a new CSV file
    df.to_csv(output_filename, index=False, header=False)


def fft_filter(torque_signal):
    # sampling rate
    sr = 1/1000
    # sampling interval
    t = np.arange(0,10,sr)

    x = torque_signal

    # FFT the signal
    sig_fft = fft(x)
    # copy the FFT results
    sig_fft_filtered = sig_fft.copy()

    # obtain the frequencies using scipy function
    freq = fftfreq(len(x), d=1./1000)

    # define the cut-off frequency
    cut_off_low =  0.0
    cut_off_high = 1.0

    # low-passfilter by assign zeros to the 
    # FFT amplitudes where the absolute 
    # frequencies higher than the cut-off 
    # sig_fft_filtered[np.abs(freq) > cut_off] = 0
    sig_fft_filtered[np.abs(freq) > cut_off_high] = 0
    sig_fft_filtered[np.abs(freq) < cut_off_low] = 0


    # get the filtered signal in time domain
    filtered = ifft(sig_fft_filtered)

    #### Showing Before after Filtered, both in time and freq domains
    # plt.figure(figsize = (12, 6))
    # plt.subplot(221)
    # plt.stem(freq, np.abs(sig_fft), 'b', \
    #         markerfmt=" ", basefmt="-b")
    # plt.title('Before filtering')
    # plt.xlim(0, 500)
    # plt.xlabel('Frequency (Hz)')
    # plt.ylabel('FFT Amplitude')
    # plt.subplot(222)
    # plt.stem(freq, np.abs(sig_fft_filtered), 'b', \
    #         markerfmt=" ", basefmt="-b")
    # plt.title('After filtering')
    # plt.xlim(0, 500)
    # plt.xlabel('Frequency (Hz)')
    # plt.ylabel('FFT Amplitude')


    # plt.subplot(223)
    # plt.plot(t,torque_signal,'b-')
    # plt.ylabel('Torque [A]')
    # plt.xlabel('Time [sec]')
    # plt.title("Output Torque")
    # plt.subplot(224)
    # plt.plot(t, filtered,'r-')
    # plt.ylabel('Torque [A]')
    # plt.xlabel('Time [sec]')
    # plt.title("Output Torque Filtered")

    # plt.tight_layout()
    # plt.show()

    ## Shows final plot of time domain
    # plt.figure(figsize = (12, 6))
    # plt.subplot(121)
    # plt.plot(t, x, 'r')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Torque (Nm)')
    # plt.title('Torque Profile, No filter')
    # plt.subplot(122)
    # plt.plot(t, filtered)
    # plt.title('Torque Profile, 1.0 Hz Low Pass')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Torque (Nm)')
    # # plt.tight_layout()
    # # plt.show()
    # plt.show()

    return filtered



def read_ref(serial):
        data = []
        ref = []
        t= [ ]
        sampling_rate = 1/1000
        data_received = 0
        while data_received < 10000:
            dat_str = serial.read_until(b'\n');  # get the data as a string, ints seperated by spaces
            dat_f = list(map(float,dat_str.split())) # now the data is a list
            ref.append(dat_f[0])
            data.append(dat_f[1])
    
            # data_float = float(dat_str)
            # data.append(data_float)
            t.append(data_received*sampling_rate)
            # if not dat_str:
            #     break
            print("Getting data\n", data_received)

            data_received+=1

        filtered_data=fft_filter(data)
        ### Plot before and after filter
        # plt.figure(figsize = (12, 6))
        # plt.subplot(121)
        # plt.plot(t, x, 'r')
        # plt.xlabel('Time (s)')
        # plt.ylabel('Torque (Nm)')
        # plt.title('Torque Profile, No filter')
        # plt.subplot(122)
        # plt.plot(t, filtered)
        # plt.title('Torque Profile, 1.5 Hz Low Pass')
        # plt.xlabel('Time (s)')
        # plt.ylabel('Torque (Nm)')
        # plt.tight_layout()
        # plt.show()

        # min_current= min(data)
        # max_current= max(data)
        # plt.text(0.8, 0.9, f'Min Current: {min_current}\nMax Current: {max_current}', transform=plt.gca().transAxes,
        #          bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.5'))

        # plt.ylim(-2.0, 2.0)
        # plt.plot(t,data,'b-')
        # plt.ylabel('Current [A]')
        # plt.xlabel('Time [sec]')
        # plt.title("Calculated Input Current")

        plt.plot(t,ref,'r-', t,filtered_data,'b-')
        plt.ylabel('Current [A]')
        plt.xlabel('Time [sec]')
        plt.title("Input vs Output Current")
        plt.legend(['Input Current', 'Output Current']) 
        plt.show()
    
def read_serial(serial):
        data = []
        t= [ ]
        sampling_rate = 1/1000
        data_received = 0
        while data_received < 5000:
            dat_str = serial.read_until(b'\n');  # get the data as a string, ints seperated by spaces
     
            data_float = float(dat_str)
            data.append(data_float)
            t.append(data_received*sampling_rate)
            # if not dat_str:
            #     break
            print("Getting data\n", data_received)

            data_received+=1

        fft_filter(data)
        ### Plot before and after filter
        # plt.figure(figsize = (12, 6))
        # plt.subplot(121)
        # plt.plot(t,data,'b-')
        # plt.ylabel('Current [A]')
        # plt.xlabel('Time [sec]')
        # plt.title("Output Current")
        # plt.subplot(122)
        # plt.plot(t, filtered_data,'r-')
        # plt.ylabel('Current [A]')
        # plt.xlabel('Time [sec]')
        # plt.title("Output Current Filtered")
        # plt.tight_layout()
        # plt.show()
        # min_current= min(data)
        # max_current= max(data)
        # plt.text(0.8, 0.9, f'Min Current: {min_current}\nMax Current: {max_current}', transform=plt.gca().transAxes,
        #          bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.5'))

        # plt.ylim(-2.0, 2.0)
        # plt.plot(t,data,'b-')
        # plt.ylabel('Current [A]')
        # plt.xlabel('Time [sec]')
        # plt.title("Calculated Input Current")
        # plt.plot(t,data,'b-')
        # plt.ylabel('Torque [Nm]')
        # plt.xlabel('Time [sec]')
        # plt.title("Input vs Output Torque")
        # plt.show()
     
def main():
    # input_filename = "05ms.csv"
    # output_filename = "05ms_clean.csv"
    # desired_decimal_places = 5
    # clean_csv(input_filename, output_filename, desired_decimal_places)

    ser = serial.Serial("/dev/ttyACM0", 230400, timeout=3.0)

    has_quit = False
    # menu loop
    while not has_quit:
        
        try: 
            read_ref(ser)
            # read_serial(ser)
         
            break
        except:
            print("error open serial port")
            exit()



main()