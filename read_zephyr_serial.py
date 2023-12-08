
import serial
import pandas as pd
import matplotlib.pyplot as plt
from numpy.fft import fft, ifft
from scipy.fftpack import fftfreq
import numpy as np
import math

torque_samples = 3998


def fft_filter(signal):
    # sampling rate
    sr = 1/1000
    # sampling interval
    t = np.arange(0,10.0,sr)

    x = signal

    # FFT the signal
    sig_fft = fft(x)
    # copy the FFT results
    sig_fft_filtered = sig_fft.copy()

    # obtain the frequencies using scipy function
    freq = fftfreq(len(x), d=1./1000)

    # define the cut-off frequency
    cut_off_low =  0.0
    cut_off_high = 1.0

    sig_fft_filtered[np.abs(freq) > cut_off_high] = 0
    sig_fft_filtered[np.abs(freq) < cut_off_low] = 0


    # get the filtered signal in time domain
    filtered = ifft(sig_fft_filtered)

    return filtered

def fft_filter_set(signal, low_cutoff, high_cutoff):

    # sampling rate
    sr = 1/1000
    scale = 1
    fft_threshold_scale= 99
    # sampling interval
    t = np.arange(0,10.00,sr)


    x = signal
    # FFT the signal
    sig_fft = fft(x)
    # copy the FFT results
    sig_fft_filtered = sig_fft.copy()

    # obtain the frequencies using scipy function
    freq = fftfreq(len(x), d=1./1000)

    sig_fft_filtered[np.abs(freq) > high_cutoff] = 0
    sig_fft_filtered[np.abs(freq) < low_cutoff] = 0

    ## For magnitude threshold filtering
    # percentile_threshold = np.percentile(sig_fft_filtered, fft_threshold_scale)
    # sig_fft_filtered = np.where(sig_fft_filtered >= percentile_threshold, sig_fft_filtered, 0)

    # get the filtered signal in time domain
    filtered = ifft(sig_fft_filtered) 

    # if type == 'Pos':
    #     plot_axes = 'Position (deg)'
    #     plot_title = 'Position'
    #     # x*=360
    #     # filtered*=360
    #     scale = 360
    # elif type == 'Accel':
    #     plot_axes = 'Acceleration (rad/s/s)'
    #     plot_title = 'Acceleration'
    #     # x*=(2*math.pi)
    #     # filtered*=(2*math.pi)
    #     scale = (2*math.pi)

    # else:
    #     plot_axes = 'Torque (Nm)'
    #     plot_title = 'Torque'

    # max_fft= freq[np.argmax(np.abs(sig_fft_filtered))]

    # if show_all:
    #     plt.figure(figsize = (12, 6))
    #     plt.subplot(221)
    #     plt.stem(freq, np.abs(sig_fft), 'b', \
    #             markerfmt=" ", basefmt="-b")
    #     plt.title('Before filtering')
    #     # plt.xlim(0, 50)
    #     plt.xlabel('Frequency (Hz)')
    #     plt.ylabel('FFT Amplitude')
    #     plt.subplot(222)
    #     plt.stem(freq, np.abs(sig_fft_filtered), 'b', \
    #             markerfmt=" ", basefmt="-b")
    #     plt.title('After filtering')
    #     # plt.xlim(0, 50)
    #     plt.xlabel('Frequency (Hz)')
    #     plt.ylabel('FFT Amplitude')
    #     plt.text(0.3, 0.9, f'Max Freq: {max_fft}', transform=plt.gca().transAxes,
    #              bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.5'))

    #     ### Plot before and after filter
    #     plt.subplot(223)
    #     plt.plot(x*scale, 'r')
    #     plt.xlabel('Time (s)')
    #     plt.ylabel(plot_axes)
    #     plt.title(plot_title + ' No filter')
    #     plt.subplot(224)
    #     plt.plot(filtered*scale)
    #     plt.title(plot_title + " "+ str(high_cutoff)+' Hz Low Pass')
    #     plt.xlabel('Time (s)')
    #     plt.ylabel(plot_axes)
    #     plt.show()

    # else:
    #     plt.figure(figsize = (12, 6))
    #     plt.subplot(121)
    #     plt.plot(x*scale, 'r')
    #     plt.xlabel('Time (s)')
    #     plt.ylabel(plot_axes)
    #     plt.title(plot_title + ' No filter')
    #     plt.subplot(122)
    #     plt.plot(filtered*scale)
    #     plt.title(plot_title + " "+ str(high_cutoff)+' Hz Low Pass')
    #     plt.xlabel('Time (s)')
    #     plt.ylabel(plot_axes)
    #     plt.show()

    return filtered


def calc_accel(pos_data):
    dt = 1/1000
    t = np.arange(0,10.0,dt)
    
    velocity_data = np.diff(pos_data) / dt

    # Calculate acceleration (second derivative of position)
    acceleration_data = np.diff(velocity_data) / dt
    filtered_accel = fft_filter_set(acceleration_data,0.0, 1.0)

    return filtered_accel

def read_ref(serial):
        data = []
        ref = []
        t= [ ]
        sampling_rate = 1/1000
        data_received = 0
        while data_received < 10000:
            dat_str = serial.read_until(b'\n'); 
            dat_f = list(map(float,dat_str.split())) 
            ref.append(dat_f[0] * (2*math.pi))  ## Ref acceleration is in turns/s/s, this converts it to rad/s/s
            data.append(dat_f[1]*(math.pi/180))  ## output position in deg comes in, here it is converted to rad and then used for accel calculation

            # ref.append(dat_f[0])   ## Ref acceleration is in turns/s/s, this converts it to rad/s/s
            # data.append(dat_f[1])  ## output position in deg comes in, here it is converted to rad and then used for accel calculation
    
            t.append(data_received*sampling_rate)
 
            print("Getting data\n", data_received)

            data_received+=1

        
        filtered_data = fft_filter_set(data,0.0, 1.2) 
        filtered_acceleration = calc_accel(filtered_data)

        plt.plot(t,ref,'r-', t[:-2],filtered_acceleration,'b-')
        # plt.plot(t,ref,'r-', t,data,'b-')
        plt.ylabel('Acceleration [rad/s/s]')
        plt.xlabel('Time [sec]')
        plt.title("Input vs Output Acceleration, 05ms Walking Profile")
        plt.legend(['Input Acceleration', 'Output Acceleration']) 


        plt.show()
    
def read_serial(serial):
        data = []
        t= [ ]
        sampling_rate = 1/320
        data_received = 0
        while data_received < 1500:
            dat_str = serial.read_until(b'\n');  # get the data as a string, ints seperated by spaces
     
            data_float = float(dat_str)
            data.append(data_float)
            t.append(data_received*sampling_rate)
            # if not dat_str:
            #     break
            print("Getting data\n", data_received)

            data_received+=1

        # filtered_data = fft_filter(data)
        # filtered_acceleration = calc_accel(filtered_data)

        # plt.ylim(-2.0, 2.0)
        plt.plot(t, data,'b-')
        plt.ylabel('IMU Magnitude [m/s/s]')
        plt.xlabel('Time [sec]')
        plt.title("0.8 m/s Walking Speeed IMU Magnitude")
        plt.text(3, 7, 'b = 0.010', fontsize=12, color='red')
        plt.show()
     


def main():
    ser = serial.Serial("/dev/ttyACM0", 230400, timeout=3.0)

    has_quit = False
    # menu loop
    while not has_quit:
        
        try: 
            # read_ref(ser)
            read_serial(ser)
         
            break
        except:
            print("error open serial port")
            exit()



main()