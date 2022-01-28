"""
Created on Mon Dec 20 11:26:39 2021

@author: Jake Merrell

Python Version: 3.9.9

Description: This script is used to visualize the data gathered with the XO-NANO Baldy Rev2 board. The script is split into 3 main sections
1. File Selction and Interpolation
    This section has you pick the file you would like to evaluate and interpolates the data between dropped BLE packets so IMU and Smartfoam measurements align
    
2. Post Processing / Filtering section
    This section is where we clean up the data before ploting it
    
3. Visualization / Plotting
    As the title indicates we will use this section to plot the data 
    
If you have any questions or problems with this script please feel free to reach out to me @ jake.merrell@xonano.com
"""

# Libaries
import pandas as pd
import tkinter as tk
from tkinter import filedialog
from scipy.signal import filtfilt, butter
import csv 
import numpy as np
import matplotlib.pyplot as plt
from tkinter.messagebox import askyesno
from scipy.signal import find_peaks

plt.close('all')

# Functions
def pick_file(select_title):
    root = tk.Tk()
    root.withdraw()
    root.attributes('-topmost', True)
    file_name = filedialog.askopenfilenames(filetypes=[("Test Data", '.csv')], title=select_title)
    return file_name

def butter_lowpass(data, highcut, fs, order):
    nyq = 0.5 * fs
    high = highcut / nyq
    b,a = butter(order,high)
    y = filtfilt(b, a, data)
    return y

def running_mean(x,N):
    cumsum = np.cumsum(np.insert(x,0,0))
    cumsum.append(0)
    return (cumsum[N:]-cumsum[:-N])/float(N)

def confirm(message):
    root = tk.Tk()
    root.title('Tkinter Yes/No Dialog')
    root.geometry('300x150')
    answer = askyesno(title='Confirmation', message = message)
    if answer:
        root.destroy()
    return answer

    
#%% Pick File and interpolate missing data so everything is aligned

# Select the file to be evaluated
file_names = pick_file('Select the files for analysis')
columns = ['V_0', 'V_1', 'V_2', 'V_3', 'V_4', 'V_5', 'A_X', 'A_Y', 'A_Z', 'G_X', 'G_Y', 'G_Z', 'M_X', 'M_Y', 'M_Z','Count ADC','Count IMU']
All_data = {}
for i, file in enumerate(file_names):
    reader = csv.reader(open(file))
    temp_list = list(reader)
    temp_cols = []
 
    # Convert the lists into column strings and float numbers
    for l in range(len(temp_list)):
        temp_cols.append(temp_list[l][0])
        temp_list[l] = temp_list[l][1:-1]
        temp_list[l] = [float(i) for i in temp_list[l]]
        
    # Find starting indecies in counts
    adc10 = temp_list[15][:100]
    imu10 = temp_list[16][:100]
    both = min(set(adc10).intersection(imu10))
    startadc = adc10.index(both)
    startimu = imu10.index(both)
    n_Data = [[] for _ in range(17)]
    
    # Trim all data to align at the beginning
    if startadc > 0:
        for a in range(6):
            temp_list[a] = temp_list[a][startadc:]
    if startimu > 0:
        for u in range(3):
            temp_list[u+9] = temp_list[u+9][startimu:]            
    temp_list[15] = temp_list[15][startadc:]
    temp_list[16] = temp_list[16][startimu:]
    
    # Look for and fill holes in ADC data
    for n, cvalue in enumerate(temp_list[15][:-1]):
        wvalue = 0
        nextvalue = temp_list[15][n+1]
        while nextvalue-(cvalue+wvalue)>1:
            n_Data[15].append(wvalue+cvalue)
            wvalue +=1
        n_Data[15].append(cvalue+wvalue)
        
        missed = wvalue
        
        # Fills data when missing up to 255 index
        if cvalue-nextvalue > 200:
            hvalue = -1
            while cvalue+hvalue < 255:
                hvalue += 1
                n_Data[15].append(hvalue+cvalue)
            if hvalue>0: missed = missed + hvalue
                
            # Fills the low side data if zero is missing              
            if nextvalue != 0:
                zvalue = -1
                while zvalue < nextvalue:
                    zvalue +=1
                    n_Data[15].append(zvalue)
                if zvalue>0: missed = missed + zvalue
                    
        # Perform all ADC interpolation for missing data
        for t in range(6):
            n_Data[t].extend(temp_list[t][n*10:n*10+10])
            if missed > 0:
                x = (temp_list[t][n*10+1]-temp_list[t][n*10])/(missed*10)
                for z in range(missed*10):
                    n_Data[t].append(temp_list[t][n*10]+z*x)
        
    # Look for and fill holes in IMU data
    for m, cvalue in enumerate(temp_list[16][:-1]):
        wvalue = 0
        nextvalue = temp_list[16][m+1]

        # Fills data when skipped                
        while nextvalue-(cvalue+wvalue)>1:
            n_Data[16].append(wvalue+cvalue)
            wvalue +=1
        n_Data[16].append(cvalue+wvalue)
        missed = wvalue
        
        # Fills data when missing up to 255 index
        if cvalue-nextvalue > 200:
            hvalue = -1
            while cvalue+hvalue < 255:
                hvalue += 1
                n_Data[16].append(hvalue+cvalue)
            if hvalue>0: missed = missed + hvalue
            
            # Fills the low side data
            if nextvalue != 0:
                zvalue = -1
                while zvalue < nextvalue:
                    zvalue +=1
                    n_Data[16].append(zvalue)
                if zvalue>0: missed = missed + zvalue        
            
        # Perform all IMU interpolation for missing data
        for t in range(9):
            n_Data[t+6].extend(temp_list[t+6][m*10:m*10+10])
            if missed > 0:
                x = (temp_list[t+6][m*10+1]-temp_list[t+6][m*10])/(missed*10)
                for z in range(missed*10):
                    n_Data[t+6].append(temp_list[t+6][m*10]+z*x)
                
    # Move all data to a DataFrame for processing        
    Data = pd.DataFrame(n_Data).T
    Data.columns = columns
    All_data[file.split('/')[-1].split('.')[0]] = Data
        
#%% Post process the data

highcut = 50 # This can be changed but 50 is generally a good cutoff frequency for the data
fs = 200 # Sampling frequency
order = 5 #
Filt_data = {}
N = 40

# This will loop through all of the variables and filter each
for d, name in enumerate(All_data):
    
    # Filter Data
    columns = All_data[name].columns.values.tolist()
    columns = columns[0:-2]
    Filt_data['butter ' + name] = pd.DataFrame()
    for i, col in enumerate(columns):
        
        # Butterworth
        Filt_data['butter '+ name]['butter '+col] = butter_lowpass(All_data[name][col], highcut, fs, order)
        


#%% Ploting Section

# Ask if the data should be plotted
plot = confirm('Would you like to plot the data?')

if plot:
    for i, name in enumerate(Filt_data):
        
        time = np.arange(0,len(Filt_data[name])/fs,1/fs)
        fig, axs = plt.subplots(2,1)
        fig.suptitle('Smartfoam and IMU Measurements')
        
        # Plot the Smartfoam signals
        axs[0].set_title('Smartfoam')
        axs[0].plot(time,Filt_data[name].iloc[:,0:6])
        axs[0].set_ylabel('Smartfoam Response (V)')
        axs[0].set_xlabel('Time (sec)')
        axs[0].legend(Filt_data[name].columns[0:6])
       
        
        # Plot the accelerometer data
        axs[1].set_title('IMU')
        axs[1].plot(time,Filt_data[name].iloc[:,6:12])
        axs[1].set_ylabel('Acceleration (g\'s)')
        axs[1].set_xlabel('Time (sec)')
        axs[1].legend(Filt_data[name].columns[6:12])
        plt.tight_layout()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    