# import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import figaspect
import matplotlib.ticker as mtick
from scipy.stats import mannwhitneyu
import csv
import os
import bisect
from collections import OrderedDict
    
def readValues(filename, time_columns=[], value_columns=[], seg_column=22):
    times = []
    values = []
    segment_starts = []
    try:
        with open(filename, 'r') as csvfile:
            csv_reader = csv.reader(csvfile, delimiter=',')
            column_names = next(csv_reader)
            if not time_columns: # default time columns: first three
                time_columns=[0, 1, 2]
            if not value_columns:  # if no columns given, read all but time
                value_columns = range(len(time_columns), len(column_names))
            
            time_names = [column_names[i] for i in time_columns]
            value_names = [column_names[i] for i in value_columns]

            times = [[] for _ in range(len(time_columns))]  # colums to read
            values = [[] for _ in range(len(value_columns))]  # colums to read
            cur_seg = -1
            index = 0
            for row in csv_reader:
                for i in range(len(time_columns)):
                    times[i].append(float(row[time_columns[i]]))
                for i in range(len(value_columns)):
                    values[i].append(float(row[value_columns[i]]))
                    
                seg = int(row[seg_column])
                while seg > cur_seg:
                    segment_starts.append(index)
                    cur_seg += 1
                
                index += 1
                
            segment_starts.append(index)

    except IOError:
        print('File \'{}\' could not be opened; skipping file'.format(filename))
        return [], [], [], []

    return times, values, time_names, value_names, segment_starts

NORM_TO_ONE_SET = {'Volume accuracy', 'Covered ROI volume', 'Vol. acc.', 'Vol. acc. bbx', 'Vol. acc. ma0', 'Vol. acc. ma50'}
NORM_TO_FRUIT_NUM_SET = {'Detected ROI cluster', 'Det. cluster ma0', 'Det. cluster ma50'}
TITLE_MAP = {'Average volume accuracy': 'Volume accuracy', 'Det. cluster ma50': 'Detected fruits', 'Dist. ma50': 'Center distance',
'Vol. acc. ma50': 'Volume accuracy'}
FRUIT_NUM = 14
#COLORS = ['darkblue', 'blue', 'lightblue', 'firebrick', 'red', 'salmon']
COLORS = ['darkblue', 'lightblue', 'firebrick', 'salmon']

def generatePlots(time_names, value_names, times_rvp, values_rvp, segments_rvp, times_vmp, values_vmp, segments_vmp, out_folder, plot_vps=True):
    for i in range(len(time_names)):
        for j in range(len(value_names)):
            if (value_names[i] in TITLE_MAP):
                value_names[i] = TITLE_MAP[value_names[i]]

            fig = plt.figure(i, figsize=figaspect(1))

            FONT_SIZE = 14

            plt.rc('font', size=FONT_SIZE)          # controls default text sizes
            plt.rc('axes', titlesize=FONT_SIZE)     # fontsize of the axes title
            plt.rc('axes', labelsize=FONT_SIZE)    # fontsize of the x and y labels
            plt.rc('xtick', labelsize=FONT_SIZE)    # fontsize of the tick labels
            plt.rc('ytick', labelsize=FONT_SIZE)    # fontsize of the tick labels
            plt.rc('legend', fontsize=10)    # legend fontsize
            plt.rc('figure', titlesize=FONT_SIZE)  # fontsize of the figure title

            vals = np.linspace(0,1,256)
            np.random.shuffle(vals)
            cmap = plt.cm.colors.ListedColormap(plt.cm.jet(vals))

            for t in range(len(times_rvp)):
                lb = '_nolegend_'
                if t == 0:
                    lb="RVP"
                    
                plt.plot(times_rvp[t][i], values_rvp[t][j], linewidth=3.0, label=lb, color='darkblue')
                if plot_vps:
                    for k in range(len(segments_rvp[t])):
                        plt.plot(times_rvp[t][i][k], values_rvp[t][j][k], marker='x', mec=cmap(int(segments_rvp[t][k])))

                if t == 0:
                    lb="VMP"

                plt.plot(times_vmp[t][i], values_vmp[t][j], linewidth=3.0, label=lb, color='firebrick')
                if plot_vps:
                    for k in range(len(segments_vmp[t])):
                        plt.plot(times_vmp[t][i][k], values_vmp[t][j][k], marker='x', mec=cmap(int(segments_vmp[t][k])))

            plt.xlabel(time_names[i])
            plt.ylabel(value_names[j])

            #plt.xlim(0, max(times_rvp[i][-1], times_vmp[i][-1]))
            #if (value_names[j] in NORM_TO_FRUIT_NUM_SET):
            #    plt.ylim((0, FRUIT_NUM))
            #elif (value_names[j] in NORM_TO_ONE_SET):
            #    plt.gca().yaxis.set_major_formatter(mtick.PercentFormatter(1.0))
            #    plt.ylim((0, 1))
            # plt.title(value_names[j])
            plt.legend(ncol=2)
            plt.grid()
            plt.tight_layout()
            figname =  value_names[j].lower().replace(" ", "_") + "_" + time_names[i].lower().replace(" ", "_") + ".png"
            plt.savefig(os.path.join(out_folder, figname))
            plt.clf()
            
            for t in range(len(times_rvp)):
                print(figname, values_rvp[t][j][-1], values_vmp[t][j][-1])

def moveSegmentStartTimes(times_rvp, times_vmp, segment_starts_rvp, segment_starts_vmp):
    if len(segment_starts_rvp) != len(segment_starts_vmp):
        print('Invalid segment starts')
        return
    
    for i in range(len(segment_starts_rvp) - 1):
        seg_rvp = segment_starts_rvp[i] 
        seg_rvp_next = segment_starts_rvp[i+1]
        seg_vmp = segment_starts_vmp[i]
        seg_vmp_next = segment_starts_vmp[i+1]
        
        if times_rvp[0][seg_rvp] > times_vmp[0][seg_vmp]:
            vmp_offset = times_rvp[0][seg_rvp] - times_vmp[0][seg_vmp]
            for j in range(seg_vmp, seg_vmp_next):
                times_vmp[0][j] += vmp_offset
        elif times_vmp[0][seg_vmp] > times_rvp[0][seg_rvp]:
            rvp_offset = times_vmp[0][seg_vmp] - times_rvp[0][seg_rvp]
            for j in range(seg_rvp, seg_rvp_next):
                times_rvp[0][j] += rvp_offset

    #return times_rvp, times_vmp

# Parameters
out_folder = "plots"
out_folder_old = "plots_old"
input1_range = range(0, 1)
input2_range = range(0, 1)
labels = ['RVP (all)', 'RVP (MA50)', 'VMP (all)', 'VMP (MA50)']

time_columns = [0, 1, 2]
columns_ec = [3, 4, 5, 9, 10, 11, 15, 16, 17, 22]
#columns_ec_ma0 = [9, 10, 11]
#columns_ec_ma50 = [15, 16, 17]
#columns = [columns_ec, columns_ec_ma50]

columns_old = [3, 6, 7, 8]

#rvp_folder = 'rvp/t3_plan_length_15/'
#vmp_folder = 'vmp/t3-2_plan_length_15/'

#rvp_folders = ['rvp/t10_w22_t60/', 'rvp/t12_w22_t60/', 'rvp/t14_w22_t60/']
#vmp_folders = ['vmp/t10_w22_t60/', 'vmp/t12_w22_t60/', 'vmp/t14_w22_t60/']

rvp_folders = ['rvp/t11_w23_t60/', 'rvp/t13_w23_t60/', 'rvp/t15_w23_t60/']
vmp_folders = ['vmp/t11_w23_t60/', 'vmp/t13_w23_t60/', 'vmp/t15_w23_t60/']

file_rvp = [rvp_folder + 'planner_results_ec0.csv' for rvp_folder in rvp_folders]
file_vmp = [vmp_folder + 'planner_results_ec0.csv' for vmp_folder in vmp_folders]

times_rvp, values_rvp, time_names_rvp, value_names_rvp, segment_starts_rvp, segments_rvp = [], [], [], [], [], []
times_vmp, values_vmp, time_names_vmp, value_names_vmp, segment_starts_vmp, segments_vmp = [], [], [], [], [], []

for i in range(len(rvp_folders)):
    t, v, tn, vn, ss = readValues(file_rvp[i], time_columns, columns_ec, seg_column=22)
    times_rvp.append(t)
    values_rvp.append(v)
    time_names_rvp.append(tn)
    value_names_rvp.append(vn)
    segment_starts_rvp.append(ss)
    segments_rvp.append(v[-1])

for i in range(len(vmp_folders)):
    t, v, tn, vn, ss = readValues(file_vmp[i], time_columns, columns_ec, seg_column=22)
    times_vmp.append(t)
    values_vmp.append(v)
    time_names_vmp.append(tn)
    value_names_vmp.append(vn)
    segment_starts_vmp.append(ss)
    segments_vmp.append(v[-1])

for i in range(len(rvp_folders) - 1):
    moveSegmentStartTimes(times_rvp[i], times_rvp[i+1], segment_starts_rvp[i], segment_starts_rvp[i+1])
    #t1, t2 = moveSegmentStartTimes(times_rvp[i], times_rvp[i+1], segment_starts_rvp[i], segment_starts_rvp[i+1])
    #times_rvp[i] = t1
    #times_rvp[i+1] = t2

for i in range(len(vmp_folders) - 1):
    moveSegmentStartTimes(times_vmp[i], times_vmp[i+1], segment_starts_vmp[i], segment_starts_vmp[i+1])
    #t1, t2 = moveSegmentStartTimes(times_vmp[i], times_vmp[i+1], segment_starts_vmp[i], segment_starts_vmp[i+1])
    #times_vmp[i] = t1
    #times_vmp[i+1] = t2

for i in range(len(rvp_folders)):
    moveSegmentStartTimes(times_rvp[i], times_vmp[i], segment_starts_rvp[i], segment_starts_vmp[i])
    #trvp, tvmp = moveSegmentStartTimes(times_rvp[i], times_vmp[i], segment_starts_rvp[i], segment_starts_vmp[i])
    #times_rvp[i] = trvp
    #times_vmp[i] = tvmp

generatePlots(time_names_rvp[0], value_names_rvp[0], times_rvp, values_rvp, segments_rvp, times_vmp, values_vmp, segments_vmp, out_folder, plot_vps=False)
