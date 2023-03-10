# import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import figaspect
import matplotlib.ticker as mtick
from scipy.stats import mannwhitneyu
import csv
import os
import bisect
    
def readValues(filename, time_columns=[], value_columns=[]):
    times = []
    values = []
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
            for row in csv_reader:
                for i in range(len(time_columns)):
                    times[i].append(float(row[time_columns[i]]))
                for i in range(len(value_columns)):
                    values[i].append(float(row[value_columns[i]]))

    except IOError:
        print('File \'{}\' could not be opened; skipping file'.format(filename))
        return [], [], [], []

    return times, values, time_names, value_names

NORM_TO_ONE_SET = {'Volume accuracy', 'Covered ROI volume', 'Vol. acc.', 'Vol. acc. bbx', 'Vol. acc. ma0', 'Vol. acc. ma50'}
NORM_TO_FRUIT_NUM_SET = {'Detected ROI cluster', 'Det. cluster ma0', 'Det. cluster ma50'}
TITLE_MAP = {'Average volume accuracy': 'Volume accuracy', 'Det. cluster ma50': 'Detected fruits', 'Dist. ma50': 'Center distance',
'Vol. acc. ma50': 'Volume accuracy'}
FRUIT_NUM = 14
#COLORS = ['darkblue', 'blue', 'lightblue', 'firebrick', 'red', 'salmon']
COLORS = ['darkblue', 'lightblue', 'firebrick', 'salmon']

def generatePlots(time_names, value_names, times_rvp, values_rvp, segments_rvp, times_vmp, values_vmp, segments_vmp, out_folder):
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

            plt.plot(times_rvp[i], values_rvp[j], linewidth=3.0, label="RVP", color='darkblue')
            for k in range(len(segments_rvp)):
                plt.plot(times_rvp[i][k], values_rvp[j][k], marker='x', mec=cmap(int(segments_rvp[k])))

            plt.plot(times_vmp[i], values_vmp[j], linewidth=3.0, label="VMP", color='firebrick')
            for k in range(len(segments_vmp)):
                plt.plot(times_vmp[i][k], values_vmp[j][k], marker='x', mec=cmap(int(segments_vmp[k])))

            plt.xlabel(time_names[i])
            plt.ylabel(value_names[j])
            plt.xlim(0, max(times_rvp[i][-1], times_vmp[i][-1]))
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

rvp_folder = 'rvp/t11_w23_t60/'
vmp_folder = 'vmp/t11_w23_t60/'

file_rvp = rvp_folder + 'planner_results_ec0.csv'
file_vmp = vmp_folder + 'planner_results_ec0.csv'

file_rvp_old = rvp_folder + 'planner_results_old0.csv'
file_vmp_old = vmp_folder + 'planner_results_old0.csv'

times_rvp, values_rvp, time_names_rvp, value_names_rvp = readValues(file_rvp, time_columns, columns_ec)
segments_rvp = values_rvp[-1]
times_vmp, values_vmp, time_names_vmp, value_names_vmp = readValues(file_vmp, time_columns, columns_ec)
segments_vmp = values_vmp[-1]

times_rvp_old, values_rvp_old, time_names_rvp_old, value_names_rvp_old = readValues(file_rvp_old, time_columns, columns_old)
segments_rvp_old = values_rvp_old[-1]
times_vmp_old, values_vmp_old, time_names_vmp_old, value_names_vmp_old = readValues(file_vmp_old, time_columns, columns_old)
segments_vmp_old = values_vmp_old[-1]

generatePlots(time_names_rvp, value_names_rvp, times_rvp, values_rvp, segments_rvp, times_vmp, values_vmp, segments_vmp, out_folder)
generatePlots(time_names_rvp_old, value_names_rvp_old, times_rvp_old, values_rvp_old, segments_rvp_old, times_vmp_old, values_vmp_old, segments_vmp_old, out_folder_old)
