# import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import figaspect
import matplotlib.ticker as mtick
# from scipy.stats import mannwhitneyu
import csv
import os
import bisect


def readInterpolatedValues(files, times, time_column, columns=[]):
    first_file = True
    for filename in files:
        c0 = []  # Times
        try:
            with open(filename, 'r') as csvfile:
                csv_reader = csv.reader(csvfile, delimiter=',')
                # read_types = False
                if first_file:
                    column_names = next(csv_reader)
                    if not columns:  # if no columns given, read all but time
                        columns = range(1,len(column_names))
                    column_names = [column_names[i] for i in columns]
                else:
                    next(csv_reader)  # skip first line

                cs = [[] for _ in range(len(columns))]  # colums to read
                for row in csv_reader:
                    c0.append(float(row[time_column]))
                    for i in range(len(cs)):
                        cs[i].append(float(row[columns[i]]))
        except IOError:
            print('File \'{}\' could not be opened; skipping file'.format(filename))
            continue

        # interpolate data to whole seconds for easier averaging
        cs_n = [[] for _ in range(len(columns))]  # colums adjusted to specified times
        ci = 0  # current index
        for t in times:
            if ci < len(c0):
                ci = bisect.bisect_left(c0, t, ci)
            if ci >= len(c0):  # end of times reached; keep last value
                for i in range(len(cs_n)):
                    cs_n[i].append(cs[i][-1])
            elif ci == 0:  # if t smaller than first value, keep first
                for i in range(len(cs_n)):
                    cs_n[i].append(cs[i][0])
            else:
                t1 = c0[ci - 1]
                t2 = c0[ci]
                for i in range(len(cs_n)):
                    val = cs[i][ci - 1] * (t - t1) / (t2 - t1) + cs[i][ci] * (t2 - t) / (t2 - t1)
                    cs_n[i].append(float(val))

        if first_file:
            results = [[] for _ in range(len(columns))]

        for i in range(len(cs_n)):
            results[i].append(cs_n[i])

        first_file = False

    return results, column_names

NORM_TO_ONE_SET = {'Volume accuracy', 'Covered ROI volume', 'Vol. acc.', 'Vol. acc. bbx', 'Vol. acc. ma0', 'Vol. acc. ma50'}
NORM_TO_FRUIT_NUM_SET = {'Detected ROI cluster', 'Det. cluster ma0', 'Det. cluster ma50'}
FRUIT_NUM = 28

def generatePlots(plot_names, times, results_avg, out_folder):
    for i in range(len(plot_names)):
        if (plot_names[i] == 'Average volume accuracy'):
            plot_names[i] = 'Volume accuracy'

        fig = plt.figure(i, figsize=figaspect(1))

        for j in range(len(results_avg)):
            plt.plot(times, results_avg[j][i], 'C{}'.format(j), linewidth=3.0, label=labels[j])

        FONT_SIZE = 14

        plt.rc('font', size=FONT_SIZE)          # controls default text sizes
        plt.rc('axes', titlesize=FONT_SIZE)     # fontsize of the axes title
        plt.rc('axes', labelsize=FONT_SIZE)    # fontsize of the x and y labels
        plt.rc('xtick', labelsize=FONT_SIZE)    # fontsize of the tick labels
        plt.rc('ytick', labelsize=FONT_SIZE)    # fontsize of the tick labels
        plt.rc('legend', fontsize=FONT_SIZE)    # legend fontsize
        plt.rc('figure', titlesize=FONT_SIZE)  # fontsize of the figure title

        plt.xlabel('Plan length (s)')
        plt.ylabel(plot_names[i])
        plt.xlim((0, times[-1]))
        if (plot_names[i] in NORM_TO_FRUIT_NUM_SET):
            plt.ylim((0, FRUIT_NUM))
        elif (plot_names[i] in NORM_TO_ONE_SET):
            plt.gca().yaxis.set_major_formatter(mtick.PercentFormatter(1.0))
            plt.ylim((0, 1))
        # plt.title(plot_names[i])
        plt.legend()
        plt.grid()
        plt.tight_layout()
        figname = plot_names[i].lower().replace(" ", "_") + ".png"
        plt.savefig(os.path.join(out_folder, figname))
        plt.clf()


def generateResultsFile(plot_names, results, out_folder):
    for i in range(len(plot_names)):
        final_results = []

        for j in range(len(results)):
            final_results.append([])
            for result in results[j][i]:
                # plt.plot(times, result[i], 'C{}--'.format(i), alpha=0.2)
                final_results[j].append(result[-1])


# Parameters
out_folder = "plots"
out_folder_old = out_folder + "_old"
out_folder_ec = out_folder + "_ec"

input_folders = ['nna2_insert_while_moving', 'nna3_no_insert_when_moving', 'nna4_old_superellipsoid']
labels = ['Insert moving', 'No insert moving', 'old se']
input_ranges = [range(0, 20), range(0, 20), range(0, 20)]

time_column = 0
columns = [3, 4, 5, 6, 7, 8, 9]
columns_old = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18]
columns_ec = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]

max_time = 300
times = range(max_time + 1)

files = []
files_old = []
files_ec = []

for i in range(len(input_folders)):
    files.append([input_folders[i] + '/planner_results_{}.csv'.format(j) for j in input_ranges[i]])
    files_old.append([input_folders[i] + '/planner_results_old{}.csv'.format(j) for j in input_ranges[i]])
    files_ec.append([input_folders[i] + '/planner_results_ec{}.csv'.format(j) for j in input_ranges[i]])

results = []
results_old = []
results_ec = []
plot_names = []
plot_names_old = []
plot_names_ec = []
for i in range(len(input_folders)):
    res, plot_names = readInterpolatedValues(files[i], times, time_column, columns)
    res_old, plot_names_old = readInterpolatedValues(files_old[i], times, time_column, columns_old)
    res_ec, plot_names_ec = readInterpolatedValues(files_ec[i], times, time_column, columns_ec)
    results.append(res)
    results_old.append(res_old)
    results_ec.append(res_ec)

results_avg = []
results_avg_old = []
results_avg_ec = []
for i in range(len(input_folders)):
    results_avg.append([np.average(results[i][j], axis=0) for j in range(len(results[i]))])
    results_avg_old.append([np.average(results_old[i][j], axis=0) for j in range(len(results_old[i]))])
    results_avg_ec.append([np.average(results_ec[i][j], axis=0) for j in range(len(results_ec[i]))])

results_to_write = ('Detected ROI cluster', 'Average distance', 'Volume accuracy', 'Covered ROI volume')

generatePlots(plot_names, times, results_avg, out_folder)
generatePlots(plot_names_old, times, results_avg_old, out_folder_old)
generatePlots(plot_names_ec, times, results_avg_ec, out_folder_ec)
