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
TITLE_MAP = {'Average volume accuracy': 'Volume accuracy', 'Det. cluster ma50': 'Detected fruits', 'Dist. ma50': 'Center distance',
'Vol. acc. ma50': 'Volume accuracy'}
FRUIT_NUM = 14
COLORS = ['darkblue', 'blue', 'lightblue', 'firebrick', 'red', 'salmon']

def generatePlots(plot_names, times, results_avg, out_folder, labels):
    for i in range(len(plot_names)):
        if (plot_names[i] in TITLE_MAP):
            plot_names[i] = TITLE_MAP[plot_names[i]]

        fig = plt.figure(i, figsize=figaspect(1))

        FONT_SIZE = 14

        plt.rc('font', size=FONT_SIZE)          # controls default text sizes
        plt.rc('axes', titlesize=FONT_SIZE)     # fontsize of the axes title
        plt.rc('axes', labelsize=FONT_SIZE)    # fontsize of the x and y labels
        plt.rc('xtick', labelsize=FONT_SIZE)    # fontsize of the tick labels
        plt.rc('ytick', labelsize=FONT_SIZE)    # fontsize of the tick labels
        plt.rc('legend', fontsize=10)    # legend fontsize
        plt.rc('figure', titlesize=FONT_SIZE)  # fontsize of the figure title

        for j in range(len(results_avg)):
            plt.plot(times, results_avg[j][i], linewidth=3.0, label=labels[j], color=COLORS[j])

        plt.xlabel('Plan length (s)')
        plt.ylabel(plot_names[i])
        plt.xlim((0, times[-1]))
        if (plot_names[i] in NORM_TO_FRUIT_NUM_SET):
            plt.ylim((0, FRUIT_NUM))
        elif (plot_names[i] in NORM_TO_ONE_SET):
            plt.gca().yaxis.set_major_formatter(mtick.PercentFormatter(1.0))
            plt.ylim((0, 1))
        # plt.title(plot_names[i])
        plt.legend(ncol=2)
        plt.grid()
        plt.tight_layout()
        figname = plot_names[i].lower().replace(" ", "_") + ".png"
        plt.savefig(os.path.join(out_folder, figname))
        plt.clf()

def generateResultsLatexOld(plot_names, results, out_folder, labels):
    format_strings = ["{:.1f}", "{:.1f}", "{:.2f}"]
    with open(os.path.join(out_folder, 'table.txt'), 'w') as outfile:
        outfile.write('\\begin{table} \\centering \\begin{tabularx}{\\linewidth}{ c | X | X | X | X | X | X |} \\cline{2-7}\n')
        outfile.write('                & Old (all) & Old (MA0) & Old (MA50) & SE (all) & SE (MA0) & SE (MA50) \\\\ \\hline\n')
        for i in range(len(plot_names)):
            outfile.write(plot_names[i])
            if (plot_names[i] == 'Center distance'):
                results[:,i] = results[:,i] * 100  # convert to cm
            for j in range(len(labels)):
                outfile.write(' & ' + format_strings[i].format(results[j][i]))
            outfile.write(' \\\\ \\hline\n')
        outfile.write('\\end{tabularx} \\caption{TODO} \\label{tab:res_table} \\end{table}\n')

def generateResultsLatex(plot_names, results, out_folder, labels):
    format_strings = ["{:.1f}", "{:.1f}", "{:.2f}"]
    with open(os.path.join(out_folder, 'table.txt'), 'w') as outfile:
        outfile.write('\\begin{table} \\centering \\begin{tabularx}{\\linewidth}{ c | X | X | X |} \\cline{2-4}\n')
        outfile.write('                & Fruits & Center & Vol. \\\\ \\hline\n')
        for i in range(len(plot_names)):
            if (plot_names[i] == 'Center distance'):
                results[:,i] = results[:,i] * 100  # convert to cm
        for i in range(len(labels)):
            outfile.write(labels[i])
            for j in range(len(plot_names)):
                outfile.write(' & ' + format_strings[j].format(results[i][j]))
            outfile.write(' \\\\ \\hline\n')
        outfile.write('\\end{tabularx} \\caption{TODO} \\label{tab:res_table} \\end{table}\n')

def generateResultsLatexWithStd(plot_names, results, results_std, out_folder, labels):
    TABLE_LABLE_MAP = {'Old (all)': '\\makecell[c]{Old\\\\all}', 'Old (MA0)': '\\makecell[c]{Old\\\\MA0}', 'Old (MA50)': '\\makecell[c]{Old\\\\MA50}',
                       'SE (all)': '\\makecell[c]{SE\\\\all}', 'SE (MA0)': '\\makecell[c]{SE\\\\MA0}', 'SE (MA50)': '\\makecell[c]{SE\\\\MA50}'}
    format_strings = ["\\makecell[r]{{{:.1f}\\\\ $\\pm$ {:.1f}}}", "\makecell[r]{{{:.1f}\\\\ $\\pm$ {:.1f}}}", "\makecell[r]{{{:.2f}\\\\ $\\pm$ {:.2f}}}"]
    with open(os.path.join(out_folder, 'table.txt'), 'w') as outfile:
        outfile.write('\\begin{table} \\centering \\begin{tabular}{ c | c | c | c |} \\cline{2-4}\n')
        outfile.write('                & Fruits & Center & Vol. \\\\ \\hline\\hline\n')
        for i in range(len(plot_names)):
            if (plot_names[i] == 'Center distance'):
                results[:,i] = results[:,i] * 100  # convert to cm
                results_std[:,i] = results_std[:,i] * 100  # convert to cm
        for i in range(len(labels)):
            outfile.write(TABLE_LABLE_MAP[labels[i]])
            for j in range(len(plot_names)):
                outfile.write(' & ' + format_strings[j].format(results[i][j], results_std[i][j]))
            if (i == 1 or i == 3):
                outfile.write(' \\\\ \\hline\\hline\n')
            else:
                outfile.write(' \\\\ \\hline\n')
                
        outfile.write('\\end{tabular} \\caption{TODO} \\label{tab:res_table} \\end{table}\n')

def generateResultsFile(plot_names, results, out_folder, labels):
    with open(os.path.join(out_folder, 'results.txt'), 'w') as outfile:
        for i in range(len(plot_names)):
            outfile.write(plot_names[i] + '\n')
            if (plot_names[i] == 'Center distance'):
                results[:,i] = results[:,i] * 100  # convert to cm
            for j in range(len(labels)):
                outfile.write('\t' + labels[j] + ': ' + str(results[j][i]) + '\n')


# Parameters
out_folder = "plots_comp_se"
input_folder = 'nna1'
input_range = range(0, 20)
labels = ['Old (all)', 'Old (MA0)', 'Old (MA50)', 'SE (all)', 'SE (MA0)', 'SE (MA50)']

time_column = 0
columns_old = [3, 6, 7]
columns_old_ma0 = [13, 14, 15]
columns_old_ma50 = [16, 17, 18]
columns_ec = [3, 4, 5]
columns_ec_ma0 = [9, 10, 11]
columns_ec_ma50 = [15, 16, 17]
columns = [columns_old, columns_old_ma0, columns_old_ma50, columns_ec, columns_ec_ma0, columns_ec_ma50]

max_time = 300
times = range(max_time + 1)

files = []

for i in range(3):
    files.append([input_folder + '/planner_results_old{}.csv'.format(j) for j in input_range])

for i in range(3):
    files.append([input_folder + '/planner_results_ec{}.csv'.format(j) for j in input_range])

results = []
plot_names = []
for i in range(len(columns)):
    res, plot_names = readInterpolatedValues(files[i], times, time_column, columns[i])
    results.append(res)

results_avg = []
results_std = []
for i in range(len(results)):
    results_avg.append([np.average(results[i][j], axis=0) for j in range(len(results[i]))])
    results_std.append([np.std(results[i][j], axis=0) for j in range(len(results[i]))])

generatePlots(plot_names, times, results_avg, out_folder, labels)

reorder_labels = np.array([0,3,1,4,2,5])

generateResultsFile(plot_names, np.array(results_avg)[reorder_labels, :, -1], out_folder, np.array(labels)[reorder_labels])

generateResultsLatexWithStd(plot_names, np.array(results_avg)[reorder_labels, :, -1], np.array(results_std)[reorder_labels, :, -1], out_folder, np.array(labels)[reorder_labels])