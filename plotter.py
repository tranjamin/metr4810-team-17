import argparse
import numpy as np
import matplotlib.pyplot as plt
import os
import pandas as pd

FIGURE_PATH = "graphs"

plt.rcParams['text.usetex'] = True
plt.rcParams['savefig.format'] = "pdf"


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--file", help="Name of .npy file containing robot log data to plot")
    parser.add_argument("--output", action=argparse.BooleanOptionalAction)

    args = parser.parse_args()
    data_file_name = args.file
    data = pd.read_csv(data_file_name)  # skip first row which contains garbage datapoint

    t = data["t"].to_numpy()
    x = data["x"].to_numpy()
    y = data["y"].to_numpy()
    theta = data["theta"].to_numpy()

    start_index = (x != 0).argmax(axis=0)
    x = x[start_index:]
    y = y[start_index:]
    start_time = t[start_index]
    t = t[start_index:] - start_time

    plt.rc('font', size=15)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(x, y)
    ax.set_xlabel(r"$x$ position")
    ax.set_ylabel(r"$y$ position")
    ax.grid(True)
    ax.axis('square')
    ax.set_xlim(0, 2000)
    ax.set_ylim(0, 2000)
    ax.set_aspect('equal', adjustable='box')
    y_tick_locs = ax.get_yticks()
    ax.set_xticks(y_tick_locs)

    if args.output:
        if not os.path.exists(FIGURE_PATH):
            os.makedirs(FIGURE_PATH)

        print(os.path.basename(data_file_name))
        plt.savefig(os.path.join(FIGURE_PATH, os.path.basename(data_file_name)[:-4] + ".pdf"), format="pdf") # remove .npy

    plt.show()