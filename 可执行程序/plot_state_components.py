import argparse
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path.cwd() / ".matplotlib"))

import matplotlib.pyplot as plt
import numpy as np


def load_data(path: Path) -> np.ndarray:
    rows = []
    with path.open("r", encoding="utf-8", errors="ignore") as fin:
        for line in fin:
            parts = line.split()
            if len(parts) < 10:
                continue
            try:
                rows.append([float(v) for v in parts])
            except ValueError:
                continue
    return np.asarray(rows, dtype=float).reshape(-1, 10)


def plot_xyz(time, data, title, labels, save_path):
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for i, ax in enumerate(axes):
        ax.plot(time, data[:, i], lw=1.0)
        ax.set_ylabel(labels[i])
        ax.grid(True, ls="--", lw=0.5)
    axes[0].set_title(title)
    axes[-1].set_xlabel("Time [s]")
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=200)
    return fig


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("track")
    parser.add_argument("--prefix", default="plot")
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    data = load_data(Path(args.track))
    t = data[:, 0]

    figs = [
        plot_xyz(t, data[:, 1:4], "Position", ["Lat [deg]", "Lon [deg]", "H [m]"], f"{args.prefix}_pos.png"),
        plot_xyz(t, data[:, 4:7], "Velocity", ["Vn [m/s]", "Ve [m/s]", "Vd [m/s]"], f"{args.prefix}_vel.png"),
        plot_xyz(t, data[:, 7:10], "Attitude", ["Roll [deg]", "Pitch [deg]", "Yaw [deg]"], f"{args.prefix}_att.png"),
    ]
    if args.show:
        plt.show()
    else:
        for fig in figs:
            plt.close(fig)


if __name__ == "__main__":
    main()
