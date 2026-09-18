import argparse
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path.cwd() / ".matplotlib"))

import matplotlib.pyplot as plt
import numpy as np


def load_track(path: Path) -> np.ndarray:
    rows = []
    with path.open("r", encoding="utf-8", errors="ignore") as fin:
        for line in fin:
            parts = line.split()
            if len(parts) < 7:
                continue
            try:
                rows.append([float(v) for v in parts])
            except ValueError:
                continue
    return np.asarray(rows, dtype=float)


def blh_error_to_ned(est: np.ndarray, ref: np.ndarray) -> np.ndarray:
    deg2rad = np.pi / 180.0
    a = 6378137.0
    e2 = 0.00669437999014
    lat0 = ref[0, 1] * deg2rad
    h0 = ref[0, 3]
    sin_lat0 = np.sin(lat0)
    rn = a / np.sqrt(1.0 - e2 * sin_lat0 * sin_lat0)
    rm = a * (1.0 - e2) / (1.0 - e2 * sin_lat0 * sin_lat0) ** 1.5
    dlat = (est[:, 1] - ref[:, 1]) * deg2rad
    dlon = (est[:, 2] - ref[:, 2]) * deg2rad
    dh = est[:, 3] - ref[:, 3]
    return np.column_stack((dlat * (rm + h0), dlon * (rn + h0) * np.cos(lat0), -dh))


def interp_track(track: np.ndarray, time: np.ndarray, end_col: int) -> np.ndarray:
    cols = [time]
    for i in range(1, end_col):
        cols.append(np.interp(time, track[:, 0], track[:, i]))
    return np.column_stack(cols)


def plot_xyz(time: np.ndarray, err: np.ndarray, title: str, labels: list[str], save_path: str) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for i, ax in enumerate(axes):
        ax.plot(time, err[:, i], lw=1.0)
        ax.set_ylabel(labels[i])
        ax.grid(True, ls="--", lw=0.5)
    axes[0].set_title(title)
    axes[-1].set_xlabel("Time [s]")
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=200)
    return fig


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("track")
    parser.add_argument("truth")
    parser.add_argument("--prefix", default="error")
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    track = load_track(Path(args.track))
    truth = load_track(Path(args.truth))

    t0 = max(track[0, 0], truth[0, 0])
    t1 = min(track[-1, 0], truth[-1, 0])
    dt = np.mean(np.diff(track[:, 0]))
    time = np.arange(t0, t1 + 0.5 * dt, dt)

    est = interp_track(track, time, 7)
    ref = interp_track(truth, time, 7)

    pos_err = blh_error_to_ned(est[:, :4], ref[:, :4])
    vel_err = est[:, 4:7] - ref[:, 4:7]

    figs = [
        plot_xyz(time, pos_err, "Position Error in NED", ["dN [m]", "dE [m]", "dD [m]"], f"{args.prefix}_pos_err_ned.png"),
        plot_xyz(time, vel_err, "Velocity Error in NED", ["dVn [m/s]", "dVe [m/s]", "dVd [m/s]"], f"{args.prefix}_vel_err_ned.png"),
    ]
    if args.show:
        plt.show()
    else:
        for fig in figs:
            plt.close(fig)


if __name__ == "__main__":
    main()
