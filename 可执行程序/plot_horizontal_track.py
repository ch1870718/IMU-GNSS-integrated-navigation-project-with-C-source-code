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
            if len(parts) < 4:
                continue
            try:
                rows.append([float(v) for v in parts])
            except ValueError:
                continue
    return np.asarray(rows, dtype=float)


def blh_to_ne(data: np.ndarray) -> np.ndarray:
    lat = np.deg2rad(data[:, 1])
    lon = np.deg2rad(data[:, 2])
    lat0 = lat[0]
    lon0 = lon[0]
    h0 = data[0, 3]
    a = 6378137.0
    e2 = 0.00669437999014
    sin_lat0 = np.sin(lat0)
    rn = a / np.sqrt(1.0 - e2 * sin_lat0 * sin_lat0)
    rm = a * (1.0 - e2) / (1.0 - e2 * sin_lat0 * sin_lat0) ** 1.5
    north = (lat - lat0) * (rm + h0)
    east = (lon - lon0) * (rn + h0) * np.cos(lat0)
    return np.column_stack((north, east))


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot horizontal track")
    parser.add_argument("track")
    parser.add_argument("--ref")
    parser.add_argument("--save")
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--label", default="track")
    parser.add_argument("--ref-label", default="ref")
    parser.add_argument("--title", default="Horizontal Track")
    args = parser.parse_args()

    track = blh_to_ne(load_track(Path(args.track)))
    ref = blh_to_ne(load_track(Path(args.ref))) if args.ref else None

    plt.figure(figsize=(8, 6))
    plt.plot(track[:, 1], track[:, 0], label=args.label, lw=1.2)
    if ref is not None:
        plt.plot(ref[:, 1], ref[:, 0], label=args.ref_label, lw=1.0)
    plt.xlabel("East [m]")
    plt.ylabel("North [m]")
    plt.title(args.title)
    plt.axis("equal")
    plt.grid(True, ls="--", lw=0.5)
    plt.legend()
    plt.tight_layout()
    if args.save:
        plt.savefig(args.save, dpi=200)
    if args.show or not args.save:
        plt.show()
    else:
        plt.close()


if __name__ == "__main__":
    main()
