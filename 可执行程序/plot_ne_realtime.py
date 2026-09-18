import math
import sys
from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt


DEFAULT_DATA_FILE = Path("output.txt")
POLL_INTERVAL_MS = 1000
EARTH_RADIUS_M = 6378137.0
PLOT_TIME_STEP_S = 5.0


class RealtimeNETrajectory:
    def __init__(self, data_file, plot_time_step_s=PLOT_TIME_STEP_S):
        self.data_file = data_file
        self.data_file_label = data_file.name
        self.plot_time_step_s = plot_time_step_s
        self.file_pos = 0
        self.ref_lat = None
        self.ref_lon = None
        self.last_plot_time = None
        self.times = []
        self.norths = []
        self.easts = []

        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.anim = None
        self.line, = self.ax.plot([], [], color="tab:blue", linewidth=1.5, label="Trajectory")
        self.current_point, = self.ax.plot([], [], "ro", markersize=6, label="Current")
        self.time_text = self.ax.text(
            0.02,
            0.98,
            "",
            transform=self.ax.transAxes,
            va="top",
            ha="left",
            fontsize=11,
            bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "none"},
        )

        self.ax.set_title("Real-Time N-E Trajectory")
        self.ax.set_xlabel("East (m)")
        self.ax.set_ylabel("North (m)")
        self.ax.grid(True, linestyle="--", alpha=0.4)
        self.ax.axis("equal")
        self.ax.legend(loc="upper right")

    def latlon_to_ne(self, lat_deg, lon_deg):
        ref_lat_rad = math.radians(self.ref_lat)
        d_lat = math.radians(lat_deg - self.ref_lat)
        d_lon = math.radians(lon_deg - self.ref_lon)
        north = d_lat * EARTH_RADIUS_M
        east = d_lon * EARTH_RADIUS_M * math.cos(ref_lat_rad)
        return north, east

    def parse_line(self, line):
        parts = line.strip().split()
        if len(parts) < 3:
            return None
        try:
            t = float(parts[0])
            lat = float(parts[1])
            lon = float(parts[2])
        except ValueError:
            return None
        return t, lat, lon

    def load_new_points(self):
        if not self.data_file.exists():
            return False

        current_size = self.data_file.stat().st_size
        if current_size < self.file_pos:
            self.file_pos = 0
            self.ref_lat = None
            self.ref_lon = None
            self.last_plot_time = None
            self.times = []
            self.norths = []
            self.easts = []

        updated = False
        with self.data_file.open("r", encoding="utf-8", errors="ignore") as f:
            f.seek(self.file_pos)
            for line in f:
                parsed = self.parse_line(line)
                if parsed is None:
                    continue
                t, lat, lon = parsed
                if self.ref_lat is None:
                    self.ref_lat = lat
                    self.ref_lon = lon
                if self.last_plot_time is None or t - self.last_plot_time >= self.plot_time_step_s:
                    north, east = self.latlon_to_ne(lat, lon)
                    self.times.append(t)
                    self.norths.append(north)
                    self.easts.append(east)
                    self.last_plot_time = t
                    updated = True
            self.file_pos = f.tell()

        return updated

    def update_plot(self, _frame):
        self.load_new_points()
        if not self.times:
            self.time_text.set_text("Waiting for data...\n{0}".format(self.data_file_label))
            return self.line, self.current_point, self.time_text

        self.line.set_data(self.easts, self.norths)
        self.current_point.set_data([self.easts[-1]], [self.norths[-1]])
        self.time_text.set_text(
            "Time: {0:.3f} s\nPoints: {1}".format(self.times[-1], len(self.times))
        )
        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.set_aspect("equal", adjustable="datalim")
        return self.line, self.current_point, self.time_text

    def run(self):
        self.load_new_points()
        self.anim = animation.FuncAnimation(
            self.fig,
            self.update_plot,
            interval=POLL_INTERVAL_MS,
            blit=False,
            cache_frame_data=False,
        )
        plt.tight_layout()
        plt.show()


def main():
    data_file = DEFAULT_DATA_FILE
    plot_time_step_s = PLOT_TIME_STEP_S

    args = sys.argv[1:]
    i = 0
    while i < len(args):
        if args[i] == "--plot-step" and i + 1 < len(args):
            try:
                plot_time_step_s = float(args[i + 1])
            except ValueError:
                plot_time_step_s = PLOT_TIME_STEP_S
            i += 2
        else:
            data_file = Path(args[i])
            i += 1

    plotter = RealtimeNETrajectory(data_file, plot_time_step_s=plot_time_step_s)
    plotter.run()


if __name__ == "__main__":
    main()
