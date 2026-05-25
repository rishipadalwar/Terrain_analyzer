import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.ticker import MaxNLocator
from scipy.ndimage import gaussian_filter1d
import sys
import os


SAMPLE_CSV = "TERRAIN.CSV"

def make_fake_data(n=120):
    np.random.seed(42)
    t = np.arange(n) * 500

    base_lat  = 18.5204
    base_lon  = 73.8567
    lats      = base_lat  + np.cumsum(np.random.randn(n) * 0.00005)
    lons      = base_lon  + np.cumsum(np.random.randn(n) * 0.00005)
    alt       = 580 + np.cumsum(np.random.randn(n) * 0.3)

    hill      = 150 + 60 * np.sin(np.linspace(0, 3*np.pi, n)) + np.random.randn(n) * 4
    strength  = 800 + np.random.randint(-50, 50, n)
    temp      = 28  + np.random.randn(n) * 1.5

    dy        = np.diff(hill, prepend=hill[0]) / 100.0
    dx_m      = np.sqrt(np.diff(lats, prepend=lats[0])**2 +
                        np.diff(lons, prepend=lons[0])**2) * 111320
    dx_m[dx_m < 0.01] = 0.01
    slope     = np.degrees(np.arctan2(np.abs(dy), dx_m))

    return pd.DataFrame({
        "idx":             np.arange(n),
        "timestamp_ms":    t,
        "latitude":        lats,
        "longitude":       lons,
        "alt_gps_m":       alt,
        "lidar_dist_cm":   hill,
        "lidar_strength":  strength,
        "lidar_temp_c":    temp,
        "slope_deg":       slope,
        "gps_valid":       1,
        "lidar_valid":     1,
    })


def load(path):
    if os.path.exists(path):
        raw = pd.read_csv(path)
        raw.columns = raw.columns.str.strip()
        return raw
    print(f"'{path}' not found — running with demo data.")
    return make_fake_data()


def smooth(arr, sigma=2):
    return gaussian_filter1d(arr.astype(float), sigma=sigma)


def terrain_colormap():
    colors = ["#0a4a6e", "#1a7a4a", "#8bc34a", "#cddc39", "#c8a84b", "#a0522d", "#ffffff"]
    return LinearSegmentedColormap.from_list("terrain", colors, N=256)


def setup_style():
    plt.rcParams.update({
        "figure.facecolor":  "#0d1117",
        "axes.facecolor":    "#161b22",
        "axes.edgecolor":    "#30363d",
        "axes.labelcolor":   "#c9d1d9",
        "axes.titlecolor":   "#f0f6fc",
        "xtick.color":       "#8b949e",
        "ytick.color":       "#8b949e",
        "grid.color":        "#21262d",
        "grid.linewidth":    0.6,
        "text.color":        "#c9d1d9",
        "font.family":       "monospace",
        "axes.spines.top":   False,
        "axes.spines.right": False,
    })


def annotate_stats(ax, values, unit="", color="#58a6ff"):
    mn, mx, av = values.min(), values.max(), values.mean()
    text = f"min {mn:.1f}{unit}   max {mx:.1f}{unit}   avg {av:.1f}{unit}"
    ax.text(0.98, 0.96, text, transform=ax.transAxes,
            ha="right", va="top", fontsize=7.5,
            color=color, alpha=0.85,
            bbox=dict(boxstyle="round,pad=0.3", facecolor="#0d1117", edgecolor="none", alpha=0.6))


def plot(df):
    setup_style()

    df = df[df["gps_valid"] == 1].copy().reset_index(drop=True)
    t  = df["timestamp_ms"].values / 1000.0

    fig = plt.figure(figsize=(18, 13), facecolor="#0d1117")
    fig.suptitle("TERRAIN ANALYZER — Field Data Dashboard",
                 fontsize=16, fontweight="bold", color="#f0f6fc",
                 y=0.97, fontfamily="monospace")

    gs = gridspec.GridSpec(3, 3, figure=fig,
                           hspace=0.52, wspace=0.38,
                           left=0.06, right=0.97,
                           top=0.92, bottom=0.06)

    valid_lidar = df[df["lidar_valid"] == 1]
    vt          = valid_lidar["timestamp_ms"].values / 1000.0

    ax_dist = fig.add_subplot(gs[0, :2])
    raw_d   = valid_lidar["lidar_dist_cm"].values
    sm_d    = smooth(raw_d)
    ax_dist.fill_between(vt, sm_d, alpha=0.18, color="#58a6ff")
    ax_dist.plot(vt, raw_d, lw=0.8, color="#58a6ff", alpha=0.45, label="raw")
    ax_dist.plot(vt, sm_d,  lw=2.0, color="#58a6ff", label="smoothed")
    ax_dist.set_title("Surface Distance (LiDAR)", fontsize=10)
    ax_dist.set_ylabel("cm")
    ax_dist.set_xlabel("time (s)")
    ax_dist.legend(fontsize=8, loc="upper right",
                   facecolor="#161b22", edgecolor="#30363d", labelcolor="#c9d1d9")
    ax_dist.grid(True)
    annotate_stats(ax_dist, raw_d, "cm")

    ax_slope = fig.add_subplot(gs[0, 2])
    slope    = df["slope_deg"].values
    colors_s = ["#3fb950" if s < 10 else "#d29922" if s < 25 else "#f85149" for s in slope]
    ax_slope.bar(t, slope, width=(t[1]-t[0])*0.85 if len(t) > 1 else 1,
                 color=colors_s, alpha=0.85)
    ax_slope.set_title("Slope Angle", fontsize=10)
    ax_slope.set_ylabel("degrees")
    ax_slope.set_xlabel("time (s)")
    ax_slope.grid(True, axis="y")
    ax_slope.yaxis.set_major_locator(MaxNLocator(integer=True))
    from matplotlib.patches import Patch
    ax_slope.legend(handles=[
        Patch(facecolor="#3fb950", label="gentle <10°"),
        Patch(facecolor="#d29922", label="moderate 10–25°"),
        Patch(facecolor="#f85149", label="steep >25°"),
    ], fontsize=7, loc="upper right",
       facecolor="#161b22", edgecolor="#30363d", labelcolor="#c9d1d9")

    ax_alt = fig.add_subplot(gs[1, :2])
    alt    = df["alt_gps_m"].values
    sm_alt = smooth(alt)
    ax_alt.fill_between(t, sm_alt, alt.min(), alpha=0.12, color="#3fb950")
    ax_alt.plot(t, alt,    lw=0.7, color="#3fb950", alpha=0.4)
    ax_alt.plot(t, sm_alt, lw=2.0, color="#3fb950")
    ax_alt.set_title("GPS Altitude", fontsize=10)
    ax_alt.set_ylabel("metres")
    ax_alt.set_xlabel("time (s)")
    ax_alt.grid(True)
    annotate_stats(ax_alt, alt, "m", "#3fb950")

    ax_sig = fig.add_subplot(gs[1, 2])
    sig    = valid_lidar["lidar_strength"].values
    ax_sig.plot(vt, sig, lw=1.2, color="#d2a8ff", alpha=0.7)
    ax_sig.fill_between(vt, sig, sig.min(), alpha=0.12, color="#d2a8ff")
    ax_sig.axhline(y=100, color="#f85149", lw=1.2, ls="--", label="min threshold")
    ax_sig.set_title("LiDAR Signal Strength", fontsize=10)
    ax_sig.set_ylabel("strength")
    ax_sig.set_xlabel("time (s)")
    ax_sig.legend(fontsize=7.5, facecolor="#161b22",
                  edgecolor="#30363d", labelcolor="#c9d1d9")
    ax_sig.grid(True)

    ax_map = fig.add_subplot(gs[2, :2])
    lats   = df["latitude"].values
    lons   = df["longitude"].values
    dist_c = valid_lidar["lidar_dist_cm"].values if len(valid_lidar) == len(df) else df["lidar_dist_cm"].values
    sc     = ax_map.scatter(lons, lats, c=dist_c, cmap=terrain_colormap(),
                            s=12, alpha=0.85, edgecolors="none")
    ax_map.plot(lons, lats, lw=0.6, color="#8b949e", alpha=0.4, zorder=0)
    ax_map.scatter(lons[0],  lats[0],  s=80, color="#3fb950", zorder=5, label="start")
    ax_map.scatter(lons[-1], lats[-1], s=80, color="#f85149", zorder=5, label="end", marker="^")
    cb = plt.colorbar(sc, ax=ax_map, pad=0.01)
    cb.set_label("dist (cm)", color="#c9d1d9", fontsize=8)
    cb.ax.yaxis.set_tick_params(color="#8b949e")
    plt.setp(cb.ax.yaxis.get_ticklabels(), color="#8b949e", fontsize=7)
    ax_map.set_title("GPS Track — Coloured by Surface Distance", fontsize=10)
    ax_map.set_xlabel("longitude")
    ax_map.set_ylabel("latitude")
    ax_map.legend(fontsize=8, facecolor="#161b22",
                  edgecolor="#30363d", labelcolor="#c9d1d9")
    ax_map.grid(True)

    ax_temp = fig.add_subplot(gs[2, 2])
    temp    = valid_lidar["lidar_temp_c"].values
    ax_temp.plot(vt, temp, lw=1.5, color="#ffa657", alpha=0.9)
    ax_temp.fill_between(vt, temp, temp.min(), alpha=0.15, color="#ffa657")
    ax_temp.set_title("LiDAR Sensor Temperature", fontsize=10)
    ax_temp.set_ylabel("°C")
    ax_temp.set_xlabel("time (s)")
    ax_temp.grid(True)
    annotate_stats(ax_temp, temp, "°C", "#ffa657")

    total_pts  = len(df)
    gps_ok     = int(df["gps_valid"].sum())
    lidar_ok   = int(df["lidar_valid"].sum())
    dur_s      = (t[-1] - t[0]) if len(t) > 1 else 0
    dist_total = np.sum(np.sqrt(np.diff(lats)**2 + np.diff(lons)**2)) * 111320

    summary = (f"  Points: {total_pts}   GPS valid: {gps_ok}   LiDAR valid: {lidar_ok}"
               f"   Duration: {dur_s:.1f}s   Track length ≈ {dist_total:.1f}m  ")
    fig.text(0.5, 0.003, summary, ha="center", fontsize=8,
             color="#8b949e", fontfamily="monospace",
             bbox=dict(boxstyle="round,pad=0.3", facecolor="#161b22",
                       edgecolor="#30363d", alpha=0.8))

    out = "terrain_dashboard.png"
    plt.savefig(out, dpi=150, bbox_inches="tight", facecolor="#0d1117")
    print(f"Saved → {out}")
    plt.show()


if __name__ == "__main__":
    csv_path = sys.argv[1] if len(sys.argv) > 1 else SAMPLE_CSV
    data     = load(csv_path)
    plot(data)
