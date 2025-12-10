#!/usr/bin/env python3
"""
Plot trajectories from a GeoScenario Server trajectory CSV.

Default CSV header expected (example):
id,type,sim_state,tick_count,sim_time,delta_time,x,x_vel,x_acc,y,y_vel,y_acc,s,s_vel,s_acc,d,d_vel,d_acc,angle

Plots produced:
- x_vel vs sim_time
- y_vel vs sim_time
- displacement vs sim_time (displacement computed by integrating speed magnitude = sqrt(x_vel^2 + y_vel^2) * delta_time)
- s_vel vs sim_time

Usage:
    python plot_trajectories.py [path/to/trajectory.csv] [--id VID] [--out OUTPUT.png] [--show]

If --id is not provided the script will pick the first vehicle id found in the file.
"""
import sys
import os
import math
import argparse

try:
    import pandas as pd
except Exception:
    pd = None

import csv
import numpy as np
import matplotlib.pyplot as plt


def read_csv(filepath):
    """Read CSV into a dict of dataframes keyed by vehicle id."""
    if pd is not None:
        df = pd.read_csv(filepath)
        # ensure required columns exist
        required = ['id', 'sim_time', 'delta_time', 'x_vel', 'y_vel', 'x', 'y']
        for c in required:
            if c not in df.columns:
                raise ValueError(f"Required column '{c}' not found in CSV")

        groups = {}
        for vid, g in df.groupby('id'):
            groups[int(vid)] = g.sort_values('sim_time').reset_index(drop=True)
        return groups

    # fallback to csv module
    groups = {}
    with open(filepath, newline='') as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            vid = int(row['id'])
            if vid not in groups:
                groups[vid] = {k: [] for k in reader.fieldnames}
            for k, v in row.items():
                groups[vid][k].append(v)

    # convert to numpy structured dicts
    out = {}
    for vid, data in groups.items():
        # convert strings to floats where appropriate
        sim_time = np.array([float(x) for x in data['sim_time']])
        delta_time = np.array([float(x) for x in data['delta_time']])
        x_vel = np.array([float(x) for x in data['x_vel']])
        y_vel = np.array([float(x) for x in data['y_vel']])
        x = np.array([float(x) for x in data.get('x', [0]*len(sim_time))])
        y = np.array([float(x) for x in data.get('y', [0]*len(sim_time))])
        # optional tick_count if present
        tick_count = np.array([int(x) for x in data.get('tick_count', list(range(1, len(sim_time)+1)))])
        out[vid] = {
            'sim_time': sim_time,
            'delta_time': delta_time,
            'x_vel': x_vel,
            'y_vel': y_vel,
            'x': x,
            'y': y,
            'tick_count': tick_count,
        }
    return out


def compute_displacement(x_vel, y_vel, delta_time):
    """Compute cumulative x and y displacements by integrating velocities over time.

    Returns (x_disp, y_disp) where each is the cumulative sum of velocity * delta_time.
    """
    x_vel = np.array(x_vel)
    y_vel = np.array(y_vel)
    dt = np.array(delta_time)
    # incremental displacements
    inc_x = x_vel * dt
    inc_y = y_vel * dt
    x_disp = np.cumsum(inc_x)
    y_disp = np.cumsum(inc_y)
    return x_disp, y_disp


def plot_for_vehicle(vid, df, outpath=None, show=False):
    # df may be a pandas DataFrame or a dict of numpy arrays
    if pd is not None and isinstance(df, pd.DataFrame):
        sim_time = df['sim_time'].to_numpy()
        delta_time = df['delta_time'].to_numpy()
        x_vel = df['x_vel'].to_numpy()
        y_vel = df['y_vel'].to_numpy()
        x = df['x'].to_numpy()
        y = df['y'].to_numpy()
    else:
        sim_time = np.array(df['sim_time'])
        delta_time = np.array(df['delta_time'])
        x_vel = np.array(df['x_vel'])
        y_vel = np.array(df['y_vel'])
        x = np.array(df['x'])
        y = np.array(df['y'])

    x_disp, y_disp = compute_displacement(x_vel, y_vel, delta_time)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    ax = axes.ravel()

    ax[0].plot(sim_time, x_vel, label='x_vel')
    ax[0].set_xlabel('sim_time [s]')
    ax[0].set_ylabel('x_vel [m/s]')
    ax[0].grid(True)

    ax[1].plot(sim_time, y_vel, label='y_vel', color='tab:orange')
    ax[1].set_xlabel('sim_time [s]')
    ax[1].set_ylabel('y_vel [m/s]')
    ax[1].grid(True)

    # plot x and y cumulative displacements computed from velocities
    ax[2].plot(sim_time, x_disp, label='x_disp', color='tab:green')
    ax[2].plot(sim_time, y_disp, label='y_disp', color='tab:blue')
    ax[2].set_xlabel('sim_time [s]')
    ax[2].set_ylabel('displacement [m]')
    ax[2].legend()
    ax[2].grid(True)

    ax[3].plot(sim_time, x, label='x pos', color='tab:red')
    ax[3].plot(sim_time, y, label='y pos', color='tab:purple')
    ax[3].set_xlabel('sim_time [s]')
    ax[3].set_ylabel('position [m]')
    ax[3].legend()
    ax[3].grid(True)

    fig.suptitle(f'Vehicle {vid} velocities and displacement')
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    # If an explicit output path was given, save the figure. Otherwise just display it.
    # Prepare second figure: compare integrated displacement (from velocities)
    # with displacement derived directly from CSV positions (x - x0, y - y0)
    # position-derived displacements
    # position-derived cumulative displacements (from CSV positions)
    x_pos_disp = x - x[0] if len(x) > 0 else np.zeros_like(x)
    y_pos_disp = y - y[0] if len(y) > 0 else np.zeros_like(y)

    # per-tick incremental displacements computed from velocities (inc = v * dt)
    inc_x = x_vel * delta_time
    inc_y = y_vel * delta_time
    # per-tick position differences from CSV
    pos_dx = np.concatenate(([0.0], np.diff(x)))
    pos_dy = np.concatenate(([0.0], np.diff(y)))

    # determine tick indices (prefer explicit tick_count if available)
    if pd is not None and isinstance(df, pd.DataFrame) and 'tick_count' in df.columns:
        ticks = df['tick_count'].to_numpy()
    elif isinstance(df, dict) and 'tick_count' in df:
        ticks = np.array(df['tick_count'])
    else:
        ticks = np.arange(len(sim_time))

    fig2, axs2 = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    
    axs2[0].plot(ticks, pos_dx, label='pos_dx (CSV)', color='tab:red', linestyle='--', marker='o', markersize=3)
    axs2[0].plot(ticks, inc_x, label='inc_x (vel*dt)', color='tab:green', alpha=0.6)
    axs2[0].set_ylabel('x delta per tick [m]')
    axs2[0].legend()
    axs2[0].grid(True)

    axs2[1].plot(ticks, inc_y, label='inc_y (vel*dt)', color='tab:blue', alpha=0.6)
    axs2[1].plot(ticks, pos_dy, label='pos_dy (CSV)', color='tab:purple', linestyle='--', marker='o', markersize=3)
    axs2[1].set_xlabel('tick')
    axs2[1].set_ylabel('y delta per tick [m]')
    axs2[1].legend()
    axs2[1].grid(True)

    fig2.suptitle(f'Vehicle {vid} displacement: integrated vs CSV positions')
    fig2.tight_layout(rect=[0, 0.03, 1, 0.95])

    # If an explicit output path was given, save both figures. Otherwise just display them.
    if outpath:
        # save main figure
        fig.savefig(outpath)
        # save comparison figure with suffix
        base, ext = os.path.splitext(outpath)
        out2 = f"{base}_disp_compare{ext or '.png'}"
        fig2.savefig(out2)
        print(f'Plots saved to {outpath} and {out2}')

    # Display the plots (show by default)
    try:
        plt.show()
    except Exception:
        # In environments without a display, fallback to printing a message
        if not outpath:
            print('No display available and no output path provided; plots not shown or saved.', file=sys.stderr)
    finally:
        plt.close(fig)
        plt.close(fig2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('csv', nargs='?', default=os.path.join('outputs', 'trajectory_v1.csv'), help='trajectory CSV file')
    parser.add_argument('--id', type=int, help='vehicle id to plot (default: first id found)')
    parser.add_argument('--out', help='output PNG file (optional)')
    parser.add_argument('--show', action='store_true', help='show plot interactively')
    args = parser.parse_args()

    if not os.path.exists(args.csv):
        print(f'CSV file not found: {args.csv}', file=sys.stderr)
        sys.exit(2)

    groups = read_csv(args.csv)
    if len(groups) == 0:
        print('No vehicle data found in CSV', file=sys.stderr)
        sys.exit(2)

    vid = args.id if args.id is not None else sorted(groups.keys())[0]
    if vid not in groups:
        print(f'Vehicle id {vid} not found in CSV', file=sys.stderr)
        sys.exit(2)

    df = groups[vid]
    # By default do not save a PNG; just display. If --out is provided, save to that path.
    outpath = args.out if args.out else None
    plot_for_vehicle(vid, df, outpath=outpath, show=args.show)


if __name__ == '__main__':
    main()
