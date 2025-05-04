"""
Script for plotting the trajectory of a quadrotor under PID control.
Copyright © 2025 Hs293Go

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

import argparse
import pathlib
from math import sqrt

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import pandas as pd

plt.rcParams.update({"text.usetex": True, "font.family": "Times"})


def plot(targets, output, output_dir):
    fig, ax = plt.subplots(1, 1, subplot_kw={"projection": "3d"})

    ax.scatter(
        targets["x"],
        targets["y"],
        targets["z"],
        color="red",
        marker="x",
        label="Target",
    )
    ax.plot(output["x"], output["y"], output["z"], label="Quadrotor Trajectory")
    ax.set_xlim(-1, 6)
    ax.set_ylim(-1, 6)
    ax.set_zlim(0, 2)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend()
    plt.tight_layout()
    fig.savefig(output_dir / "quadrotor_trajectory.png", bbox_inches="tight")

    fig, axes = plt.subplots(nrows=3, ncols=2, figsize=(8, 4))

    fig.suptitle(
        r"Quadrotor Body Rate $(\omega_x,\omega_y,\omega_z)$ Tracking under PID control",
        fontsize=15,
    )

    for comp, disp_comp, ax in zip("pqr", "xyz", axes[:, 0]):
        ax.plot(output["time"], output[comp], "b", label="Measurement")
        ax.plot(output["time"], output[comp + "d"], "--r", label="Setpoint")
        ax.set_ylabel(rf"$\omega_{disp_comp}$ (rad/s)")
        ax.legend(fontsize=10, loc="upper right")
    for comp, disp_comp, ax in zip("pqr", "xyz", axes[:, 1]):
        abs_error = (output[comp] - output[comp + "d"]).abs()
        ax.plot(output["time"], abs_error, "k", label="Error")
        ax.set_ylabel(rf"$|\omega_{disp_comp}|$ (rad/s)")
        ax.set_ylim(0, 3)
        ax.legend(fontsize=10, loc="upper right")

    fig.supxlabel("Time (s)", fontsize=13)
    fig.savefig(output_dir / "quadrotor_control.png")


def animate(targets, output, output_dir, mode):
    # Setup figure and axis
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"}, figsize=(8, 8))
    ax.set_xlim(-1, 6)
    ax.set_ylim(-1, 6)
    ax.set_zlim(0, 5)

    ax.set_xlabel("X (m)", fontsize=15)
    ax.set_ylabel("Y (m)", fontsize=15)
    ax.set_zlabel("Y (m)", fontsize=15)

    # Plot target positions as static background
    ax.scatter(
        targets["x"],
        targets["y"],
        targets["z"],
        s=200,
        color="red",
        marker="x",
        label="Target",
    )
    (line,) = ax.plot([], [], lw=2, label="Quadrotor Trajectory")
    (dot,) = ax.plot([], [], "bo")  # current position

    # Animation update function
    def update(frame):
        current = output.iloc[:frame]
        line.set_data(current["x"], current["y"])
        line.set_3d_properties([current["z"]])
        if not current.empty:
            dot.set_data(
                [current["x"].iloc[-1]],
                [current["y"].iloc[-1]],
            )
            dot.set_3d_properties([current["z"].iloc[-1]])
        return line, dot

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=range(0, len(output), 4),  # higher downsampling to reduce size
        interval=16,  # slower frame rate
        blit=True,
        repeat=False,
    )
    plt.legend(fontsize=15)
    if mode == "show":
        plt.show()
    elif mode == "save":
        try:
            writer = animation.FFMpegWriter(fps=20, bitrate=800)
        except (AttributeError, RuntimeError):
            writer = "pillow"
        # Save with reduced resolution and FPS to keep GIF size small
        ani.save(
            output_dir / "quadrotor_animation.gif",
            dpi=60,
            writer=writer,
        )


def main():
    parser = argparse.ArgumentParser("Plot Quadrotor Trajectory")
    parser.add_argument(
        "--animate", choices=["", "show", "save"], help="Animate the trajectory"
    )
    args = parser.parse_args()
    crate_root = pathlib.Path(__file__).parent / ".."
    output_dir = crate_root / "output"
    df = pd.read_csv(output_dir / "quadrotor_trajectory.csv")

    targets = df[df["type"] == "target"]
    trajectory = df[df["type"] == "output"]

    plot(targets, trajectory, output_dir)

    if args.animate:
        animate(targets, trajectory, output_dir, args.animate)
    else:
        plt.show()


if __name__ == "__main__":
    main()
