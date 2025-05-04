"""
Script for plotting the step response of a mass-spring-damper system under PID
control
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

import pathlib

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def main():
    crate_root = pathlib.Path(__file__).parent / ".."
    output_dir = crate_root / "output"
    df = pd.read_csv(output_dir / "step_response.csv")
    fig, ax = plt.subplots()

    ax.plot(df["time"], df["p"], label="Measurement")
    ax.plot(df["time"], df["sp"], label="Setpoint")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")

    FINAL_VALUE = 1
    MAG = 1

    ax.fill_between(
        df["time"],
        (FINAL_VALUE + MAG * 0.02),
        (FINAL_VALUE - MAG * 0.02),
        color="C4",
        alpha=0.5,
        label="Envelope for 2% settling time",
    )

    ax.axhline(
        FINAL_VALUE + MAG * 0.2,
        color="r",
        linestyle="--",
        label="20% Overshoot Goal",
    )

    ax.set_ylim(top=2.0)
    ax.legend(loc="upper right")
    ax.grid()
    plt.show()
    fig.savefig(output_dir / "step_response.png", bbox_inches="tight")


if __name__ == "__main__":
    main()
