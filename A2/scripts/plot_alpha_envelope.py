"""Generate the vector PDF and PNG preview used in Part A, Question 2."""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    output = Path(__file__).resolve().parents[1] / "figures"
    output.mkdir(parents=True, exist_ok=True)

    plt.rcParams.update({
        "font.family": "serif",
        "mathtext.fontset": "cm",
        "font.size": 10,
        "axes.labelsize": 11,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.linewidth": 0.7,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    })

    p = np.linspace(0, 1, 501)
    values = np.array([15*p - 5, 10 - 15*p, 4 + 0*p, 2 - p, 1 + 4.5*p])
    colors = ["#3476A8", "#CE842E", "#378B78", "#9A76A4", "#BF5952"]
    labels = [
        r"$V_1=15p-5$", r"$V_2=10-15p$", r"$V_3=4$",
        r"$V_4=2-p$", r"$V_5=1+4.5p$",
    ]
    dark = "#172E43"

    fig, ax = plt.subplots(figsize=(6.6, 3.7))
    fig.subplots_adjust(left=0.10, right=0.98, top=0.96, bottom=0.29)
    ax.set_axisbelow(True)
    ax.grid(axis="y", color="#E4E8EC", linewidth=0.6)

    for i, (value, color, label) in enumerate(zip(values, colors, labels)):
        ax.plot(p, value, color=color, linewidth=1.25,
                linestyle="--" if i >= 3 else "-", label=label, zorder=2)

    ax.plot(p, values.max(axis=0), color=dark, linewidth=2.8,
            label=r"$V(p)=\max_i V_i(p)$", zorder=4)
    ax.vlines([0.4, 0.6], -5.5, 4, color="#A0A9B2",
              linestyle=(0, (2, 3)), linewidth=0.85, zorder=1)
    ax.scatter([0.4, 0.6], [4, 4], s=25, facecolor="white",
               edgecolor=dark, linewidth=1.3, zorder=5)

    for x, y, label in [(0.16, 8.4, r"$\alpha_2$"),
                        (0.50, 4.8, r"$\alpha_3$"),
                        (0.84, 8.4, r"$\alpha_1$")]:
        ax.text(x, y, label, color=dark, ha="center", fontsize=11)

    ax.set(xlim=(0, 1), ylim=(-5.5, 10.7),
           xlabel=r"Belief $p=b(s_1)$", ylabel=r"Value $\alpha_i\cdot b$")
    ax.set_xticks([0, 0.2, 0.4, 0.6, 0.8, 1],
                  ["0", r"$\frac{1}{5}$", r"$\frac{2}{5}$",
                   r"$\frac{3}{5}$", r"$\frac{4}{5}$", "1"])
    ax.set_yticks([-5, 0, 4, 10])
    ax.tick_params(width=0.7, length=3)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.25), ncol=3,
              frameon=False, fontsize=9, handlelength=2.2,
              columnspacing=1.7, handletextpad=0.6)

    for extension in ("pdf", "png"):
        fig.savefig(output / f"alpha_envelope.{extension}",
                    bbox_inches="tight", pad_inches=0.05, dpi=220)
    plt.close(fig)


if __name__ == "__main__":
    main()
