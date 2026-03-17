
from __future__ import annotations


def setup_dark_mpl() -> None:
    import matplotlib as mpl

    mpl.rcParams.update({
        "figure.facecolor": "#0b0b0b",
        "axes.facecolor": "#0b0b0b",
        "axes.edgecolor": "#666666",
        "axes.labelcolor": "#d8d8d8",
        "axes.titlecolor": "#ffffff",
        "xtick.color": "#c8c8c8",
        "ytick.color": "#c8c8c8",
        "grid.color": "#5a5a5a",
        "grid.alpha": 0.22,
        "text.color": "#ffffff",
        "legend.facecolor": "#141414",
        "legend.edgecolor": "#444444",
        "savefig.facecolor": "#0b0b0b",
        "savefig.edgecolor": "#0b0b0b",
        "font.size": 7,
        "axes.titlesize": 8,
        "axes.labelsize": 7,
        "xtick.labelsize": 6,
        "ytick.labelsize": 6,
        "legend.fontsize": 6,
        "legend.title_fontsize": 6,
        "axes.titlepad": 2.0,
        "axes.labelpad": 1.5,
        "xtick.major.pad": 1.0,
        "ytick.major.pad": 1.0,
        "toolbar": "toolbar2",
    })