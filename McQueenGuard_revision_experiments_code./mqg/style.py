"""Figure style matching the existing manuscript figures (spreadsheet-style charts)."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

BLUE, RED, YELLOW, GREEN, ORANGE, TEAL = "#4285F4", "#EA4335", "#FBBC04", "#34A853", "#FF6D01", "#46BDC6"
LBLUE, GRAY, PURPLE = "#A1C2FA", "#9E9E9E", "#8E63CE"

METHOD_STYLE = {
    "McQueenGuard": dict(label="McQueenGuard", color=BLUE),
    "McQueenGuard (exact only)": dict(label="McQueenGuard (exact only)", color=LBLUE),
    "McQueenGuard w/o admission (ablation)": dict(label="Learned w/o admission (ablation)", color=GRAY),
    "VerifAI runtime assurance": dict(label="VerifAI RA", color=ORANGE),
    "MPC (Zhang et al. 2022)": dict(label="MPC", color=GREEN),
    "Rule-based (IDM + AEB)": dict(label="Rule-based (IDM+AEB)", color=YELLOW),
    "End-to-end RL (PPO)": dict(label="End-to-end RL (PPO)", color=RED),
    "McQueenGuard (static bound)": dict(label="McQueenGuard (static bound)", color=BLUE),
    "McQueenGuard (online recalibration)": dict(label="McQueenGuard (online recalibration)", color=PURPLE),
}


def setup():
    fams = {f.name for f in font_manager.fontManager.ttflist}
    family = "Liberation Sans" if "Liberation Sans" in fams else "DejaVu Sans"
    plt.rcParams.update({
        "font.family": family, "font.size": 8.5, "axes.titlesize": 9, "axes.labelsize": 8.5,
        "xtick.labelsize": 8, "ytick.labelsize": 8, "legend.fontsize": 7.5,
        "axes.edgecolor": "#333333", "axes.linewidth": 0.8, "axes.spines.top": False,
        "axes.spines.right": False, "axes.grid": True, "axes.grid.axis": "y", "grid.color": "#D9D9D9",
        "grid.linewidth": 0.7, "axes.axisbelow": True, "legend.frameon": False,
        "savefig.dpi": 300, "savefig.bbox": "tight", "pdf.fonttype": 42, "ps.fonttype": 42,
        "xtick.direction": "out", "ytick.direction": "out", "lines.linewidth": 1.4, "lines.markersize": 4,
    })


def top_legend(ax, ncol=4, **kw):
    h, l = ax.get_legend_handles_labels()
    return ax.legend(h, l, loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=ncol, handlelength=1.2,
                     columnspacing=1.2, borderaxespad=0.3, **kw)
