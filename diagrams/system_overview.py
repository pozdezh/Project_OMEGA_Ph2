"""System overview schematic for the memo.

A hand-placed block diagram sized for one memo page: portrait, 6.1 in wide,
so that at \\textwidth the type renders close to its designed point size.
Every box and every connector is set by explicit coordinates.

Rules that keep it readable:
  * Each box carries ONLY its bold name. The detail that used to sit inside
    the boxes lives in the figure caption and the chapter prose, so the
    picture stays a structural map, not a spec sheet.
  * Every link is axis-aligned - it shares either its x or its y with both
    endpoints. No diagonals. Links run in a few fixed columns so none cross.
  * Every arrowhead is one solid filled triangle at one size, drawn as its
    own short segment, so a dotted internal link is capped exactly like a
    solid telemetry link.

    py -3.12 system_overview.py

Outputs to out/ (png transparent, png on white, svg, pdf) and copies the pdf
to memo/figures/fig_system_overview.pdf. Only needs matplotlib.
"""

import pathlib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

plt.rcParams["font.family"] = "DejaVu Sans"

OUT = pathlib.Path(__file__).resolve().parent / "out"

FS_HEAD = 8.6      # the two section headings
FS_TITLE = 8.2     # bold box names
FS_TAG = 7.2       # the writes / reads arrow labels
FS_KEY = 7.1       # legend rows

HEAD = 11.0        # one size for every arrowhead
INK = "#1A1A1A"
SLATE = "#3A4453"

# -- palette: style -> (edge, fill) -----------------------------------
C = {
    "dev":   ("#2848A8", "#E9EDFA"),
    "telem": ("#123E8C", "#E6ECF8"),
    "data":  ("#8A6D1D", "#FBF3DF"),
    "web":   ("#7A3EA1", "#F2E9F8"),
    "op":    ("#B5620E", "#FBEEDD"),
    "ext":   ("#3F566B", "#EDF1F4"),
    "off":   ("#B0322B", "#FBE9E7"),
    "frame": ("#6B7280", "none"),
    "key":   ("#9AA1AC", "none"),
}

# -- line styles: colour, width, dash, double-headed -----------------
L = {
    "dtls":  ("#123E8C", 2.0, "solid",         True),
    "https": ("#7A3EA1", 2.0, "solid",         True),
    "wan":   ("#3F566B", 2.0, "solid",         False),
    "prov":  ("#B0322B", 1.7, (0, (4, 2.6)),   False),
    "local": ("#8A6D1D", 1.7, (0, (2.4, 2.2)), False),
}

# -- boxes: name -> (x0, y0, x1, y1, style, title) ------------------
BOXES = {
    "amu":  (0.20, 11.00, 5.75, 12.10, "dev",  "AMU 01 … AMU N  ·  air quality"),
    "nmu":  (6.25, 11.00, 11.80, 12.10, "dev", "NMU 01 … NMU N  ·  noise"),

    "ca":   (2.85, 9.60, 9.15, 10.55, "off",   "Certificate authority  (offline)"),

    "frame": (0.45, 4.20, 11.55, 9.15, "frame", ""),

    "telem": (0.80, 7.40, 11.20, 8.35, "telem", "Telemetry and discovery service"),
    "db":    (0.80, 5.95, 11.20, 6.90, "data",  "Database"),
    "web":   (0.80, 4.50, 5.90, 5.45, "web",    "Operator web plane"),
    "cron":  (6.10, 4.50, 11.20, 5.45, "web",   "Scheduled jobs  (cron)"),

    "op":   (0.20, 2.95, 5.55, 3.90, "op",   "Operator workstation"),
    "boss": (6.45, 2.95, 11.80, 3.90, "ext", "Research endpoint  (off-site)"),
}

# -- links: (a, b, style, label, label_xy, label_ha) ----------------
# a -> b is the direction data moves. Two-headed styles add a head at a too.
LINKS = [
    ((2.00, 11.00), (2.00, 9.15),   "dtls",  "", None, "center"),
    ((10.00, 11.00), (10.00, 9.15),  "dtls",  "", None, "center"),

    ((4.60, 10.55), (4.60, 11.00),  "prov",  "", None, "center"),
    ((7.40, 10.55), (7.40, 11.00),  "prov",  "", None, "center"),
    ((6.00, 9.60), (6.00, 9.15),    "prov",  "", None, "center"),

    ((3.00, 7.40), (3.00, 6.90),    "local", "writes",         (3.24, 7.15), "left"),
    ((3.00, 5.95), (3.00, 5.45),    "local", "reads",          (3.24, 5.70), "left"),
    ((8.50, 5.95), (8.50, 5.45),    "local", "reads, prunes",  (8.74, 5.70), "left"),

    ((3.00, 3.90), (3.00, 4.50),    "https", "", None, "center"),
    ((8.50, 4.50), (8.50, 3.90),    "wan",   "", None, "center"),
]

# -- legend rows: (style, text) ------------------------------------
KEY_ROWS = [
    ("dtls",  "DTLS 1.3 over UDP, mutual certificates  -  telemetry + ACK (5000)"),
    ("https", "Mutual-TLS over HTTPS  -  dashboard, REST API, MCP tools (443)"),
    ("wan",   "HTTPS + bearer token, server-authenticated only  -  nightly report"),
    ("prov",  "Offline provisioning by hand  -  CA cert and signed identities"),
    ("local", "Internal to the server  -  inter-process and database access"),
]

KEY_BOX = (0.15, 0.20, 11.85, 2.60)


def draw_box(ax, x0, y0, x1, y1, style, title):
    ec, fc = C[style]
    plain = style in ("frame", "key")
    ax.add_patch(FancyBboxPatch(
        (x0, y0), x1 - x0, y1 - y0,
        boxstyle="round,pad=0.02,rounding_size=0.10",
        linewidth=1.3 if plain else 1.9,
        edgecolor=ec, facecolor=fc,
        linestyle="--" if style == "off" else "solid",
        zorder=1 if plain else 2,
    ))
    if title:
        ax.text((x0 + x1) / 2, (y0 + y1) / 2, title, ha="center", va="center",
                fontsize=FS_TITLE, fontweight="bold", color=ec, zorder=4)


def _arrow(ax, a, b, colour, width, dash, double):
    """One connector. The shaft carries the dash; each head is its own
    short solid filled triangle so the dash pattern never touches it."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    dist = (dx * dx + dy * dy) ** 0.5 or 1.0
    ux, uy = dx / dist, dy / dist
    tip = 0.42

    shaft_a = (a[0] + tip * ux, a[1] + tip * uy) if double else a
    shaft_b = (b[0] - tip * ux, b[1] - tip * uy)
    ax.add_patch(FancyArrowPatch(
        shaft_a, shaft_b, arrowstyle="-", mutation_scale=HEAD,
        linewidth=width, linestyle=dash, color=colour,
        shrinkA=0, shrinkB=0, zorder=3))
    ax.add_patch(FancyArrowPatch(
        shaft_b, b, arrowstyle="-|>", mutation_scale=HEAD,
        linewidth=width, linestyle="solid", color=colour,
        shrinkA=0, shrinkB=0, zorder=3, joinstyle="miter"))
    if double:
        ax.add_patch(FancyArrowPatch(
            shaft_a, a, arrowstyle="-|>", mutation_scale=HEAD,
            linewidth=width, linestyle="solid", color=colour,
            shrinkA=0, shrinkB=0, zorder=3, joinstyle="miter"))


def draw_link(ax, a, b, style, label, label_xy, label_ha):
    colour, width, dash, double = L[style]
    _arrow(ax, a, b, colour, width, dash, double)
    if label:
        ax.text(label_xy[0], label_xy[1], label, ha=label_ha, va="center",
                fontsize=FS_TAG, style="italic", color=colour, zorder=5)


def draw_key(ax):
    x0, y0, x1, y1 = KEY_BOX
    draw_box(ax, x0, y0, x1, y1, "key", "")
    ax.text(0.45, 2.38, "KEY", fontsize=FS_HEAD, fontweight="bold",
            color=SLATE, ha="left", va="center")
    ax.text(1.32, 2.38,
            "double arrowhead = request / response,    single = one-way",
            ha="left", va="center", fontsize=FS_KEY, style="italic",
            color="#4A5666")
    y = 2.00
    for style, text in KEY_ROWS:
        colour, width, dash, double = L[style]
        _arrow(ax, (0.45, y), (1.62, y), colour, width, dash, double)
        ax.text(1.88, y, text, ha="left", va="center",
                fontsize=FS_KEY, color=INK)
        y -= 0.36


def build():
    fig, ax = plt.subplots(figsize=(6.1, 6.75))
    ax.set_xlim(-0.15, 12.15)
    ax.set_ylim(-0.15, 13.05)
    ax.set_aspect("equal")
    ax.axis("off")

    ax.text(6.00, 12.78, "FIELD UNITS", fontsize=FS_HEAD, fontweight="bold",
            color=SLATE, ha="center", va="center")
    ax.text(6.00, 8.90, "SYSTEM SERVER  -  Ubuntu mini-PC", fontsize=FS_HEAD,
            fontweight="bold", color=SLATE, ha="center", va="center")

    for x0, y0, x1, y1, style, title in BOXES.values():
        draw_box(ax, x0, y0, x1, y1, style, title)
    for a, b, style, label, lxy, lha in LINKS:
        draw_link(ax, a, b, style, label, lxy, lha)
    draw_key(ax)

    OUT.mkdir(exist_ok=True)
    for name, kw in (("system_overview.png", dict(dpi=300, transparent=True)),
                     ("system_overview_on_white.png", dict(dpi=300, facecolor="white")),
                     ("system_overview.svg", dict(transparent=True)),
                     ("system_overview.pdf", dict(transparent=True))):
        fig.savefig(OUT / name, bbox_inches="tight", pad_inches=0.04, **kw)
    plt.close(fig)

    memo_fig = (pathlib.Path(__file__).resolve().parents[2]
                / "memo" / "figures" / "fig_system_overview.pdf")
    if memo_fig.parent.is_dir():
        memo_fig.write_bytes((OUT / "system_overview.pdf").read_bytes())
        print("copied to", memo_fig)
    print("wrote out/system_overview.{png,svg,pdf} and _on_white.png")


if __name__ == "__main__":
    build()
