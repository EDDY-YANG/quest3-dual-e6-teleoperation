"""Draw the source-verified, fixed-up X-only calibration construction."""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

ROOT = Path(__file__).resolve().parents[1]
plt.rcParams.update({'font.family': 'DejaVu Sans', 'svg.fonttype': 'none'})
fig, ax = plt.subplots(figsize=(7.8, 5.0), dpi=180)
fig.patch.set_facecolor('white')
ax.set(xlim=(0, 7.8), ylim=(0, 5.0))
ax.axis('off')
ink, muted, blue, orange, purple = '#172331', '#627183', '#2563eb', '#d97706', '#7c3aed'

def arrow(start, end, color, width=2):
    ax.add_patch(FancyArrowPatch(start, end, arrowstyle='-|>', mutation_scale=13,
                               color=color, linewidth=width))

ax.text(.25, 4.7, 'One sampled axis. One fixed vertical reference.', fontsize=13, color=ink, weight='bold')
ax.text(.25, 4.34, 'A yaw calibration for level E6 bases', fontsize=10, color=muted)
origin = (1.9, 2.35)
ax.plot([.5, 3.55], [2.02, 2.02], color='#dce2e9', linewidth=1)
arrow(origin, (3.55, 2.7), blue)
arrow(origin, (.9, 1.67), orange)
arrow(origin, (1.9, 3.9), purple)
ax.scatter(*origin, color=ink, s=20, zorder=4)
ax.text(3.0, 3.0, 'x: sampled +X', fontsize=10, color=blue)
ax.text(.2, 1.28, 'y = normalize(u × x)', fontsize=10, color=orange)
ax.text(2.03, 3.8, 'u: WebXR up', fontsize=10, color=purple)
ax.text(1.53, 2.14, 'p₀', fontsize=11, color=ink)
ax.text(3.55, 2.48, 'p₁', fontsize=11, color=ink)
ax.text(.5, .85, 'Project the recorded motion onto the horizontal plane.', fontsize=9, color=muted)
ax.text(.5, .52, 'Capture at least 80 mm; keep the robot stationary.', fontsize=9, color=muted)
ax.plot([4.1, 4.1], [.6, 4.05], color='#dce2e9', linewidth=1)
ax.text(4.55, 3.83, 'Per-arm direction map', fontsize=12, color=ink, weight='bold')
ax.text(4.55, 3.32, 'C = [ xᵀ ; yᵀ ; uᵀ ]', fontsize=13, color=ink)
for y, left, right, color in [(2.78, 'sampled x', 'base +X', blue), (2.31, 'derived y', 'base +Y', orange), (1.84, 'fixed up u', 'base +Z', purple)]:
    ax.text(4.55, y, left, fontsize=10, color=color)
    arrow((5.75, y+.04), (6.18, y+.04), color, 1.3)
    ax.text(6.32, y, right, fontsize=10, color=color)
ax.text(4.55, 1.15, 'At each Grip engagement:', fontsize=10, color=ink)
ax.text(4.55, .76, 'Δp_TCP = 1000 C Δp_hand', fontsize=11, color=ink)
ax.text(4.55, .4, 'metres → millimetres · anchored target', fontsize=9, color=muted)
fig.subplots_adjust(0, 0, 1, 1)
for ext in ('svg', 'png'):
    fig.savefig(ROOT / f'assets/calibration.{ext}', facecolor='white', dpi=180)
plt.close(fig)
print('Rendered the calibration schematic (not measured motion).')
