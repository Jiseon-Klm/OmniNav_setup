import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as mcolors
import numpy as np
import math
import csv

input_file = '../data/OmniNav/log/waypoint_data_Sample_data.csv'
output_file = '../data/OmniNav/log/result4.png'

# Read CSV
data = []
with open(input_file, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        data.append({
            'frame_idx': int(row['frame_idx']),
            'subframe_idx': int(row['subframe_idx']),
            'dx': float(row['dx']),
            'dy': float(row['dy']),
            'dtheta': float(row['dtheta']),
            'arrive': int(row['arrive'])
        })

subframe_to_use = 0
filtered_data = [d for d in data if d['subframe_idx'] == subframe_to_use]

Gx, Gy, Gtheta = 0.0, 0.0, 0.0
trajectory = [(Gx, Gy, Gtheta)]

for record in filtered_data:
    dx = record['dx']
    dy = record['dy']
    dtheta_deg = record['dtheta']
    
    # SWAPPED sin/cos version
    delta_gx = dx * math.cos(Gtheta) - dy * math.sin(Gtheta)
    delta_gy = dx * math.sin(Gtheta) + dy * math.cos(Gtheta)
    
    Gx += delta_gx
    Gy += delta_gy
    Gtheta += math.radians(dtheta_deg)
    
    trajectory.append((Gx, Gy, Gtheta))

# Plot
fig, ax = plt.subplots(figsize=(14, 12))

x_coords = [p[0] for p in trajectory]
y_coords = [p[1] for p in trajectory]
n_points = len(trajectory)

cmap = cm.jet
norm = mcolors.Normalize(vmin=0, vmax=n_points - 1)

for i in range(n_points - 1):
    color = cmap(norm(i))
    ax.plot(x_coords[i:i+2], y_coords[i:i+2], color=color, linewidth=2)

arrow_freq = 2
arrow_scale = 0.03
for i in range(0, len(trajectory), arrow_freq):
    x, y, theta = trajectory[i]
    dx_arrow = arrow_scale * math.sin(theta)
    dy_arrow = arrow_scale * math.cos(theta)
    color = cmap(norm(i))
    ax.arrow(x, y, dx_arrow, dy_arrow, head_width=0.5, head_length=0.5, 
              fc=color, ec=color, alpha=0.8)

ax.scatter(0, 0, color='green', s=150, marker='*', label='Start', zorder=5)
ax.scatter(x_coords[-1], y_coords[-1], color='red', s=150, marker='o', label='End', zorder=5)

sm = cm.ScalarMappable(cmap=cmap, norm=norm)
sm.set_array([])
cbar = plt.colorbar(sm, ax=ax, label='Time Step (Frame Index)')

ax.set_title(f'Trajectory (SWAPPED sin/cos) - 063a442aee (Subframe {subframe_to_use})')
ax.set_xlabel('Global X (m)')
ax.set_ylabel('Global Y (m)')
ax.legend()
ax.grid(True)
ax.set_aspect('equal')

plt.savefig(output_file, dpi=150)
print(f"Trajectory saved to {output_file}")