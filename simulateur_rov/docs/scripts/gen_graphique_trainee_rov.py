"""
Script pour générer le graphique traînée horizontale et verticale du ROV
en fonction de la vitesse (mission M102).
Génère docs/07_Ordres_de_grandeur_fig_trainee_rov.png
"""
import numpy as np
import matplotlib.pyplot as plt
import os

# Paramètres mission M102
M102 = {
    "rov": {"m": 30.0, "vol": 0.03, "a": 0.4, "b": 0.7, "h": 0.4, "Cx": 0.9, "Cy": 0.9},
    "environment": {"rho_eau": 1030.0, "g": 9.81},
}

rho = M102["environment"]["rho_eau"]
a, b, h = M102["rov"]["a"], M102["rov"]["b"], M102["rov"]["h"]
Cx, Cy = M102["rov"]["Cx"], M102["rov"]["Cy"]

Sx = a * h  # 0.16 m²
Sy = a * b  # 0.28 m²

# Vitesses (m/s) - de -2 à +2 pour couvrir descente et remontée
v = np.linspace(-2, 2, 201)

# Traînée horizontale : Fx_drag = -Cx * 0.5 * rho * Sx * vx * |vx|
# Module : |Fx_drag| = Cx * 0.5 * rho * Sx * v^2
Fx_drag_mag = Cx * 0.5 * rho * Sx * np.abs(v) * v  # Signé (négatif si v>0)
Fx_drag_abs = np.abs(Fx_drag_mag)

# Traînée verticale : Fy_drag = -Cy * 0.5 * rho * Sy * vy * |vy|
# Module : |Fy_drag| = Cy * 0.5 * rho * Sy * v^2
Fy_drag_mag = -Cy * 0.5 * rho * Sy * np.abs(v) * v  # Signé (négatif si vy>0 = montée)
Fy_drag_abs = np.abs(Fy_drag_mag)

# Graphique
fig, ax = plt.subplots(figsize=(8, 5))
ax.plot(v, Fx_drag_abs, "b-", linewidth=2, label="Traînée horizontale |Fx_drag|")
ax.plot(v, Fy_drag_abs, "r-", linewidth=2, label="Traînée verticale |Fy_drag|")

# Vitesse de remontée libre
F_apparent = 1030 * 0.03 * 9.81 - 30 * 9.81  # 8.829 N
vy_remontee = np.sqrt(2 * F_apparent / (Cy * rho * Sy))
ax.axvline(vy_remontee, color="green", linestyle="--", alpha=0.7, label=f"v remontée libre = {vy_remontee:.3f} m/s")
ax.axhline(F_apparent, color="green", linestyle=":", alpha=0.5)

ax.set_xlabel("Vitesse (m/s)", fontsize=11)
ax.set_ylabel("|Traînée| (N)", fontsize=11)
ax.set_title("Traînée horizontale et verticale du ROV (mission M102)\nen fonction de la vitesse", fontsize=12)
ax.legend(loc="upper left", fontsize=10)
ax.grid(True, alpha=0.3)
ax.set_xlim(-2, 2)
ax.set_ylim(0, None)

# Sauvegarder
script_dir = os.path.dirname(os.path.abspath(__file__))
output_path = os.path.join(script_dir, "..", "07_Ordres_de_grandeur_fig_trainee_rov.png")
plt.tight_layout()
plt.savefig(output_path, dpi=150, bbox_inches="tight")
print(f"Graphique sauvegardé : {output_path}")
plt.close()
