"""
Fenêtre modale « Snapshot » : profil du câble, bateau et ROV (PyQt6, sans matplotlib).

Appeler ``install_snapshot_bridge(app)`` depuis le thread GUI après ``QApplication``
(p.ex. dans ``pyqt_app.main()``) pour permettre l'affichage depuis le thread de simulation.
"""

from __future__ import annotations

import math

import numpy as np

from PyQt6.QtCore import QObject, QPointF, QRectF, QThread, QMetaObject, Qt, pyqtSlot
from PyQt6.QtGui import QColor, QFont, QPainter, QPen
from PyQt6.QtWidgets import (
    QApplication,
    QDialog,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)


def _coerce_sim_time(t) -> float | None:
    """
    Convertit le temps simulé (scalaire numpy, 0-dim, etc.) en float Python pour l'affichage.
    Retourne None si absent ou non convertible.
    """
    if t is None:
        return None
    try:
        arr = np.asarray(t, dtype=np.float64)
        if arr.size == 0:
            return None
        v = float(arr.reshape(-1)[0])
    except (TypeError, ValueError):
        return None
    if not math.isfinite(v):
        return None
    return v


def _invoke_snapshot_pause() -> None:
    """Déclenche la pause simulation si ``QApplication._cable_snapshot_request_pause`` est enregistré."""
    app = QApplication.instance()
    if app is None:
        return
    fn = getattr(app, "_cable_snapshot_request_pause", None)
    if callable(fn):
        fn()


def _effective_endpoints(
    x_cable,
    y_cable,
    x_boat,
    y_boat,
    x_rov,
    y_rov,
) -> tuple[float, float, float, float]:
    """Même logique que ``CableSolver._normalize_cable_length`` pour les extrémités."""
    x_boat_eff = x_boat if x_boat is not None else float(x_cable[0])
    y_boat_eff = y_boat if y_boat is not None else float(y_cable[0])
    x_rov_eff = x_rov if x_rov is not None else float(x_cable[-1])
    y_rov_eff = y_rov if y_rov is not None else float(y_cable[-1])
    return x_boat_eff, y_boat_eff, x_rov_eff, y_rov_eff


class _CableSnapshotCanvas(QWidget):
    """Tracé 2D du câble (polyline) et marqueurs bateau / ROV."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setMinimumSize(480, 320)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._xc: np.ndarray = np.zeros(2)
        self._yc: np.ndarray = np.zeros(2)
        self._xb: float = 0.0
        self._yb: float = 0.0
        self._xr: float = 0.0
        self._yr: float = 0.0

    def set_cable_data(
        self,
        x_cable: np.ndarray,
        y_cable: np.ndarray,
        x_boat: float,
        y_boat: float,
        x_rov: float,
        y_rov: float,
    ) -> None:
        self._xc = np.asarray(x_cable, dtype=float).reshape(-1)
        self._yc = np.asarray(y_cable, dtype=float).reshape(-1)
        self._xb, self._yb = float(x_boat), float(y_boat)
        self._xr, self._yr = float(x_rov), float(y_rov)
        self.update()

    def paintEvent(self, event) -> None:  # noqa: ANN001
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        margin = 36
        plot = QRectF(margin, margin, max(1, w - 2 * margin), max(1, h - 2 * margin))

        n = min(self._xc.size, self._yc.size)
        if n < 2:
            painter.setPen(QColor(120, 120, 120))
            painter.drawText(plot, Qt.AlignmentFlag.AlignCenter, "Pas assez de points")
            return

        xs = np.concatenate([self._xc[:n], [self._xb, self._xr]])
        ys = np.concatenate([self._yc[:n], [self._yb, self._yr]])
        xmin, xmax = float(np.min(xs)), float(np.max(xs))
        ymin, ymax = float(np.min(ys)), float(np.max(ys))
        dx = xmax - xmin
        dy = ymax - ymin
        pad = max(dx, dy) * 0.08 + 1e-6
        xmin -= pad
        xmax += pad
        ymin -= pad
        ymax += pad
        wx = xmax - xmin
        wy = ymax - ymin
        side = min(plot.width(), plot.height())
        inner = QRectF(0, 0, side, side)
        inner.moveCenter(plot.center())

        def to_screen(x: float, y: float) -> QPointF:
            u = (x - xmin) / wx
            v = (y - ymin) / wy
            ix = inner.left() + u * inner.width()
            iy = inner.bottom() - v * inner.height()
            return QPointF(ix, iy)

        painter.fillRect(self.rect(), QColor(255, 255, 255))
        painter.setPen(QPen(QColor(40, 40, 40), 1.5))
        for i in range(n - 1):
            p0 = to_screen(float(self._xc[i]), float(self._yc[i]))
            p1 = to_screen(float(self._xc[i + 1]), float(self._yc[i + 1]))
            painter.drawLine(p0, p1)

        r = 6.0
        boat_pt = to_screen(self._xb, self._yb)
        rov_pt = to_screen(self._xr, self._yr)
        painter.setPen(QPen(QColor(0, 90, 160), 2))
        painter.setBrush(QColor(30, 120, 200, 200))
        painter.drawEllipse(boat_pt, r, r)
        painter.setPen(QPen(QColor(160, 60, 0), 2))
        painter.setBrush(QColor(220, 100, 30, 200))
        painter.drawEllipse(rov_pt, r, r)

        painter.setPen(QColor(60, 60, 60))
        font = QFont()
        font.setPointSize(9)
        painter.setFont(font)
        painter.drawText(boat_pt + QPointF(10, -8), "Bateau")
        painter.drawText(rov_pt + QPointF(10, 16), "ROV")

        painter.setPen(QColor(180, 180, 180))
        painter.drawRect(inner)


class CableSnapshotDialog(QDialog):
    """Dialogue bloquant avec tracé et bouton Quit."""

    def __init__(
        self,
        x_cable: np.ndarray,
        y_cable: np.ndarray,
        x_boat: float,
        y_boat: float,
        x_rov: float,
        y_rov: float,
        L_target: float,
        t: float | None,
        window_title: str = "Snapshot",
        parent: QWidget | None = None,
    ):
        super().__init__(parent)
        self.setWindowTitle(window_title)
        self.setModal(True)

        self._canvas = _CableSnapshotCanvas(self)
        self._canvas.set_cable_data(x_cable, y_cable, x_boat, y_boat, x_rov, y_rov)

        t_disp = _coerce_sim_time(t)
        t_str = f"{t_disp:.3f} s" if t_disp is not None else "—"
        info = QLabel(f"L_target = {float(L_target):.4f} m   |   t = {t_str}")
        info.setStyleSheet("color: #444;")

        btn_quit = QPushButton("Quit")
        btn_quit.setDefault(True)
        btn_quit.clicked.connect(self.accept)

        btn_pause = QPushButton("Pause")
        btn_pause.setStyleSheet("background-color: #ffc107; color: black; padding: 6px 14px;")
        btn_pause.clicked.connect(_invoke_snapshot_pause)

        row = QHBoxLayout()
        row.addStretch(1)
        row.addWidget(btn_quit)
        row.addWidget(btn_pause)
        row.addStretch(1)

        layout = QVBoxLayout(self)
        layout.addWidget(self._canvas, stretch=1)
        layout.addWidget(info)
        layout.addLayout(row)
        self.resize(640, 480)


class _SnapshotBridge(QObject):
    """Récepteur sur le thread GUI pour ``dlg.exec()`` depuis un autre thread."""

    def __init__(self, parent: QObject | None = None):
        super().__init__(parent)
        self._pending = None
        # Temps simulé pour le libellé (float Python) : défini avec _pending avant dlg.exec().
        self._snapshot_t: float | None = None

    @pyqtSlot()
    def show_locked(self) -> None:
        pack = self._pending
        self._pending = None
        t_eff = getattr(self, "_snapshot_t", None)
        self._snapshot_t = None
        if pack is None:
            return
        xc, yc, xb, yb, xr, yr, L_target, _t_pack, window_title = pack
        if t_eff is None:
            t_eff = _coerce_sim_time(_t_pack)
        dlg = CableSnapshotDialog(
            xc, yc, xb, yb, xr, yr, L_target, t_eff, window_title=window_title
        )
        dlg.exec()


def install_snapshot_bridge(app: QApplication) -> None:
    """À appeler une fois depuis le thread GUI après création de ``QApplication``."""
    if getattr(app, "_cable_snapshot_bridge", None) is not None:
        return
    bridge = _SnapshotBridge(app)
    app._cable_snapshot_bridge = bridge


def show_cable_snapshot_modal(
    x_cable,
    y_cable,
    L_target,
    x_boat=None,
    y_boat=0.0,
    x_rov=None,
    y_rov=None,
    k_tail=10,
    k_max=2.0,
    bidirectional=True,
    _from_direction=None,
    t: float | None = None,
    Id: str = "Snapshot",
) -> None:
    """
    Affiche une fenêtre modale (titre = ``Id``, défaut « Snapshot »), bloquante jusqu'au clic sur Quit.

    Signature alignée sur ``CableSolver._normalize_cable_length`` ; les paramètres
    ``k_tail``, ``k_max``, ``bidirectional``, ``_from_direction`` ne sont pas utilisés
    pour le dessin mais conservés pour compatibilité d'appel.

    Sans QApplication (tests hors GUI), la fonction ne fait rien.
    Depuis le thread de simulation, le pont doit avoir été installé via
    ``install_snapshot_bridge`` dans ``main()`` ; sinon la fonction est ignorée.
    """
    del k_tail, k_max, bidirectional, _from_direction

    window_title = str(Id)

    app = QApplication.instance()
    if app is None:
        return

    xc = np.asarray(x_cable, dtype=float)
    yc = np.asarray(y_cable, dtype=float)
    xb, yb, xr, yr = _effective_endpoints(xc, yc, x_boat, y_boat, x_rov, y_rov)
    t_display = _coerce_sim_time(t)
    pack = (xc, yc, xb, yb, xr, yr, float(L_target), t_display, window_title)

    bridge = getattr(app, "_cable_snapshot_bridge", None)
    if bridge is None:
        if QThread.currentThread() != app.thread():
            return
        install_snapshot_bridge(app)
        bridge = app._cable_snapshot_bridge

    bridge._snapshot_t = t_display
    bridge._pending = pack

    if QThread.currentThread() == app.thread():
        bridge.show_locked()
    else:
        QMetaObject.invokeMethod(
            bridge,
            "show_locked",
            Qt.ConnectionType.BlockingQueuedConnection,
        )
