"""Onglet Essais : campagnes d'initialisation statique écrites dans le classeur Excel."""
from __future__ import annotations

from PyQt6.QtCore import QThread, pyqtSignal
from PyQt6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from src.utils.essais_initialisation import (
    DEFAULT_EXCEL_PATH,
    EssaisResult,
    run_essais_comparatifs,
)


class _EssaisWorker(QThread):
    """Calcule les lignes du classeur sans bloquer l'interface."""

    progress_changed = pyqtSignal(int, int, str)
    succeeded = pyqtSignal(object)
    failed = pyqtSignal(str)

    def run(self) -> None:
        try:
            result = run_essais_comparatifs(progress=self._on_progress)
            self.succeeded.emit(result)
        except Exception as exc:
            text = str(exc)
            if not text.endswith("\n"):
                text += "\n"
            self.failed.emit(text)

    def _on_progress(self, done: int, total: int, detail: str = "") -> None:
        self.progress_changed.emit(int(done), int(total), str(detail))


class EssaisTab(QWidget):
    """Lance les essais comparatifs d'initialisation et affiche leur avancement."""

    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        self._worker: _EssaisWorker | None = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)

        intro = QLabel(
            "Compare les métriques d'initialisation statique pour les lignes "
            "encore vides de chaque onglet dont le nom commence par Data. "
            "Les lignes déjà renseignées sont conservées, et le classeur est "
            "enregistré après chaque ligne calculée. Les autres paramètres "
            "viennent de la mission M_Référence."
        )
        intro.setWordWrap(True)
        layout.addWidget(intro)

        self.path_label = QLabel(str(DEFAULT_EXCEL_PATH))
        self.path_label.setWordWrap(True)
        layout.addWidget(self.path_label)

        controls = QHBoxLayout()
        self.launch_button = QPushButton("Lancer essais")
        self.launch_button.clicked.connect(self._launch)
        controls.addWidget(self.launch_button)
        controls.addStretch(1)
        layout.addLayout(controls)

        self.progress = QProgressBar()
        self.progress.setRange(0, 1)
        self.progress.setValue(0)
        self.progress.setFormat("%v / %m")
        layout.addWidget(self.progress)

        self.status_label = QLabel("En attente.")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)
        layout.addStretch(1)

    def _launch(self) -> None:
        if self._worker is not None and self._worker.isRunning():
            return
        self.launch_button.setEnabled(False)
        self.progress.setRange(0, 1)
        self.progress.setValue(0)
        self.status_label.setText("Initialisation des essais…")
        self._worker = _EssaisWorker(self)
        self._worker.progress_changed.connect(self._on_progress)
        self._worker.succeeded.connect(self._on_success)
        self._worker.failed.connect(self._on_failure)
        self._worker.start()

    def _on_progress(self, done: int, total: int, detail: str = "") -> None:
        self.progress.setRange(0, max(total, 1))
        self.progress.setValue(done)
        if detail:
            self.status_label.setText(f"{detail} ({done} / {total})")

    def _on_success(self, result: EssaisResult) -> None:
        self.launch_button.setEnabled(True)
        self.progress.setRange(0, max(result.n_rows, 1))
        self.progress.setValue(result.n_rows)
        skipped = (
            f", {result.n_skipped} déjà renseignée(s)" if result.n_skipped else ""
        )
        if result.n_rows == 0:
            if result.n_skipped:
                self.status_label.setText(
                    f"Aucune ligne à calculer : {result.n_skipped} ligne(s) déjà renseignée(s)."
                )
            else:
                self.status_label.setText(
                    "Aucune ligne de données dans les onglets Data."
                )
            return
        if result.failed_rows:
            details = "".join(
                item if str(item).endswith("\n") else f"{item}\n"
                for item in result.failed_rows
            )
            self.status_label.setText(
                f"Terminé. {result.n_rows} ligne(s) calculée(s){skipped}.\n"
                "Les métriques des lignes en échec ont été laissées vides.\n"
                f"{details}"
            )
            QMessageBox.warning(
                self,
                "Essais",
                "Essais terminés.\n" + details,
            )
            return
        self.status_label.setText(
            f"Essais terminés ({result.n_rows} ligne(s) calculée(s){skipped})."
        )

    def _on_failure(self, message: str) -> None:
        self.launch_button.setEnabled(True)
        self.status_label.setText(message)
        QMessageBox.critical(self, "Essais", message)
