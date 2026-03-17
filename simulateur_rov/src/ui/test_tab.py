"""
Onglet « Test » pour l'application PyQt.

Fonctionnalités :
- liste des tests définis dans src/tests/test_catalog.py,
- cases à cocher pour sélectionner les tests à lancer,
- boutons « Mettre à jour » (recharge le catalogue et l'état) et « Lancer tests »,
- persistance de l'état (dernière exécution, statut, commentaires, cases cochées)
  dans tests/test_status.json.
"""

from __future__ import annotations

import json
import os
import shlex
import subprocess
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QAbstractItemView,
    QCheckBox,
    QMessageBox,
)

from src.tests.test_catalog import get_all_tests, TestEntry
from src.utils.logger import trace_print


BASE_DIR = Path(__file__).resolve().parents[2]
STATUS_PATH = BASE_DIR / "tests" / "test_status.json"


@dataclass
class TestStatus:
    """État persistant pour un test."""

    checked: bool = False
    last_run: Optional[str] = None  # ISO format
    status: Optional[str] = None  # "PASS", "FAIL" ou None
    comment: Optional[str] = None


class TestTab(QWidget):
    """Onglet de gestion et d'exécution des tests."""

    COL_CHECK = 0
    COL_NAME = 1
    COL_DESC = 2
    COL_LAST_RUN = 3
    COL_STATUS = 4
    COL_COMMENT = 5

    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        self._entries: List[TestEntry] = []
        self._status: Dict[str, TestStatus] = {}
        self._init_ui()
        self._load_status()
        self._reload_catalog()

    # ------------------------------------------------------------------
    # Initialisation UI
    # ------------------------------------------------------------------
    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)

        # Barre de boutons
        button_layout = QHBoxLayout()

        self.btn_refresh = QPushButton("🔄 Mettre à jour")
        self.btn_run = QPushButton("▶️ Lancer tests")

        self.btn_refresh.clicked.connect(self.on_refresh_clicked)
        self.btn_run.clicked.connect(self.on_run_clicked)

        button_layout.addWidget(self.btn_refresh)
        button_layout.addWidget(self.btn_run)
        button_layout.addStretch()

        layout.addLayout(button_layout)

        # Tableau des tests
        self.table = QTableWidget(0, 6, self)
        self.table.setHorizontalHeaderLabels(
            ["✔", "Test / Fonction", "Description", "Dernière exécution", "Résultat", "Commentaires / Rapport"]
        )
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(self.COL_CHECK, QHeaderView.ResizeMode.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(self.COL_STATUS, QHeaderView.ResizeMode.ResizeToContents)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setAlternatingRowColors(True)

        layout.addWidget(self.table)

    # ------------------------------------------------------------------
    # Persistance
    # ------------------------------------------------------------------
    def _load_status(self) -> None:
        """Charge test_status.json s'il existe."""
        self._status.clear()
        if not STATUS_PATH.exists():
            return
        try:
            with STATUS_PATH.open("r", encoding="utf-8") as f:
                raw = json.load(f)
        except Exception as exc:
            trace_print(7, f"[DEBUG] TestTab: impossible de lire {STATUS_PATH}: {exc}")
            return

        for test_id, data in raw.items():
            self._status[test_id] = TestStatus(
                checked=bool(data.get("checked", False)),
                last_run=data.get("last_run"),
                status=data.get("status"),
                comment=data.get("comment"),
            )

    def _save_status(self) -> None:
        """Sauvegarde test_status.json."""
        try:
            STATUS_PATH.parent.mkdir(parents=True, exist_ok=True)
            payload: Dict[str, dict] = {}
            for test_id, st in self._status.items():
                payload[test_id] = {
                    "checked": st.checked,
                    "last_run": st.last_run,
                    "status": st.status,
                    "comment": st.comment,
                }
            with STATUS_PATH.open("w", encoding="utf-8") as f:
                json.dump(payload, f, ensure_ascii=False, indent=2)
        except Exception as exc:
            trace_print(10, f"[DEBUG] TestTab: erreur lors de la sauvegarde de {STATUS_PATH}: {exc}")

    # ------------------------------------------------------------------
    # Remplissage du tableau
    # ------------------------------------------------------------------
    def _reload_catalog(self) -> None:
        """Recharge la liste des tests et (re)construit le tableau."""
        self._entries = get_all_tests()
        self.table.setRowCount(0)

        for idx, entry in enumerate(self._entries):
            self.table.insertRow(idx)

            # État persistant ou valeur par défaut
            st = self._status.get(entry.id, TestStatus())
            self._status.setdefault(entry.id, st)

            # Col 0 : case à cocher
            checkbox = QCheckBox()
            checkbox.setChecked(st.checked)
            checkbox.stateChanged.connect(lambda _state, e_id=entry.id: self._on_checkbox_changed(e_id))
            self.table.setCellWidget(idx, self.COL_CHECK, checkbox)

            # Col 1 : nom
            item_name = QTableWidgetItem(entry.name)
            item_name.setData(Qt.ItemDataRole.UserRole, entry.id)
            self.table.setItem(idx, self.COL_NAME, item_name)

            # Col 2 : description (avec thème)
            desc_text = f"[{entry.group}] {entry.description}"
            self.table.setItem(idx, self.COL_DESC, QTableWidgetItem(desc_text))

            # Col 3 : dernière exécution
            last_run_str = st.last_run or ""
            self.table.setItem(idx, self.COL_LAST_RUN, QTableWidgetItem(last_run_str))

            # Col 4 : statut (PASS/FAIL)
            status_item = QTableWidgetItem(st.status or "")
            if st.status == "PASS":
                status_item.setForeground(Qt.GlobalColor.darkGreen)
            elif st.status == "FAIL":
                status_item.setForeground(Qt.GlobalColor.red)
            self.table.setItem(idx, self.COL_STATUS, status_item)

            # Col 5 : commentaire (et éventuel lien vers rapport)
            comment_parts: List[str] = []
            if st.comment:
                comment_parts.append(st.comment)
            if entry.html_report:
                rel = entry.html_report
                comment_parts.append(f"Rapport: {rel}")
            comment_text = " | ".join(comment_parts)
            self.table.setItem(idx, self.COL_COMMENT, QTableWidgetItem(comment_text))

        self.table.resizeRowsToContents()

    # ------------------------------------------------------------------
    # Slots
    # ------------------------------------------------------------------
    def _on_checkbox_changed(self, test_id: str) -> None:
        """Met à jour l'état 'checked' d'un test lorsqu'on coche/décoche."""
        for row in range(self.table.rowCount()):
            item = self.table.item(row, self.COL_NAME)
            if not item:
                continue
            if item.data(Qt.ItemDataRole.UserRole) == test_id:
                w = self.table.cellWidget(row, self.COL_CHECK)
                if isinstance(w, QCheckBox):
                    checked = w.isChecked()
                    st = self._status.get(test_id, TestStatus())
                    st.checked = checked
                    self._status[test_id] = st
                    self._save_status()
                break

    def on_refresh_clicked(self) -> None:
        """Recharge le catalogue et l'état depuis le disque."""
        self._load_status()
        self._reload_catalog()

    def on_run_clicked(self) -> None:
        """Exécute tous les tests cochés, séquentiellement."""
        selected_entries: List[TestEntry] = []
        for row in range(self.table.rowCount()):
            item = self.table.item(row, self.COL_NAME)
            if not item:
                continue
            test_id = item.data(Qt.ItemDataRole.UserRole)
            st = self._status.get(test_id, TestStatus())
            w = self.table.cellWidget(row, self.COL_CHECK)
            if isinstance(w, QCheckBox) and w.isChecked():
                # Utiliser l'entrée correspondante
                for e in self._entries:
                    if e.id == test_id:
                        selected_entries.append(e)
                        break

        if not selected_entries:
            QMessageBox.information(self, "Tests", "Aucun test sélectionné.")
            return

        project_root = str(BASE_DIR)

        for entry in selected_entries:
            trace_print(7, f"[DEBUG] TestTab: exécution du test '{entry.id}' avec commande: {entry.command}")
            started_at = datetime.now().isoformat(timespec="seconds")
            status = "FAIL"
            comment = ""

            try:
                # On exécute la commande dans le dossier racine du projet
                # entry.command est une ligne entière (ex: "python -m pytest -q tests/...")
                args = shlex.split(entry.command)
                completed = subprocess.run(
                    args,
                    cwd=project_root,
                    capture_output=True,
                    text=True,
                )
                if completed.returncode == 0:
                    status = "PASS"
                    comment = "Succès."
                else:
                    status = "FAIL"
                    # On tronque la sortie pour éviter un commentaire trop long
                    stderr = (completed.stderr or "").strip()
                    stdout = (completed.stdout or "").strip()
                    msg_parts: List[str] = []
                    if stderr:
                        msg_parts.append(stderr.splitlines()[-1])
                    elif stdout:
                        msg_parts.append(stdout.splitlines()[-1])
                    comment = " / ".join(msg_parts) or "Échec (retour différent de 0)."
            except FileNotFoundError as exc:
                status = "FAIL"
                comment = f"Commande introuvable: {exc}"
            except Exception as exc:  # pragma: no cover - garde-fou
                status = "FAIL"
                comment = f"Erreur: {exc}"

            # Mettre à jour le statut
            st = self._status.get(entry.id, TestStatus())
            st.last_run = started_at
            st.status = status
            st.comment = comment
            self._status[entry.id] = st
            self._update_row_for_entry(entry.id, st)
            self._save_status()

        QMessageBox.information(self, "Tests", "Exécution des tests terminée.")

    # ------------------------------------------------------------------
    # Mise à jour d'une ligne
    # ------------------------------------------------------------------
    def _update_row_for_entry(self, test_id: str, st: TestStatus) -> None:
        """Met à jour les colonnes Last run / Status / Comment pour une entrée."""
        for row in range(self.table.rowCount()):
            item_name = self.table.item(row, self.COL_NAME)
            if not item_name:
                continue
            if item_name.data(Qt.ItemDataRole.UserRole) != test_id:
                continue

            # Col 3 : dernière exécution
            self.table.setItem(row, self.COL_LAST_RUN, QTableWidgetItem(st.last_run or ""))

            # Col 4 : statut
            status_item = QTableWidgetItem(st.status or "")
            if st.status == "PASS":
                status_item.setForeground(Qt.GlobalColor.darkGreen)
            elif st.status == "FAIL":
                status_item.setForeground(Qt.GlobalColor.red)
            self.table.setItem(row, self.COL_STATUS, status_item)

            # Col 5 : commentaire + lien éventuel
            # Retrouver l'entrée pour savoir s'il y a un rapport
            entry = next((e for e in self._entries if e.id == test_id), None)
            comment_parts: List[str] = []
            if st.comment:
                comment_parts.append(st.comment)
            if entry and entry.html_report:
                comment_parts.append(f"Rapport: {entry.html_report}")
            comment_text = " | ".join(comment_parts)
            self.table.setItem(row, self.COL_COMMENT, QTableWidgetItem(comment_text))
            break


__all__ = ["TestTab"]

