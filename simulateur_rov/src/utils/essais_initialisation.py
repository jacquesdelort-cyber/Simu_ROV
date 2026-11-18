"""Essais comparatifs d'initialisation statique, à partir de la mission M_Référence."""
from __future__ import annotations

import copy
import json
import os
import re
import traceback
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from openpyxl import load_workbook
from openpyxl.utils import column_index_from_string, get_column_letter

from src.models.state_projection import reconcile_straight_mode_after_normalize
from src.models.system_model import ROVSystem
from src.solvers.forces import compute_cable_forces
from src.utils.initial_conditions import get_initial_state

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_EXCEL_PATH = PROJECT_ROOT / "docs" / "Essais comparatifs initialisation.xlsx"
DEFAULT_MISSION_NAME = "M_Référence"

INPUT_COLUMNS = ("x", "y", "L_init", "courant", "mv_câble")
METRIC_COLUMNS = (
    "T_bat",
    "T_ROV",
    "T_max",
    "Tr_câble_x",
    "Tr_ROV_x",
    "Flèche",
    "L_straight",
    "Slack",
)


class EssaisExcelError(RuntimeError):
    """Le classeur d'essais n'a pas pu être lu ou enregistré."""


@dataclass
class EssaisResult:
    n_rows: int
    failed_rows: list[str]
    excel_path: str
    n_skipped: int = 0


def default_mission_path() -> Path:
    return PROJECT_ROOT / "Missions" / DEFAULT_MISSION_NAME / "Param_mission.json"


def load_reference_mission(mission_path: Path | None = None) -> dict:
    path = Path(mission_path) if mission_path is not None else default_mission_path()
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _error_message(text: str) -> str:
    """Formate un message d'erreur, une ligne logique par retour à la ligne."""
    lines = [line.rstrip() for line in str(text).splitlines()] or [""]
    return "\n".join(lines) + "\n"


def _report_error(text: str) -> str:
    message = _error_message(text)
    print(message, end="", flush=True)
    return message


def _row_failure_message(title: str, row: int, inputs, raw, exc: BaseException) -> str:
    if inputs:
        params = ", ".join(f"{name}={inputs.get(name)}" for name in INPUT_COLUMNS)
    elif raw:
        params = ", ".join(f"{name}={raw.get(name)!r}" for name in INPUT_COLUMNS)
    else:
        params = "paramètres absents"
    frames = traceback.extract_tb(exc.__traceback__)
    location = ""
    if frames:
        last = frames[-1]
        location = f" ({Path(last.filename).name}:{last.lineno}, {last.name})"
    reason = f"{type(exc).__name__}: {exc}".strip() or type(exc).__name__
    return _report_error(
        f"Onglet « {title} », ligne {row} ({params}) : {reason}{location}"
    )


def _is_blank(value) -> bool:
    if value is None:
        return True
    if isinstance(value, str) and value.strip() == "":
        return True
    return False


def _as_float(value) -> float:
    if isinstance(value, str):
        value = value.strip().replace(",", ".")
    return float(value)


_PENDING = object()
_CELL_REFERENCE = re.compile(r"\$?([A-Za-z]{1,3})\$?(\d+)")


class _FormulaError(Exception):
    """La formule ne peut pas être calculée par l'évaluateur intégré."""


def _formula_number(value) -> float:
    if value is None or (isinstance(value, str) and value.strip() == ""):
        return 0.0
    if isinstance(value, bool):
        return float(int(value))
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value.strip().replace(",", "."))
        except ValueError as exc:
            raise _FormulaError(f"« {value} » n'est pas un nombre") from exc
    raise _FormulaError(f"« {value} » n'est pas un nombre")


class _FormulaParser:
    """Calcule une formule limitée aux nombres, références et opérations + - * /."""

    def __init__(self, evaluator, sheet_name: str, text: str):
        self.evaluator = evaluator
        self.sheet_name = sheet_name
        self.text = text
        self.position = 0

    def skip(self) -> None:
        while self.position < len(self.text) and self.text[self.position].isspace():
            self.position += 1

    def parse_expression(self):
        value = self.parse_term()
        while True:
            self.skip()
            if self.position >= len(self.text) or self.text[self.position] not in "+-":
                return value
            operator = self.text[self.position]
            self.position += 1
            right = self.parse_term()
            left = _formula_number(value)
            right_value = _formula_number(right)
            value = left + right_value if operator == "+" else left - right_value

    def parse_term(self):
        value = self.parse_factor()
        while True:
            self.skip()
            if self.position >= len(self.text) or self.text[self.position] not in "*/":
                return value
            operator = self.text[self.position]
            self.position += 1
            right = _formula_number(self.parse_factor())
            left = _formula_number(value)
            if operator == "*":
                value = left * right
            elif right == 0:
                raise _FormulaError("division par zéro")
            else:
                value = left / right

    def parse_factor(self):
        self.skip()
        if self.position >= len(self.text):
            raise _FormulaError("formule incomplète")
        char = self.text[self.position]
        if char in "+-":
            self.position += 1
            value = _formula_number(self.parse_factor())
            return value if char == "+" else -value
        if char == "(":
            self.position += 1
            value = self.parse_expression()
            self.skip()
            if self.position >= len(self.text) or self.text[self.position] != ")":
                raise _FormulaError("parenthèse fermante manquante")
            self.position += 1
            return value
        if char.isdigit() or char in ".,":
            return self.parse_number()
        return self.parse_reference()

    def parse_number(self) -> float:
        start = self.position
        while self.position < len(self.text) and (
            self.text[self.position].isdigit() or self.text[self.position] in ".,"
        ):
            self.position += 1
        if self.position < len(self.text) and self.text[self.position] in "eE":
            end = self.position + 1
            if end < len(self.text) and self.text[end] in "+-":
                end += 1
            exponent = end
            while end < len(self.text) and self.text[end].isdigit():
                end += 1
            if end > exponent:
                self.position = end
        token = self.text[start:self.position].replace(",", ".")
        try:
            return float(token)
        except ValueError as exc:
            raise _FormulaError(f"nombre « {token} » illisible") from exc

    def parse_reference(self):
        self.skip()
        match = _CELL_REFERENCE.match(self.text[self.position:])
        if match is None:
            remainder = self.text[self.position:].strip().split()
            token = remainder[0] if remainder else "?"
            raise _FormulaError(f"élément « {token} » non reconnu")
        self.position += match.end()
        column = column_index_from_string(match.group(1))
        row = int(match.group(2))
        return self.evaluator.value(self.sheet_name, row, column)


class _FormulaEvaluator:
    """Lit la valeur calculée d'une cellule, y compris pour une formule simple."""

    def __init__(self, workbook, values_workbook):
        self.workbook = workbook
        self.values_workbook = values_workbook
        self._memo: dict[tuple, object] = {}

    def value(self, sheet_name: str, row: int, column: int):
        key = (sheet_name, row, column)
        if key in self._memo:
            stored = self._memo[key]
            if stored is _PENDING:
                raise _FormulaError("référence circulaire")
            return stored
        self._memo[key] = _PENDING
        try:
            raw = self.workbook[sheet_name].cell(row, column).value
            if isinstance(raw, str) and raw.startswith("="):
                try:
                    result = self._evaluate(sheet_name, raw)
                except _FormulaError as exc:
                    result = self._cached_value(sheet_name, row, column)
                    if result is None:
                        coord = f"{get_column_letter(column)}{row}"
                        raise ValueError(
                            f"Formule « {raw} » en {sheet_name}!{coord} : {exc}."
                        ) from exc
            else:
                result = raw
        except Exception:
            self._memo.pop(key, None)
            raise
        self._memo[key] = result
        return result

    def _evaluate(self, sheet_name: str, formula: str):
        parser = _FormulaParser(self, sheet_name, formula[1:])
        value = parser.parse_expression()
        parser.skip()
        if parser.position < len(parser.text):
            remainder = parser.text[parser.position:]
            raise _FormulaError(f"partie non reconnue « {remainder} »")
        return value

    def _cached_value(self, sheet_name: str, row: int, column: int):
        if self.values_workbook is None or sheet_name not in self.values_workbook.sheetnames:
            return None
        value = self.values_workbook[sheet_name].cell(row, column).value
        if isinstance(value, str) and value.startswith("="):
            return None
        return value


def _evaluated_inputs(worksheet, row: int, columns: dict[str, int], raw: dict, evaluator) -> dict:
    """Remplace les formules de la ligne par leur valeur calculée."""
    evaluated = {}
    for name in INPUT_COLUMNS:
        value = raw.get(name)
        if isinstance(value, str) and value.startswith("="):
            evaluated[name] = evaluator.value(worksheet.title, row, columns[name])
        else:
            evaluated[name] = value
    return evaluated


def _header_map(worksheet) -> dict[str, int]:
    found: dict[str, int] = {}
    for column in range(1, worksheet.max_column + 1):
        name = worksheet.cell(1, column).value
        if isinstance(name, str):
            found[name.strip()] = column
    missing = [name for name in (*INPUT_COLUMNS, *METRIC_COLUMNS) if name not in found]
    if missing:
        expected = ", ".join((*INPUT_COLUMNS, *METRIC_COLUMNS))
        raise EssaisExcelError(_error_message(
            f"Colonnes manquantes dans l'onglet « {worksheet.title} » : "
            f"{', '.join(missing)}. Colonnes attendues : {expected}."
        ))
    return found


def _row_inputs(worksheet, row: int, columns: dict[str, int]) -> dict[str, object] | None:
    values = {name: worksheet.cell(row, columns[name]).value for name in INPUT_COLUMNS}
    if all(_is_blank(values[name]) for name in INPUT_COLUMNS):
        return None
    return values


def _distance_to_line(px: float, py: float, x0: float, y0: float, x1: float, y1: float) -> float:
    dx = x1 - x0
    dy = y1 - y0
    length = float(np.hypot(dx, dy))
    if length < 1e-12:
        return float(np.hypot(px - x0, py - y0))
    return abs((px - x0) * dy - (py - y0) * dx) / length


def _fleche(x_cable, y_cable, x_boat: float, y_boat: float, x_rov: float, y_rov: float) -> float:
    distances = [
        _distance_to_line(float(x), float(y), x_boat, y_boat, x_rov, y_rov)
        for x, y in zip(x_cable, y_cable)
    ]
    if not distances:
        return 0.0
    return float(max(distances))


def _finalize_like_simulation_ui(system, x_cable, y_cable, length, x_boat, y_boat, x_rov, y_rov):
    """Même post-traitement que l'onglet Simulation en mode strict avec courant."""
    solver = system.cable.solver
    xc, yc, straight_mode, _ = solver._normalize_cable_length(
        np.asarray(x_cable, dtype=float),
        np.asarray(y_cable, dtype=float),
        float(length),
        x_boat=float(x_boat),
        y_boat=float(y_boat),
        x_rov=float(x_rov),
        y_rov=float(y_rov),
        k_tail=10,
        t=0.0,
    )
    xc, yc, x_rov, y_rov = reconcile_straight_mode_after_normalize(
        xc, yc, straight_mode, float(x_rov), float(y_rov), float(x_boat), float(y_boat)
    )
    xc, yc = solver._enforce_uniform_current_concavity(
        np.asarray(xc, dtype=float),
        np.asarray(yc, dtype=float),
        float(x_boat),
        float(y_boat),
        float(x_rov),
        float(y_rov),
        strength=0.78,
    )
    xc, yc, straight_mode, _ = solver._normalize_cable_length(
        xc,
        yc,
        float(length),
        x_boat=float(x_boat),
        y_boat=float(y_boat),
        x_rov=float(x_rov),
        y_rov=float(y_rov),
        k_tail=10,
        t=0.0,
    )
    xc, yc, x_rov, y_rov = reconcile_straight_mode_after_normalize(
        xc, yc, straight_mode, float(x_rov), float(y_rov), float(x_boat), float(y_boat)
    )
    return np.asarray(xc, dtype=float), np.asarray(yc, dtype=float)


def compute_init_metrics(
    mission: dict,
    x: float,
    y: float,
    l_init: float,
    courant,
    rho_cable: float,
    n_segments: int | None = None,
) -> dict[str, float]:
    """Initialise le câble et renvoie les huit métriques de la feuille Data."""
    params = copy.deepcopy(mission["parameters"])
    params.setdefault("cable", {})
    params["cable"]["rho_cable"] = float(rho_cable)

    calc = mission.get("calc_params") or {}
    init = mission.get("init_params") or {}
    n_seg = int(n_segments if n_segments is not None else calc.get("N_segments", 50))
    x_boat = float(init.get("x_boat_init", 0.0))
    y_boat = 0.0

    system = ROVSystem(params, N_segments=n_seg)
    system.environment.v_courant_raw = courant
    y0 = get_initial_state(
        system,
        x_rov=float(x),
        y_rov=float(y),
        x_boat=x_boat,
        L=float(l_init),
        use_current_geometry=True,
        cable_init_mode=init.get("cable_init_mode", "strict_static"),
        legacy_init_geometry=bool(init.get("legacy_init_geometry", False)),
    )
    (_, _, _, _, _, _, x_cable, y_cable, _, length) = system.unpack_state(y0)
    x_cable, y_cable = _finalize_like_simulation_ui(
        system, x_cable, y_cable, length, x_boat, y_boat, float(x), float(y)
    )

    weight = (
        (system.cable.rho_cable - system.environment.rho_eau)
        * system.cable.A_cable
        * system.environment.g
    )
    tension = np.asarray(
        system.cable.solver._compute_catenary_tensions(
            x_cable,
            y_cable,
            weight,
            rov_m=system.rov.m,
            rov_vol=system.rov.V,
        ),
        dtype=float,
    )
    vx = np.zeros(len(x_cable))
    vy = np.zeros(len(y_cable))
    params_cable = {
        "d": system.cable.d,
        "rho_cable": system.cable.rho_cable,
        "Cx_cable": system.cable.Cx_cable,
        "Cf_cable": system.cable.Cf_cable,
    }
    fx_cable, *_ = compute_cable_forces(
        x_cable, y_cable, vx, vy, system.environment, params_cable, float(l_init)
    )
    fx_rov, _ = system.rov.compute_drag_force(0.0, 0.0, float(y), system.environment)

    l_straight = float(np.hypot(float(x) - x_boat, float(y) - y_boat))
    return {
        "T_bat": float(tension[0]) if tension.size else 0.0,
        "T_ROV": float(tension[-1]) if tension.size else 0.0,
        "T_max": float(np.max(tension)) if tension.size else 0.0,
        "Tr_câble_x": float(np.sum(fx_cable)),
        "Tr_ROV_x": float(fx_rov),
        "Flèche": _fleche(x_cable, y_cable, x_boat, y_boat, float(x), float(y)),
        "L_straight": l_straight,
        "Slack": float(l_init) - l_straight,
    }


def _resolve_inputs(raw: dict[str, object], mission: dict) -> dict[str, object]:
    init = mission.get("init_params") or {}
    cable = (mission.get("parameters") or {}).get("cable") or {}
    defaults = {
        "x": init.get("x_rov_init", 0.0),
        "y": init.get("y_rov_init", -20.0),
        "L_init": init.get("L_init", 35.0),
        "courant": init.get("v_courant", "0.0"),
        "mv_câble": cable.get("rho_cable", 950.0),
    }
    resolved: dict[str, object] = {}
    for name in INPUT_COLUMNS:
        if _is_blank(raw.get(name)):
            resolved[name] = defaults[name]
        else:
            resolved[name] = raw[name]
    resolved["x"] = _as_float(resolved["x"])
    resolved["y"] = _as_float(resolved["y"])
    resolved["L_init"] = _as_float(resolved["L_init"])
    resolved["mv_câble"] = _as_float(resolved["mv_câble"])
    courant = resolved["courant"]
    if isinstance(courant, str):
        text = courant.strip()
        if text == "":
            resolved["courant"] = defaults["courant"]
        elif ":" not in text and ";" not in text:
            resolved["courant"] = _as_float(text)
        else:
            resolved["courant"] = text
    else:
        resolved["courant"] = _as_float(courant)
    return resolved


def _clear_metrics(worksheet, row: int, columns: dict[str, int]) -> None:
    for name in METRIC_COLUMNS:
        worksheet.cell(row, columns[name]).value = None


def _metrics_all_blank(worksheet, row: int, columns: dict[str, int]) -> bool:
    return all(
        _is_blank(worksheet.cell(row, columns[name]).value) for name in METRIC_COLUMNS
    )


def _data_rows(worksheet, columns: dict[str, int]) -> list[int]:
    rows = []
    for row in range(2, worksheet.max_row + 1):
        if _row_inputs(worksheet, row, columns) is not None:
            rows.append(row)
    return rows


def _data_sheets(workbook):
    return [sheet for sheet in workbook.worksheets if sheet.title.casefold().startswith("data")]


def _excel_locked_message(path: Path, exc: BaseException) -> str:
    return _error_message(
        f"Impossible d'enregistrer « {path} ». "
        "Fermez ce classeur s'il est ouvert dans Excel, puis relancez les essais. "
        f"Erreur système : {exc}"
    )


def _save_workbook(workbook, path: Path) -> None:
    """Enregistre le classeur par remplacement atomique du fichier d'origine."""
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    try:
        workbook.save(tmp_path)
    except PermissionError as exc:
        raise EssaisExcelError(_excel_locked_message(path, exc)) from exc
    try:
        os.replace(tmp_path, path)
    except PermissionError as exc:
        raise EssaisExcelError(_excel_locked_message(path, exc)) from exc
    finally:
        if tmp_path.exists() and tmp_path != path:
            try:
                tmp_path.unlink()
            except OSError:
                pass


def run_essais_comparatifs(
    excel_path: Path | str | None = None,
    mission_path: Path | str | None = None,
    n_segments: int | None = None,
    progress=None,
) -> EssaisResult:
    """
    Calcule les métriques des lignes encore vides de chaque onglet « Data* ».

    Un onglet est pris en compte lorsque son nom commence par « Data ».
    Une ligne n'est calculée que si toutes les cellules de métriques sont vides.
    Les formules des cellules d'entrée sont lues par leur valeur calculée.
    Le classeur est enregistré après chaque ligne calculée, donc aussi à la fin
    de chaque onglet. ``progress(done, total, detail)`` est appelé au début,
    puis après chaque ligne. Les lignes en échec gardent des métriques vides
    et produisent un message qui se termine par un retour à la ligne.
    Si le classeur est ouvert ailleurs, l'enregistrement en cours est refusé
    et les lignes déjà écrites restent dans le fichier.
    """
    path = Path(excel_path) if excel_path is not None else DEFAULT_EXCEL_PATH
    mission = load_reference_mission(Path(mission_path) if mission_path is not None else None)
    if not path.is_file():
        raise EssaisExcelError(_error_message(
            f"Fichier Excel introuvable : {path}"
        ))

    values_workbook = load_workbook(path, data_only=True)
    try:
        workbook = load_workbook(path)
    except Exception:
        values_workbook.close()
        raise
    try:
        evaluator = _FormulaEvaluator(workbook, values_workbook)
        sheets = _data_sheets(workbook)
        if not sheets:
            present = ", ".join(workbook.sheetnames) or "aucun"
            raise EssaisExcelError(_error_message(
                f"Aucun onglet dont le nom commence par « Data » dans « {path.name} ». "
                f"Onglets présents : {present}."
            ))

        pending: list[tuple] = []
        skipped = 0
        for worksheet in sheets:
            columns = _header_map(worksheet)
            for row in _data_rows(worksheet, columns):
                if _metrics_all_blank(worksheet, row, columns):
                    pending.append((worksheet, columns, row))
                else:
                    skipped += 1

        total = len(pending)
        if progress is not None:
            progress(0, total, "")

        failed: list[str] = []
        for done, (worksheet, columns, row) in enumerate(pending, start=1):
            raw = _row_inputs(worksheet, row, columns)
            inputs = None
            try:
                evaluated = _evaluated_inputs(worksheet, row, columns, raw or {}, evaluator)
                inputs = evaluated
                inputs = _resolve_inputs(evaluated, mission)
                metrics = compute_init_metrics(
                    mission,
                    x=float(inputs["x"]),
                    y=float(inputs["y"]),
                    l_init=float(inputs["L_init"]),
                    courant=inputs["courant"],
                    rho_cable=float(inputs["mv_câble"]),
                    n_segments=n_segments,
                )
                for name in METRIC_COLUMNS:
                    worksheet.cell(row, columns[name]).value = float(metrics[name])
            except Exception as exc:
                _clear_metrics(worksheet, row, columns)
                failed.append(
                    _row_failure_message(worksheet.title, row, inputs, raw, exc)
                )
            _save_workbook(workbook, path)
            if progress is not None:
                progress(done, total, f"{worksheet.title} — ligne {row}")

        return EssaisResult(
            n_rows=total,
            failed_rows=failed,
            excel_path=str(path),
            n_skipped=skipped,
        )
    finally:
        workbook.close()
        values_workbook.close()
