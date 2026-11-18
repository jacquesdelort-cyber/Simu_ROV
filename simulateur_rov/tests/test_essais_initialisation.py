"""Essais comparatifs : une ligne d'initialisation est écrite dans une copie du classeur."""
from pathlib import Path
import shutil

from openpyxl import Workbook, load_workbook

import pytest

from src.utils.essais_initialisation import (
    DEFAULT_EXCEL_PATH,
    INPUT_COLUMNS,
    METRIC_COLUMNS,
    EssaisExcelError,
    run_essais_comparatifs,
)


def _fresh_campaign(dest: Path, keep_readme: bool = False):
    """Copie le classeur puis ne conserve qu'un onglet Data vide."""
    if keep_readme:
        shutil.copy(DEFAULT_EXCEL_PATH, dest)
        workbook = load_workbook(dest)
        for name in list(workbook.sheetnames):
            if name.casefold().startswith("data"):
                del workbook[name]
    else:
        workbook = Workbook()
    sheet = workbook.create_sheet("Data", 0)
    if not keep_readme and workbook.sheetnames != ["Data"]:
        for name in list(workbook.sheetnames):
            if name != "Data":
                del workbook[name]
    for column, name in enumerate((*INPUT_COLUMNS, *METRIC_COLUMNS), start=1):
        sheet.cell(1, column).value = name
    return workbook


def test_essais_init_ecrit_metriques(tmp_path):
    dest = Path(tmp_path) / "essais.xlsx"
    workbook = _fresh_campaign(dest, keep_readme=True)
    readme_before = workbook["Read me"]["B2"].value
    worksheet = workbook["Data"]
    headers = {
        worksheet.cell(1, column).value: column
        for column in range(1, worksheet.max_column + 1)
    }
    values = {
        "x": 0,
        "y": -20,
        "L_init": 35,
        "courant": 0,
        "mv_câble": 950,
    }
    for name, value in values.items():
        worksheet.cell(2, headers[name]).value = value
    workbook.save(dest)
    workbook.close()

    result = run_essais_comparatifs(dest, n_segments=40)

    assert result.failed_rows == []
    assert result.n_rows == 1

    saved = load_workbook(dest, data_only=True)
    assert saved["Read me"]["B2"].value == readme_before
    data = saved["Data"]
    saved_headers = {
        data.cell(1, column).value: column
        for column in range(1, data.max_column + 1)
    }
    row = {name: data.cell(2, saved_headers[name]).value for name in METRIC_COLUMNS}
    saved.close()

    for name, value in row.items():
        assert isinstance(value, (int, float)), name
    assert row["L_straight"] == 20
    assert row["Slack"] == 15
    assert row["Flèche"] >= 0


def _write_inputs(worksheet, row, headers, values):
    for name, value in values.items():
        worksheet.cell(row, headers[name]).value = value


def test_essais_onglets_data_lignes_vides_et_sauvegarde(tmp_path):
    dest = Path(tmp_path) / "essais.xlsx"
    workbook = _fresh_campaign(dest)
    data = workbook["Data"]
    headers = {
        data.cell(1, column).value: column
        for column in range(1, data.max_column + 1)
    }
    inputs = {
        "x": 0,
        "y": -20,
        "L_init": 35,
        "courant": 0,
        "mv_câble": 950,
    }
    _write_inputs(data, 2, headers, inputs)
    for name in METRIC_COLUMNS:
        data.cell(2, headers[name]).value = 111.0
    _write_inputs(data, 3, headers, inputs)

    other = workbook.create_sheet("Data courant")
    for column in range(1, data.max_column + 1):
        other.cell(1, column).value = data.cell(1, column).value
    _write_inputs(other, 2, headers, {**inputs, "L_init": 40})

    notes = workbook.create_sheet("Notes")
    notes["A1"] = "hors campagne"
    workbook.save(dest)
    workbook.close()

    seen = []

    def on_progress(done, total, detail=""):
        seen.append((done, total, detail))
        if done != 1:
            return
        partial = load_workbook(dest, data_only=True)
        partial_data = partial["Data"]
        partial_headers = {
            partial_data.cell(1, column).value: column
            for column in range(1, partial_data.max_column + 1)
        }
        kept = partial_data.cell(2, partial_headers["T_bat"]).value
        written = partial_data.cell(3, partial_headers["L_straight"]).value
        pending = partial["Data courant"].cell(2, partial_headers["L_straight"]).value
        partial.close()
        assert kept == 111.0
        assert written == 20
        assert pending is None

    result = run_essais_comparatifs(dest, n_segments=40, progress=on_progress)

    assert result.failed_rows == []
    assert result.n_rows == 2
    assert result.n_skipped == 1
    assert seen[0] == (0, 2, "")
    assert seen[-1][0] == 2

    saved = load_workbook(dest, data_only=True)
    saved_data = saved["Data"]
    saved_headers = {
        saved_data.cell(1, column).value: column
        for column in range(1, saved_data.max_column + 1)
    }
    assert saved_data.cell(2, saved_headers["T_bat"]).value == 111.0
    assert saved_data.cell(3, saved_headers["Slack"]).value == 15
    assert saved["Data courant"].cell(2, saved_headers["L_straight"]).value == 20
    assert saved["Data courant"].cell(2, saved_headers["Slack"]).value == 20
    assert saved["Notes"]["A1"].value == "hors campagne"
    saved.close()


def test_essais_message_erreur_se_termine_par_retour_ligne(tmp_path, monkeypatch):
    dest = Path(tmp_path) / "essais.xlsx"
    workbook = _fresh_campaign(dest)
    worksheet = workbook["Data"]
    headers = {
        worksheet.cell(1, column).value: column
        for column in range(1, worksheet.max_column + 1)
    }
    _write_inputs(
        worksheet,
        2,
        headers,
        {"x": 0, "y": -20, "L_init": 35, "courant": 0, "mv_câble": 950},
    )
    workbook.save(dest)
    workbook.close()

    def echec(*_args, **_kwargs):
        raise ValueError("courant hors plage")

    monkeypatch.setattr(
        "src.utils.essais_initialisation.compute_init_metrics",
        echec,
    )

    result = run_essais_comparatifs(dest, n_segments=4)

    assert len(result.failed_rows) == 1
    message = result.failed_rows[0]
    assert message.endswith("\n")
    assert "Onglet « Data », ligne 2" in message
    assert "x=0" in message and "y=-20" in message and "L_init=35" in message
    assert "ValueError: courant hors plage" in message

    with pytest.raises(EssaisExcelError) as caught:
        run_essais_comparatifs(tmp_path / "absent.xlsx")
    assert str(caught.value).endswith("\n")
    assert "absent.xlsx" in str(caught.value)


def test_essais_lit_valeur_calculee_des_formules(tmp_path, monkeypatch):
    dest = Path(tmp_path) / "essais.xlsx"
    workbook = _fresh_campaign(dest)
    worksheet = workbook["Data"]
    headers = {
        worksheet.cell(1, column).value: column
        for column in range(1, worksheet.max_column + 1)
    }
    base = {"x": 0, "y": -20, "L_init": 35, "courant": 0, "mv_câble": 900}
    _write_inputs(worksheet, 2, headers, base)
    _write_inputs(worksheet, 3, headers, {**base, "mv_câble": None})
    _write_inputs(worksheet, 4, headers, {**base, "mv_câble": None})
    worksheet.cell(3, headers["mv_câble"]).value = "=E2+10"
    worksheet.cell(4, headers["mv_câble"]).value = "=E3+10"
    workbook.save(dest)
    workbook.close()

    seen = []

    def faux_calcul(*_args, rho_cable, **_kwargs):
        seen.append(rho_cable)
        return {name: 0.0 for name in METRIC_COLUMNS}

    monkeypatch.setattr(
        "src.utils.essais_initialisation.compute_init_metrics",
        faux_calcul,
    )

    result = run_essais_comparatifs(dest, n_segments=4)

    assert result.failed_rows == []
    assert seen == [900.0, 910.0, 920.0]

    saved = load_workbook(dest)
    assert saved["Data"].cell(3, headers["mv_câble"]).value == "=E2+10"
    assert saved["Data"].cell(4, headers["mv_câble"]).value == "=E3+10"
    saved.close()
