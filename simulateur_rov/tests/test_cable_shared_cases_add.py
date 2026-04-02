"""Tests pour add_cable_to_test_cases (copie isolée du module, pas de modification du dépôt)."""

from __future__ import annotations

import importlib.util
import shutil
import sys
from pathlib import Path

import numpy as np
import pytest


@pytest.fixture
def cable_cases_module(tmp_path: Path):
    """Copie ``cable_shared_cases.py`` dans tmp et charge le module depuis ce fichier."""
    here = Path(__file__).resolve().parent
    src = here / "cable_shared_cases.py"
    dst = tmp_path / "cable_shared_cases.py"
    shutil.copy2(src, dst)
    spec = importlib.util.spec_from_file_location("cable_shared_cases_isolated", dst)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod, dst


def test_add_cable_to_test_cases_inserts_nouveau_cas(cable_cases_module):
    mod, dst = cable_cases_module
    n_before = len(mod.ALL_SHARED_CABLE_CASES)

    c = mod.add_cable_to_test_cases(
        np.array([0.0, 2.0]),
        np.array([0.0, -1.0]),
        2.5,
        x_boat=0.0,
        y_boat=0.0,
        x_rov=2.0,
        y_rov=-1.0,
        _path=dst,
    )

    assert c.name == "Nouveau cas"
    assert len(mod.ALL_SHARED_CABLE_CASES) == n_before + 1
    assert mod.ALL_SHARED_CABLE_CASES[0].name == "Nouveau cas"
    assert np.allclose(mod.ALL_SHARED_CABLE_CASES[0].x_cable, [0.0, 2.0])

    text = dst.read_text(encoding="utf-8")
    assert "Nouveau cas" in text
    assert dst.with_suffix(dst.suffix + ".bak").is_file()


def test_add_cable_to_test_cases_replaces_existing_nouveau_cas(cable_cases_module):
    mod, dst = cable_cases_module
    mod.add_cable_to_test_cases([0.0, 1.0], [0.0, 0.0], 1.0, _path=dst)
    n_after_first = len(mod.ALL_SHARED_CABLE_CASES)
    mod.add_cable_to_test_cases([0.0, 3.0], [0.0, 0.0], 3.0, _path=dst)
    assert len(mod.ALL_SHARED_CABLE_CASES) == n_after_first
    assert mod.ALL_SHARED_CABLE_CASES[0].name == "Nouveau cas"
    assert float(mod.ALL_SHARED_CABLE_CASES[0].l_target) == 3.0
