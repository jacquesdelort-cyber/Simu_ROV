"""Hooks pytest : rapport d'exécution écrit dans ``results/`` (comme les rapports HTML)."""

from __future__ import annotations

from pathlib import Path

from src.tests.test_catalog import TEXT_REPORT_PYTEST_AGGREGATE

from tests._report_output import report_generated_first_line


def pytest_terminal_summary(terminalreporter, exitstatus, config) -> None:
    root = Path(config.rootpath)
    report_path = root / TEXT_REPORT_PYTEST_AGGREGATE
    report_path.parent.mkdir(parents=True, exist_ok=True)

    lines: list[str] = []
    lines.append(report_generated_first_line())
    lines.append("=" * 72)
    lines.append("Rapport d'exécution pytest (tests non graphiques)")
    lines.append(f"Code de sortie : {exitstatus}")
    lines.append("=" * 72)

    stats = getattr(terminalreporter, "stats", {}) or {}
    order = ("passed", "failed", "skipped", "error", "xfailed", "xpassed", "warnings")
    summary_parts: list[str] = []
    total_duration = 0.0
    for key in order:
        reports = stats.get(key)
        if not reports:
            continue
        summary_parts.append(f"{key}={len(reports)}")
        for rep in reports:
            if hasattr(rep, "duration") and rep.duration is not None:
                total_duration += float(rep.duration)
    if summary_parts:
        lines.append("Résumé : " + ", ".join(summary_parts))
        lines.append(f"Durée totale (tests) : {total_duration:.3f} s")
    else:
        lines.append("Résumé : aucun test exécuté (collecte vide ou arrêt anticipé).")
    lines.append("")

    for key in order:
        reports = stats.get(key)
        if not reports:
            continue
        lines.append(f"--- {key.upper()} ({len(reports)}) ---")
        for rep in reports:
            nodeid = getattr(rep, "nodeid", "?")
            dur = getattr(rep, "duration", None)
            if dur is not None:
                lines.append(f"  [{dur:.3f}s] {nodeid}")
            else:
                lines.append(f"  {nodeid}")
        lines.append("")

    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
