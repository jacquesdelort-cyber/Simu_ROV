"""Gestion simple du niveau de trace global."""
import sys

TRACE_LEVEL = 0
TRACE_FILE_HANDLE = None


def set_trace_level(level: int) -> None:
    """Définit le niveau de trace global (entier)."""
    global TRACE_LEVEL
    TRACE_LEVEL = int(level)


def set_trace_file(path) -> None:
    """Active l'écriture des traces dans un fichier."""
    global TRACE_FILE_HANDLE
    close_trace_file()
    if path:
        TRACE_FILE_HANDLE = open(path, "w", encoding="utf-8")


def close_trace_file() -> None:
    """Ferme le fichier de trace si ouvert."""
    global TRACE_FILE_HANDLE
    if TRACE_FILE_HANDLE is not None:
        try:
            TRACE_FILE_HANDLE.close()
        except Exception:
            pass
        TRACE_FILE_HANDLE = None


def get_trace_level() -> int:
    """Retourne le niveau de trace global."""
    return TRACE_LEVEL


def trace_print(level: int, *args, **kwargs) -> None:
    """
    Affiche le message si level >= TRACE_LEVEL.
    Utiliser trace_print(level, "...") à la place de print.
    """
    if int(level) >= TRACE_LEVEL:
        print(*args, **kwargs, flush=True)
        if TRACE_FILE_HANDLE is not None:
            try:
                print(*args, **kwargs, file=TRACE_FILE_HANDLE, flush=True)
            except Exception:
                pass
        # Forcer l'écriture immédiate dans le terminal pour garantir l'ordre d'affichage
        sys.stdout.flush()
        sys.stderr.flush()