"""Gestion simple du niveau de trace global."""

TRACE_LEVEL = 0


def set_trace_level(level: int) -> None:
    """Définit le niveau de trace global (entier)."""
    global TRACE_LEVEL
    TRACE_LEVEL = int(level)


def get_trace_level() -> int:
    """Retourne le niveau de trace global."""
    return TRACE_LEVEL


def trace_print(level: int, *args, **kwargs) -> None:
    """
    Affiche le message si level >= TRACE_LEVEL.
    Utiliser trace_print(level, "...") à la place de print.
    """
    if int(level) >= TRACE_LEVEL:
        print( *args, **kwargs)
