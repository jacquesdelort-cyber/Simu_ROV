"""

Formatage de l'état dynamique pour traces (simulation, debug).

"""

from __future__ import annotations



import math

from typing import Any, Mapping



import numpy as np





def _str_etat_depuis_mapping(état: Mapping[str, Any]) -> str:

    """

    À partir d'un dictionnaire d'état, produit une ligne synthétique des grandeurs principales.



    Clés reconnues (toutes optionnelles) :



    - ``t`` : temps (s)

    - ``x_rov``, ``y_rov``, ``x_boat``, ``y_bateau`` / ``y_boat`` : positions (m) ;

      ordonnée bateau par défaut 0 si absente

    - ``vx_rov``, ``vy_rov`` : vitesse ROV (m/s)

    - ``vx_boat``, ``vy_boat`` : vitesse bateau (m/s) ; ``vy_boat`` défaut 0 si absente

    - ``L`` : longueur câble scalaire (m)

    - ``x_cable``, ``y_cable`` : polyligne (N+1) ; premier nœud = bateau, dernier = ROV.

      Sert aussi à ``L_seg`` et aux extrémités affichées ``cbeg`` / ``cend``.

    - ``T`` : tensions aux nœuds (N+1) ; sinon ``T_bateau``, ``T_rov``, ``T_max``

    """

    def _f(v: Any, nd: int = 2) -> str:

        if v is None:

            return "n/a"

        try:

            x = float(v)

            if math.isfinite(x):

                return f"{x:.{nd}f}"

        except (TypeError, ValueError):

            pass

        return "n/a"



    t = état.get("t")

    x_rov = état.get("x_rov")

    y_rov = état.get("y_rov")

    vx_rov = état.get("vx_rov")

    vy_rov = état.get("vy_rov")

    x_boat = état.get("x_boat")

    y_boat = état.get("y_bateau", état.get("y_boat", 0.0))

    vx_boat = état.get("vx_boat")

    vy_boat = état.get("vy_boat", 0.0)

    L = état.get("L")



    L_seg = état.get("L_seg")

    if L_seg is None:

        xc = état.get("x_cable")

        yc = état.get("y_cable")

        if xc is not None and yc is not None:

            xc = np.asarray(xc, dtype=float).reshape(-1)

            yc = np.asarray(yc, dtype=float).reshape(-1)

            if xc.size == yc.size and xc.size >= 2:

                L_seg = float(np.sum(np.hypot(np.diff(xc), np.diff(yc))))



    L_straight = état.get("L_straight")

    if L_straight is None and x_rov is not None and y_rov is not None and x_boat is not None:

        try:

            L_straight = float(

                math.hypot(float(x_rov) - float(x_boat), float(y_rov) - float(y_boat))

            )

        except (TypeError, ValueError):

            L_straight = None



    x_cbeg = y_cbeg = x_cend = y_cend = None

    _xc_end = état.get("x_cable")

    _yc_end = état.get("y_cable")

    if _xc_end is not None and _yc_end is not None:

        _xa = np.asarray(_xc_end, dtype=float).reshape(-1)

        _ya = np.asarray(_yc_end, dtype=float).reshape(-1)

        if _xa.size == _ya.size and _xa.size >= 1:

            x_cbeg, y_cbeg = float(_xa[0]), float(_ya[0])

            x_cend, y_cend = float(_xa[-1]), float(_ya[-1])



    T = état.get("T")

    T_bateau = état.get("T_bateau")

    T_rov = état.get("T_rov")

    T_max = état.get("T_max")

    if T is not None:

        Ta = np.asarray(T, dtype=float).reshape(-1)

        if Ta.size > 0:

            if T_bateau is None:

                T_bateau = float(Ta[0])

            if T_rov is None:

                T_rov = float(Ta[-1])

            if T_max is None:

                T_max = float(np.max(Ta))



    parts = [

        f"t={_f(t)}",

        f" ROV=({_f(x_rov)},{_f(y_rov)})",

        f" vrov=({_f(vx_rov)},{_f(vy_rov)})",

        f" bat=({_f(x_boat)},{_f(y_boat)})",

        f" vbat=({_f(vx_boat)},{_f(vy_boat)})",

        f" L={_f(L)}",

        f" L_seg={_f(L_seg)}",

        f" L_str={_f(L_straight)}",

        f" cbeg=({_f(x_cbeg)},{_f(y_cbeg)})",

        f" cend=({_f(x_cend)},{_f(y_cend)})",

        f" T0={_f(T_bateau)}",

        f" TN={_f(T_rov)}",

        f" Tmax={_f(T_max)}",

    ]

    return " ".join(parts)





def str_etat_système(system: Any, y: Any, *, t: float | None = None) -> str:

    """

    Dépaquette le vecteur d'état ``y`` (ex. ``y_current`` du thread) via ``system.unpack_state``

    et renvoie la même ligne formatée que l'ancien appel avec un dictionnaire dérivé.

    """

    x_rov, y_rov, vx_rov, vy_rov, x_boat, vx_boat, x_cable, y_cable, T, L = system.unpack_state(y)

    return _str_etat_depuis_mapping(

        {

            "t": t,

            "x_rov": x_rov,

            "y_rov": y_rov,

            "vx_rov": vx_rov,

            "vy_rov": vy_rov,

            "x_boat": x_boat,

            "y_boat": 0.0,

            "vx_boat": vx_boat,

            "vy_boat": 0.0,

            "L": L,

            "x_cable": x_cable,

            "y_cable": y_cable,

            "T": T,

        }

    )
