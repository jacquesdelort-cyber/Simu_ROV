import numpy as np
import pytest

from src.utils.utils import next_point


def _ls_from_Q(Q: np.ndarray) -> tuple[np.ndarray, float]:
    Q = np.asarray(Q, dtype=float)[:, :2]
    lseg = np.linalg.norm(Q[1:] - Q[:-1], axis=1)
    ls = np.zeros(Q.shape[0], dtype=float)
    ls[1:] = np.cumsum(lseg)
    return ls, float(ls[-1])


def _point_at_s(Q: np.ndarray, ls: np.ndarray, seg_idx: int, s: float) -> np.ndarray:
    Q = np.asarray(Q, dtype=float)[:, :2]
    seg_start = float(ls[seg_idx])
    seg_end = float(ls[seg_idx + 1])
    seg_len = seg_end - seg_start
    if abs(seg_len) <= 1e-15:
        return Q[seg_idx].copy()
    t = (float(s) - seg_start) / seg_len
    t = float(np.clip(t, 0.0, 1.0))
    return (1.0 - t) * Q[seg_idx] + t * Q[seg_idx + 1]


def _expected_next_point(Q: np.ndarray, ls: np.ndarray, Lseg_total: float, s_R: float, step: float) -> tuple[int | None, np.ndarray | None]:
    s_target = float(s_R) + float(step)
    if s_target > float(Lseg_total):
        return None, None
    if s_target <= float(ls[0]):
        return 0, np.asarray(Q[0], dtype=float)[:2].copy()
    if s_target >= float(ls[-1]):
        return Q.shape[0] - 2, np.asarray(Q[-1], dtype=float)[:2].copy()
    n_seg = Q.shape[0] - 1
    ns = int(np.searchsorted(ls, s_target, side="right") - 1)
    ns = max(0, min(ns, n_seg - 1))
    return ns, _point_at_s(Q, ls, ns, s_target)


def test_next_point_on_first_segment_halfway():
    Q = np.array([[0.0, 0.0], [2.0, -0.0], [4.0, -1.0]])
    ls, L_total = _ls_from_Q(Q)

    num_seg_R = 0
    s_R = float(ls[num_seg_R]) + 0.5 * (float(ls[num_seg_R + 1]) - float(ls[num_seg_R]))
    step = 0.3 * (float(ls[num_seg_R + 1]) - float(ls[num_seg_R]))
    R_pt = _point_at_s(Q, ls, num_seg_R, s_R)

    ns, T = next_point(Q, ls, L_total, R_pt, num_seg_R=num_seg_R, s_R=s_R, step=step)
    exp_ns, exp_T = _expected_next_point(Q, ls, L_total, s_R=s_R, step=step)

    assert ns == exp_ns
    assert np.allclose(T, exp_T, atol=1e-12)


def test_next_point_crosses_multiple_segments():
    Q = np.array([[0.0, 0.0], [1.0, -1.0], [2.0, -2.0], [4.0, -2.0]])
    ls, L_total = _ls_from_Q(Q)

    num_seg_R = 0
    # Placer R proche de la fin du segment 0
    s_R = float(ls[0]) + 0.8 * (float(ls[1]) - float(ls[0]))
    step = (float(ls[1]) - float(ls[0])) * 0.5  # devrait aller dans le segment 1
    R_pt = _point_at_s(Q, ls, num_seg_R, s_R)

    ns, T = next_point(Q, ls, L_total, R_pt, num_seg_R=num_seg_R, s_R=s_R, step=step)
    exp_ns, exp_T = _expected_next_point(Q, ls, L_total, s_R=s_R, step=step)

    assert ns == exp_ns
    assert np.allclose(T, exp_T, atol=1e-12)


def test_next_point_with_scurv_offset_consistency():
    Q = np.array([[0.0, 0.0], [1.0, -0.5], [3.0, -1.5], [4.5, -1.6]])
    ls, L_total = _ls_from_Q(Q)

    num_seg_R = 1
    s_R = float(ls[num_seg_R]) + 0.25 * (float(ls[num_seg_R + 1]) - float(ls[num_seg_R]))
    step = -0.15 * (float(ls[num_seg_R + 1]) - float(ls[num_seg_R]))
    R_pt = _point_at_s(Q, ls, num_seg_R, s_R)

    ns, T = next_point(Q, ls, L_total, R_pt, num_seg_R=num_seg_R, s_R=s_R, step=step)
    exp_ns, exp_T = _expected_next_point(Q, ls, L_total, s_R=s_R, step=step)

    assert ns == exp_ns
    assert np.allclose(T, exp_T, atol=1e-12)


def test_next_point_clamps_outside_range():
    Q = np.array([[0.0, 0.0], [1.0, -1.0], [2.0, -1.0]])
    ls, L_total = _ls_from_Q(Q)

    num_seg_R = 0
    s_R = float(ls[num_seg_R])  # au début
    R_pt = Q[0].copy()

    # step négatif => on clamp à gauche (dans notre implémentation)
    ns1, T1 = next_point(Q, ls, L_total, R_pt, num_seg_R=num_seg_R, s_R=s_R, step=-10.0)
    assert ns1 == 0
    assert np.allclose(T1, Q[0], atol=1e-12)

    # step trop grand => (None, None)
    ns2, T2 = next_point(Q, ls, L_total, R_pt, num_seg_R=num_seg_R, s_R=s_R, step=10.0)
    assert ns2 is None
    assert T2 is None


def test_next_point_rejects_invalid_inputs():
    Q = np.array([[0.0, 0.0], [1.0, -1.0]])
    ls = [0.0, 1.0, 2.0]  # mauvais nombre
    with pytest.raises(ValueError):
        next_point(Q, ls, Lseg_total=1.0, R=Q[0], num_seg_R=0, s_R=0.0, step=0.5)

