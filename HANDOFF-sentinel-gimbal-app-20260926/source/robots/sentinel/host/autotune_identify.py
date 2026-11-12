#!/usr/bin/env python3
"""M4-A 离线系统辨识 — 从 trace 数据拟合模型（整改版）.

纯离线：不连接硬件、不烧录、不运动、不写 PID。
仅 Fire Yaw 与 Pitch 参数空间。Pitch 保持 HARDWARE_BLOCKED。

整改要点（M4-R1/R2，独立验收报告 2026-11-01）：
- R1: 显式 schema 校验。缺失必需字段时返回结构化原因（schema_missing:<field>），
       绝不静默填 0，也绝不把 M1 字段名悄悄重释。M1→M4 转换必须走显式适配器
       adapt_m1_samples()（版本化、结构化拒绝）。
- R2: 辨识输入语义显式分类 ——
       identify_model()            输入 u=参考/设定值 → model_kind="closed_loop_reference"
                                    （参考→输出闭环黑箱，仅诊断/比较，禁止用于换 PID 仿真）
       identify_dynamics_model()   输入 u=控制输出(output_normalized) → model_kind="dynamics"
                                    （可解释受控对象等效动态，可用于候选仿真）
       数值稳健性：列尺度化 + 岭回归（λ 正则）替代裸正规方程；病态 fail-closed。
       R3: 共用 60/20/20 时序分区，训练块拟合，中间块比较所有候选延迟，
        末块只验证已选模型；分区间清除滞后历史重叠。正则化前检查秩。
        validation_residual 为末块误差；confidence 是启发式而非概率。
       激励丰富度：输入序列方差过低时拒绝（insufficient_excitation）。
       dt 校验：dt<=0、NaN/Inf、非有限观测、非有限参数全部 fail-closed。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

# ---------------------------------------------------------------------------
# Schema 定义
# ---------------------------------------------------------------------------

# M4 原生 schema（参考模型辨识必需字段）
M4_REQUIRED_FIELDS = ("target_rad", "actual_rad", "velocity_rad_s")

# M4 动力学模型额外必需字段（控制输出）
M4_DYNAMICS_FIELDS = ("output_normalized", "actual_rad", "velocity_rad_s")

# M1 trace 字段别名（仅显式适配器使用；identify_* 不做隐式映射）
M1_ALIASES = {
    "target_rad": "adopted_target_rad",     # adopted 目标（已生效）
    "actual_rad": "angle_rad",              # 实测关节角
    "velocity_rad_s": "velocity_rad_s",     # 实测角速度
    "output_normalized": "output_normalized",  # M1 同名
}


@dataclass
class IdentifiedModel:
    """二阶线性模型: θ̈ = -a·θ̇ - b·θ + c·u(t-d).

    model_kind:
        "dynamics"              — u=控制输出，可用于候选仿真（默认；手工构造模型视为此类）
        "closed_loop_reference" — u=参考/设定值，闭环黑箱，仅诊断/比较
    """

    a: float = 0.0
    b: float = 0.0
    c: float = 0.0
    delay_ticks: int = 0
    confidence: float = 0.0
    valid: bool = False
    reason: str = "not_fitted"
    fit_residual: float = 0.0
    validation_residual: float = 0.0
    n_samples: int = 0
    n_train: int = 0
    n_validation: int = 0
    model_kind: str = "dynamics"
    excitation_variance: float = 0.0
    selection_residual: float = 0.0
    selection_method: str = "common_windows_validation_mse_v1"
    delay_candidates: list[dict[str, Any]] = field(default_factory=list)
    partition: dict[str, int] = field(default_factory=dict)
    delay_identifiable: bool = False
    delay_margin: float = 0.0
    confidence_kind: str = "heuristic_not_probability"
    dt_mode: str = "per_interval_sample_dt"



# ---------------------------------------------------------------------------
# 数值与类型检查
# ---------------------------------------------------------------------------


def _check_finite(value: Any) -> bool:
    """值必须是有限实数."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    return math.isfinite(float(value))


def _first_missing(fields: tuple[str, ...], keys: set[str]) -> str | None:
    for f in fields:
        if f not in keys:
            return f
    return None


def _validate_fields(
    samples: list[dict[str, Any]], fields: tuple[str, ...]
) -> tuple[bool, str]:
    """严格校验：必需字段全部在场且为有限实数；缺失/类型错误返回结构化原因."""
    if not samples:
        return False, "schema_missing:empty_samples"

    keys = set(samples[0].keys())
    missing = _first_missing(fields, keys)
    if missing is not None:
        # 提示是否为 M1 格式（帮助调用方走显式适配器）
        if all(M1_ALIASES.get(f, f) in keys for f in fields):
            return False, (
                f"schema_missing:{missing}"
                "(m1_format_detected;use_adapt_m1_samples)"
            )
        return False, f"schema_missing:{missing}"

    for i, s in enumerate(samples):
        for f in fields:
            if not _check_finite(s.get(f)):
                return False, f"schema_invalid:{f}:sample[{i}]"
        dt = s.get("dt_s")
        if dt is not None and not _check_finite(dt):
            return False, f"schema_invalid:dt_s:sample[{i}]"
    return True, "ok"


# ---------------------------------------------------------------------------
# M1 → M4 显式适配器（M4-R1）
# ---------------------------------------------------------------------------


def adapt_m1_samples(
    samples: list[dict[str, Any]],
    required: tuple[str, ...] = M4_REQUIRED_FIELDS,
) -> tuple[list[dict[str, Any]] | None, str]:
    """版本化 M1→M4 字段适配器（只重命名，不造值）.

    M1: adopted_target_rad / angle_rad / velocity_rad_s [/ output_normalized]
    M4: target_rad        / actual_rad  / velocity_rad_s [/ output_normalized]

    返回 (adapted, reason)；适配失败返回 (None, 结构化原因)。
    适配成功 ≠ 数据具备足够激励，后续 identify_* 仍会做质量门禁。
    """
    if not samples:
        return None, "schema_missing:empty_samples"

    keys = set(samples[0].keys())
    # 校验 M1 侧字段
    for m4_name in required:
        m1_name = M1_ALIASES[m4_name]
        if m4_name not in keys and m1_name not in keys:
            return None, f"schema_missing:{m1_name}"

    adapted: list[dict[str, Any]] = []
    for i, s in enumerate(samples):
        out = dict(s)
        for m4_name, m1_name in M1_ALIASES.items():
            if m4_name in required and m4_name not in out and m1_name in out:
                out[m4_name] = out[m1_name]
        # 字段类型/有限性
        for f in required:
            if not _check_finite(out.get(f)):
                return None, f"schema_invalid:{f}:sample[{i}]"
        adapted.append(out)
    return adapted, "ok"


# ---------------------------------------------------------------------------
# 数据质量（M4-R2）
# ---------------------------------------------------------------------------

DT_MIN_S = 1e-6      # 采样周期下限（防除零）
DT_MAX_S = 0.05      # 采样周期上限
EXCITATION_VAR_MIN = 1e-6  # 激励方差下限（零激励拒绝）

_WINDOW_EXTRACT_MIN = 20   # 窗口提取下限
MIN_WINDOW = 30           # 拟合最小窗口（短于此 → window_too_short）


def _is_usable(s: dict[str, Any], fields: tuple[str, ...]) -> bool:
    """样本可用于辨识（非 stop/estop/feedback_lost 且字段有限）."""
    if s.get("stop_reason", 0) != 0:
        return False
    if s.get("estop", False):
        return False
    if s.get("feedback_lost", False):
        return False
    for f in fields:
        if not _check_finite(s.get(f)):
            return False
    return True


def _extract_windows(
    samples: list[dict[str, Any]],
    fields: tuple[str, ...],
    min_len: int = _WINDOW_EXTRACT_MIN,
) -> list[list[dict[str, Any]]]:
    windows: list[list[dict[str, Any]]] = []
    current: list[dict[str, Any]] = []
    for s in samples:
        if _is_usable(s, fields):
            current.append(s)
        else:
            if len(current) >= min_len:
                windows.append(current)
            current = []
    if len(current) >= min_len:
        windows.append(current)
    return windows


def _excitation_variance(inputs: list[float]) -> float:
    """输入序列方差（激励丰富度代理）."""
    n = len(inputs)
    if n < 2:
        return 0.0
    mean = sum(inputs) / n
    return sum((t - mean) ** 2 for t in inputs) / n


# ---------------------------------------------------------------------------
# 数值核心：尺度化岭回归（M4-R2）
# ---------------------------------------------------------------------------


def _ridge_fit(
    X: list[list[float]], y: list[float], lam: float = 1e-6
) -> tuple[list[float] | None, float]:
    """列尺度化 + 岭回归求解 β（3 维）.

    病态（主元过小/列全零/非有限结果）返回 (None, inf)。
    """
    n = len(X)
    if n < 10 or len(X[0]) != 3:
        return None, float("inf")

    dim = 3
    scale = [0.0] * dim
    for row in X:
        for j in range(dim):
            scale[j] = max(scale[j], abs(row[j]))
    if any(s < 1e-12 for s in scale):
        return None, float("inf")
    Xn = [[row[j] / scale[j] for j in range(dim)] for row in X]

    xtx = [[0.0] * dim for _ in range(dim)]
    xty = [0.0] * dim
    for i in range(n):
        for r in range(dim):
            for c in range(dim):
                xtx[r][c] += Xn[i][r] * Xn[i][c]
            xty[r] += Xn[i][r] * y[i]
    # Check information rank BEFORE ridge. A steady sinusoid spans only sin/cos;
    # regularization must not invent a unique three-parameter identification.
    g = [[value / n for value in row] for row in xtx]
    determinant = (g[0][0]*(g[1][1]*g[2][2]-g[1][2]*g[2][1])
                   - g[0][1]*(g[1][0]*g[2][2]-g[1][2]*g[2][0])
                   + g[0][2]*(g[1][0]*g[2][1]-g[1][1]*g[2][0]))
    diagonal_product = g[0][0]*g[1][1]*g[2][2]
    if (not math.isfinite(determinant) or diagonal_product <= 0
            or determinant / diagonal_product <= 1e-10):
        return None, float("inf")
    for j in range(dim):
        xtx[j][j] += lam * n

    a_mat = [row[:] for row in xtx]
    b_vec = xty[:]
    for col in range(dim):
        piv = col
        for row in range(col + 1, dim):
            if abs(a_mat[row][col]) > abs(a_mat[piv][col]):
                piv = row
        a_mat[col], a_mat[piv] = a_mat[piv], a_mat[col]
        b_vec[col], b_vec[piv] = b_vec[piv], b_vec[col]
        if abs(a_mat[col][col]) < 1e-10:
            return None, float("inf")
        for row in range(col + 1, dim):
            f = a_mat[row][col] / a_mat[col][col]
            for c in range(col, dim):
                a_mat[row][c] -= f * a_mat[col][c]
            b_vec[row] -= f * b_vec[col]
    beta_n = [0.0] * dim
    for row in range(dim - 1, -1, -1):
        s = b_vec[row]
        for c in range(row + 1, dim):
            s -= a_mat[row][c] * beta_n[c]
        beta_n[row] = s / a_mat[row][row]

    beta = [beta_n[j] / scale[j] for j in range(dim)]

    ss = 0.0
    for i in range(n):
        pred = sum(X[i][j] * beta[j] for j in range(dim))
        ss += (y[i] - pred) ** 2
    residual = math.sqrt(ss / n)
    if not all(math.isfinite(v) for v in beta) or not math.isfinite(residual):
        return None, float("inf")
    return beta, residual


def _predict_residual(
    X: list[list[float]], y: list[float], beta: list[float]
) -> float:
    """在验证集上计算残差 RMS."""
    if not X:
        return float("inf")
    ss = 0.0
    for i in range(len(X)):
        pred = sum(X[i][j] * beta[j] for j in range(3))
        ss += (y[i] - pred) ** 2
    return math.sqrt(ss / len(X))


def _estimate_delay(
    inputs: list[float], actuals: list[float], max_lag: int = 20
) -> int:
    """互相关估计纯延迟（拍数）."""
    n = len(inputs)
    if n < max_lag * 2:
        return 0
    best_lag, best_corr = 0, -2.0
    for lag in range(max_lag + 1):
        if lag >= n:
            break
        sxy = sx = sy = sx2 = sy2 = 0.0
        cnt = 0
        for i in range(n - lag):
            x, yv = inputs[i], actuals[i + lag]
            sxy += x * yv
            sx += x
            sy += yv
            sx2 += x * x
            sy2 += yv * yv
            cnt += 1
        if cnt < 5:
            continue
        mx, my = sx / cnt, sy / cnt
        vx = sx2 / cnt - mx * mx
        vy = sy2 / cnt - my * my
        if vx < 1e-12 or vy < 1e-12:
            continue
        corr = (sxy / cnt - mx * my) / math.sqrt(vx * vy)
        if corr > best_corr:
            best_corr, best_lag = corr, lag
    return best_lag


# ---------------------------------------------------------------------------
# 质量门槛
# ---------------------------------------------------------------------------

CONFIDENCE_MIN = 0.20            # 置信度低于此值不产候选
VALIDATION_RESIDUAL_MAX = 200.0  # 验证残差上限（rad/s² RMS）
FIT_RESIDUAL_MAX = 100.0


# ---------------------------------------------------------------------------
# 通用拟合内核
# ---------------------------------------------------------------------------


def _fit_second_order(
    samples: list[dict[str, Any]],
    u_field: str,
    dt_s: float,
    model_kind: str,
) -> IdentifiedModel:
    """二阶模型拟合内核（u_field 指定输入字段）."""
    m = IdentifiedModel(model_kind=model_kind)

    # 1. dt 校验
    if not _check_finite(dt_s) or dt_s <= DT_MIN_S or dt_s > DT_MAX_S:
        m.reason = f"invalid_dt_s({dt_s})"
        return m

    # 2. 逐样本 dt 校验（样本 dt 优先于全局）
    for i, s in enumerate(samples):
        d = s.get("dt_s", dt_s)
        if not _check_finite(d) or d <= DT_MIN_S or d > DT_MAX_S:
            m.reason = f"invalid_sample_dt_s:sample[{i}]({d})"
            return m

    fields = (u_field, "actual_rad", "velocity_rad_s")

    # 3. 有效窗口
    windows = _extract_windows(samples, fields)
    if not windows:
        m.reason = "no_valid_windows"
        return m
    window = max(windows, key=len)
    if len(window) < MIN_WINDOW:
        m.reason = f"window_too_short({len(window)})"
        return m

    inputs = [float(s[u_field]) for s in window]
    actuals = [float(s["actual_rad"]) for s in window]
    velocities = [float(s["velocity_rad_s"]) for s in window]

    # A fixed, bounded delay grid; never derive the search range from the
    # validation/test signal or stop at the first acceptable delay.
    n = len(window)
    max_delay = min(20, max(0, (n - 40) // 10))
    train_stop = int(n * .60)
    selection_start = train_stop + max_delay + 2
    selection_stop = int(n * .80)
    test_start = selection_stop + max_delay + 2
    test_stop = n - 1
    m.partition = dict(train_start=max_delay, train_stop=train_stop,
                       selection_start=selection_start, selection_stop=selection_stop,
                       test_start=test_start, test_stop=test_stop,
                       max_delay=max_delay, purge_ticks=max_delay+2)
    # k uses state[k], state[k+1] and input[k-delay]. Purging D+2 rows
    # keeps even those raw observations disjoint across partitions.
    train_ks = range(max_delay, train_stop)
    selection_ks = range(selection_start, selection_stop)
    test_ks = range(test_start, test_stop)
    m.n_samples = n
    m.n_train, m.n_validation = len(train_ks), len(test_ks)
    if min(len(train_ks), len(selection_ks), len(test_ks)) < 4 or len(train_ks)<10:
        m.reason = "insufficient_samples_for_split"
        return m

    exc_var = _excitation_variance(inputs[max_delay:train_stop])
    m.excitation_variance = exc_var
    if exc_var < EXCITATION_VAR_MIN:
        m.reason = f"insufficient_excitation(variance={exc_var:.2e})"
        return m
    intervals = [float(s.get("dt_s", dt_s)) for s in window]
    # Each dt belongs to the interval [k,k+1], not the next sample or a mean dt.
    obs = [(velocities[k+1]-velocities[k])/intervals[k] for k in range(n-1)]
    if not all(math.isfinite(v) for v in obs):
        m.reason = "non_finite_observation"
        return m
    # Normalization and ridge column scales use TRAINING data only.
    signal_rms = math.sqrt(sum(obs[k]**2 for k in train_ks)/len(train_ks))
    signal_scale = max(signal_rms, 1e-12)
    first_reject = ""
    eligible = []
    for delay in range(max_delay+1):
        diag = dict(delay_ticks=delay, n_train=len(train_ks),
                    n_selection=len(selection_ks), eligible=False,
                    train_rmse=None, selection_rmse=None, selection_loss=None,
                    reason="not_fitted")
        m.delay_candidates.append(diag)
        def rows(ks):
            return [[-velocities[k],-actuals[k],inputs[k-delay]] for k in ks]
        X_tr, y_tr = rows(train_ks), [obs[k] for k in train_ks]
        beta, res_tr = _ridge_fit(X_tr, y_tr)
        if beta is None:
            diag["reason"] = "ill_conditioned_regression"
        else:
            res_sel = _predict_residual(rows(selection_ks),[obs[k] for k in selection_ks],beta)
            if not all(math.isfinite(v) for v in (*beta,res_tr,res_sel)):
                diag["reason"] = "non_finite_params"
            else:
                diag.update(train_rmse=res_tr, selection_rmse=res_sel,
                            selection_loss=(res_sel/signal_scale)**2)
                if any(v<0 for v in beta):
                    diag["reason"] = "negative_params"
                elif res_tr > FIT_RESIDUAL_MAX:
                    diag["reason"] = "high_residual"
                elif res_sel > VALIDATION_RESIDUAL_MAX:
                    diag["reason"] = "selection_validation_failed"
                else:
                    diag.update(eligible=True,reason="ok")
                    eligible.append((diag["selection_loss"],delay,beta,res_tr,res_sel))
        if not diag["eligible"]:
            first_reject = first_reject or diag["reason"]
    if not eligible:
        m.reason = first_reject or "no_valid_delay_candidate"
        return m

    # All models have the same 3 regression coefficients. Error is evaluated on
    # exactly the same selection rows; lower delay breaks exact ties only.
    # Do not refit on selection data or use final holdout to choose a lag.
    eligible.sort(key=lambda row:(row[0],row[1]))
    loss, delay, beta, res_tr, res_sel = eligible[0]
    m.a,m.b,m.c = beta
    m.delay_ticks = delay
    m.fit_residual,m.selection_residual = res_tr,res_sel
    if len(eligible)>1:
        runner_loss = eligible[1][0]
        margin = (runner_loss-loss)/max(runner_loss,1e-12)
    else:
        # Conditional on positive-parameter constraints, only one model remains.
        # Its confidence is capped below, not treated as statistical certainty.
        margin = 1.0
    m.delay_margin = max(0.,min(1.,margin))
    if margin < .02:
        m.reason = "ambiguous_delay(selection_errors_indistinguishable)"
        return m
    m.delay_identifiable = True

    # A third, untouched block certifies/rejects the selected model only.
    # Failure here must NEVER trigger trying a different delay.
    X_test = [[-velocities[k],-actuals[k],inputs[k-delay]] for k in test_ks]
    res_test = _predict_residual(X_test,[obs[k] for k in test_ks],beta)
    m.validation_residual = res_test
    if not math.isfinite(res_test) or res_test > VALIDATION_RESIDUAL_MAX:
        m.reason = "validation_failed(final_holdout)"
        return m
    data_conf = min(1.,len(train_ks)/200.)
    fit_conf = max(0.,1.-res_tr/FIT_RESIDUAL_MAX)
    test_conf = max(0.,1.-res_test/VALIDATION_RESIDUAL_MAX)
    separation_conf = min(1.,m.delay_margin/.10)
    confidence = data_conf*fit_conf*test_conf*separation_conf
    if len(eligible)==1:
        confidence = min(confidence,.5)
    if confidence < CONFIDENCE_MIN:
        m.reason = "low_confidence(delay_selection)"
        return m
    m.confidence = round(confidence,4)
    m.valid,m.reason = True,"ok"
    return m


# ---------------------------------------------------------------------------
# 公开入口
# ---------------------------------------------------------------------------


def identify_model(
    samples: list[dict[str, Any]], dt_s: float = 0.001
) -> IdentifiedModel:
    """参考→输出闭环黑箱辨识（model_kind=closed_loop_reference，仅诊断）.

    输入 samples 必须含 M4 原生字段 target_rad/actual_rad/velocity_rad_s。
    M1 格式数据请先走 adapt_m1_samples()；本函数对 M1 原生字段显式拒绝。
    """
    m = IdentifiedModel(model_kind="closed_loop_reference")
    ok, why = _validate_fields(samples, M4_REQUIRED_FIELDS)
    if not ok:
        m.reason = why
        return m
    fitted = _fit_second_order(samples, "target_rad", dt_s, "closed_loop_reference")
    return fitted


def identify_dynamics_model(
    samples: list[dict[str, Any]], dt_s: float = 0.001
) -> IdentifiedModel:
    """控制输出→受控对象动力学辨识（model_kind=dynamics，可用于候选仿真）.

    输入 u=output_normalized（控制输出），与优化器仿真输入语义一致。
    """
    m = IdentifiedModel(model_kind="dynamics")
    ok, why = _validate_fields(samples, M4_DYNAMICS_FIELDS)
    if not ok:
        m.reason = why
        return m
    return _fit_second_order(samples, "output_normalized", dt_s, "dynamics")


def model_to_dict(m: IdentifiedModel) -> dict[str, Any]:
    """序列化为严格 JSON 安全 dict（无 NaN/Infinity，非有限转 None）."""

    def _f(v: float) -> Any:
        if isinstance(v, float) and not math.isfinite(v):
            return None
        return v

    return {
        "a": _f(m.a),
        "b": _f(m.b),
        "c": _f(m.c),
        "delay_ticks": m.delay_ticks,
        "confidence": _f(m.confidence),
        "valid": m.valid,
        "reason": m.reason,
        "fit_residual": _f(m.fit_residual),
        "validation_residual": _f(m.validation_residual),
        "n_samples": m.n_samples,
        "n_train": m.n_train,
        "n_validation": m.n_validation,
        "model_kind": m.model_kind,
        "excitation_variance": _f(m.excitation_variance),
        "selection_residual": _f(m.selection_residual),
        "selection_method": m.selection_method,
        "delay_candidates": m.delay_candidates,
        "partition": m.partition,
        "delay_identifiable": m.delay_identifiable,
        "delay_margin": _f(m.delay_margin),
        "confidence_kind": m.confidence_kind,
        "dt_mode": m.dt_mode,
    }


__all__ = [
    "IdentifiedModel",
    "identify_model",
    "identify_dynamics_model",
    "model_to_dict",
    "adapt_m1_samples",
    "M1_ALIASES",
    "M4_REQUIRED_FIELDS",
    "M4_DYNAMICS_FIELDS",
    "CONFIDENCE_MIN",
]
