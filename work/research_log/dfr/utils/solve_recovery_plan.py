import numpy as np

def solve_three_phase(x0, v0, t0, xe, ve, te, a1, a2, tol=1e-6, max_iter=1000):
    """
    3フェーズ(加速->定速->減速) もしくは (減速->定速->加速) を想定し、
    パラメータ (t_acc, t_coast, t_dec) を数値探索して
    (x0, v0, t0) から (xe, ve, te) へ到達するプランを求める。

    a1: フェーズ1 の加速度 (正 or 負)
    a2: フェーズ3 の加速度 (正 or 負)
    tol: 距離合計が誤差 tol 以内なら「解あり」とみなす
    max_iter: t_acc のスキャン分割数 (簡単実装のため)
    """
    T = te - t0
    if T <= 0:
        return None

    D = xe - x0

    # 可能な t_acc の範囲は [0, T]。
    t_acc_candidates = np.linspace(0, T, max_iter)

    for t_acc in t_acc_candidates:
        # フェーズ1 終了時の速度
        v_cruise = v0 + a1 * t_acc

        # フェーズ3 で v_cruise -> v_e
        # v_e = v_cruise + a2 * t_dec => t_dec = (v_e - v_cruise)/a2
        # a2 が 0 に近いときは処理不能なので除外
        if abs(a2) < 1e-12:
            continue

        t_dec = (ve - v_cruise) / a2

        # 時間が負だと不適
        if t_dec < 0:
            continue

        t_coast = T - t_acc - t_dec
        # 定速区間が負なら不適
        if t_coast < 0:
            continue

        # 距離を計算
        d_acc = v0 * t_acc + 0.5 * a1 * (t_acc ** 2)
        d_dec = v_cruise * t_dec + 0.5 * a2 * (t_dec ** 2)
        d_coast = v_cruise * t_coast

        total_dist = d_acc + d_coast + d_dec

        if abs(total_dist - D) < tol:
            # 解が見つかった！
            # 区間ごとの情報をまとめて返す
            t1_start = t0
            t1_end = t0 + t_acc

            x1_end = x0 + d_acc  # フェーズ1終了時点の位置

            t2_start = t1_end
            t2_end = t2_start + t_coast

            x2_end = x1_end + d_coast  # フェーズ2終了時点の位置

            t3_start = t2_end
            t3_end = te

            x3_end = x2_end + d_dec  # フェーズ3終了時点の位置(=xe)

            return [
                {
                    "t_start": t1_start,
                    "t_end":   t1_end,
                    "acc":     a1,
                    "x_start": x0,
                    "v_start": v0,
                    "x_end":   x1_end,
                    "v_end":   v_cruise,  # = v0 + a1*t_acc
                },
                {
                    "t_start": t2_start,
                    "t_end":   t2_end,
                    "acc":     0.0,
                    "x_start": x1_end,
                    "v_start": v_cruise,
                    "x_end":   x2_end,
                    "v_end":   v_cruise,
                },
                {
                    "t_start": t3_start,
                    "t_end":   t3_end,
                    "acc":     a2,
                    "x_start": x2_end,
                    "v_start": v_cruise,
                    "x_end":   x3_end,
                    "v_end":   ve,
                }
            ]

    # 見つからない場合
    return None

def bang_bang_trajectory(x0, v0, t0, xe, ve, te, a_max, a_min):
    """
    1) a_max -> 0 -> a_min (加速→定速→減速)
    2) a_min -> 0 -> a_max (減速→定速→加速)
    の2パターンを試し、先に見つかった方の解を返す。
    見つからなければ None。
    """

    # パターン(1): 加速 -> 定速 -> 減速
    #   フェーズ1 の加速度 a1 = +a_max
    #   フェーズ3 の加速度 a2 = a_min (負)
    sol1 = solve_three_phase(x0, v0, t0, xe, ve, te, a1=a_max, a2=a_min)
    if sol1 is not None:
        return {
            "pattern": "accelerate -> cruise -> decelerate",
            "segments": sol1
        }

    # パターン(2): 減速 -> 定速 -> 加速
    #   フェーズ1 の加速度 a1 = a_min (負)
    #   フェーズ3 の加速度 a2 = +a_max
    sol2 = solve_three_phase(x0, v0, t0, xe, ve, te, a1=a_min, a2=a_max)
    if sol2 is not None:
        return {
            "pattern": "decelerate -> cruise -> accelerate",
            "segments": sol2
        }

    # いずれのパターンでも解が見つからなかった
    return None

# -------------------------------------------------
# テスト実行例
if __name__ == "__main__":
    # 入力例
    x0 = 0.0    # 初期位置
    v0 = 10.0   # 初期速度
    t0 = 0.0    # 開始時刻

    xe = 200.0  # 目標位置
    ve = 5.0    # 目標速度
    te = 30.0   # 終了時刻

    a_max = 2.0   # 加速度 (正)
    a_min = -3.0  # 減速度 (負)

    result = bang_bang_3phase_two_patterns(x0, v0, t0, xe, ve, te, a_max, a_min)

    if result is None:
        print("どちらの3フェーズパターンでも到達解が見つかりませんでした。")
    else:
        print(f"パターン: {result['pattern']}")
        for i, seg in enumerate(result["segments"], start=1):
            print(f"--- Phase {i} ---")
            for k, v in seg.items():
                print(f"{k}: {v}")
            print()


def check_feasible_trajectory(x0, v0, t0, xe, ve, te):
    """
    初期状態 (x0, v0, t0) から目標状態 (xe, ve, te) へ
    固定加速値 +3 m/s² および -3 m/s² を使って到達可能かどうか判定し、
    可能な場合は各フェーズ（フェーズ1: 加減速、フェーズ2: 一定走行、フェーズ3: 反対側の加減速）
    の変遷を表示する関数です。

    ※修正点:
       - v0, ve の大小に依存せず、どちらのプロファイルも検証する
       - 両方で解が得られた場合は両方を表示する

    戻り値:
      走行可能なら True、どちらのプロファイルでも解が見つからなければ False
    """
    T = te - t0       # 全走行時間
    D = xe - x0       # 必要な走行距離

    # 数値探索のパラメータ
    resolution = 0.0005  # t1 の刻み幅
    solutions = []       # 解が見つかった場合、その情報を辞書として格納

    # --- プロファイル1: 「加速 → 一定走行 → 減速」
    # フェーズ1は a = +3, フェーズ3は a = -3 として計算
    t1_min = 0.0       # t1は0からTまで
    t1_max = T
    t1 = t1_min
    while t1 <= t1_max:
        # フェーズ1終了時の速度
        v1 = v0 + 3 * t1
        # パターン1では、通常 v1 >= ve でなければ意味がない
        if v1 < ve:
            t1 += resolution
            continue

        # フェーズ3の時間: v1から目標速度 ve まで減速 (a = -3)
        t3 = (v1 - ve) / 3
        t2 = T - t1 - t3  # 定速区間の時間
        if t2 < 0:
            t1 += resolution
            continue

        # 各フェーズの走行距離
        d1 = v0 * t1 + 0.5 * 3 * t1**2         # フェーズ1（加速）: s = v0*t1 + 0.5*a*t1²
        d2 = v1 * t2                           # フェーズ2（一定走行）
        d3 = v1 * t3 - 0.5 * 3 * t3**2           # フェーズ3（減速）: s = v1*t3 + 0.5*(-3)*t3²
        d_total = d1 + d2 + d3

        if abs(d_total - D) < 0.001:  # 許容誤差: 約1mm
            solutions.append({
                'pattern': '加速→一定走行→減速',
                't1': t1, 't2': t2, 't3': t3, 'v1': v1,
                'd1': d1, 'd2': d2, 'd3': d3,
                't_total': T, 'd_total': d_total
            })
            break  # 最初の解でループ終了
        t1 += resolution

    # --- プロファイル2: 「減速 → 一定走行 → 加速」
    # フェーズ1は a = -3, フェーズ3は a = +3 として計算
    t1_min = 0.0
    t1_max = T
    t1 = t1_min
    while t1 <= t1_max:
        # フェーズ1終了時の速度（減速の場合）
        v1 = v0 - 3 * t1
        # このパターンでは、通常 v1 <= ve でなければならない
        if v1 > ve:
            t1 += resolution
            continue

        # フェーズ3の時間: v1から目標速度 ve まで加速 (a = +3)
        t3 = (ve - v1) / 3
        t2 = T - t1 - t3  # 定速区間の時間
        if t2 < 0:
            t1 += resolution
            continue

        d1 = v0 * t1 - 0.5 * 3 * t1**2         # フェーズ1（減速）: s = v0*t1 + 0.5*(-3)*t1²
        d2 = v1 * t2                           # フェーズ2（一定走行）
        d3 = v1 * t3 + 0.5 * 3 * t3**2           # フェーズ3（加速）: s = v1*t3 + 0.5*(+3)*t3²
        d_total = d1 + d2 + d3

        if abs(d_total - D) < 0.001:
            solutions.append({
                'pattern': '減速→一定走行→加速',
                't1': t1, 't2': t2, 't3': t3, 'v1': v1,
                'd1': d1, 'd2': d2, 'd3': d3,
                't_total': T, 'd_total': d_total
            })
            break
        t1 += resolution

    if not solutions:
        print("与えられたパラメータでは、固定加減速（＋3／－3）で正確に目標状態に到達する走行プランは見つかりませんでした。")
        return False

    # 解が見つかった場合、各プロファイルごとに各フェーズの時刻、位置、速度、加速度の変遷を表示
    for sol in solutions:
        print("【走行可能】")
        print("パターン：", sol['pattern'])
        # フェーズ1
        t1 = sol['t1']
        t_phase1_start = t0
        t_phase1_end   = t0 + t1
        if sol['pattern'] == '加速→一定走行→減速':
            a_phase1 = 3
        else:
            a_phase1 = -3
        v_phase1_end   = v0 + a_phase1 * t1
        x_phase1_end   = x0 + (v0 * t1 + 0.5 * a_phase1 * t1**2)
        print("フェーズ1（{}）: t = {:.3f}～{:.3f}, 加速度 = {:.1f} m/s², 速度: {:.3f} → {:.3f}, 距離: {:.3f} m"
              .format("加速" if a_phase1 > 0 else "減速",
                      t_phase1_start, t_phase1_end,
                      a_phase1, v0, v_phase1_end, x_phase1_end - x0))
        # フェーズ2
        t2 = sol['t2']
        t_phase2_start = t_phase1_end
        t_phase2_end   = t_phase2_start + t2
        v_phase2 = v_phase1_end
        x_phase2_end   = x_phase1_end + v_phase2 * t2
        print("フェーズ2（一定走行）: t = {:.3f}～{:.3f}, 加速度 = 0 m/s², 速度: {:.3f} → {:.3f}, 距離: {:.3f} m"
              .format(t_phase2_start, t_phase2_end, v_phase2, v_phase2, x_phase2_end - x_phase1_end))
        # フェーズ3
        t3 = sol['t3']
        t_phase3_start = t_phase2_end
        t_phase3_end   = te
        if sol['pattern'] == '加速→一定走行→減速':
            a_phase3 = -3
        else:
            a_phase3 = 3
        v_phase3_end   = v_phase2 + a_phase3 * t3
        x_phase3_end   = x_phase2_end + v_phase2 * t3 + 0.5 * a_phase3 * t3**2
        print("フェーズ3（{}）: t = {:.3f}～{:.3f}, 加速度 = {:.1f} m/s², 速度: {:.3f} → {:.3f}, 距離: {:.3f} m"
              .format("減速" if a_phase3 < 0 else "加速",
                      t_phase3_start, t_phase3_end,
                      a_phase3, v_phase2, v_phase3_end, x_phase3_end - x_phase2_end))
        print("総走行時間: {:.3f} s, 総走行距離: {:.3f} m".format(T, x_phase3_end - x0))
        print("")
    return True


check_feasible_trajectory(x0=1114.1643970955297, v0=10, t0=266.1, xe=1216, ve=4, te=275.1)   
