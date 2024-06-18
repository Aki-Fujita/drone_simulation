def will_collide(x0, v0, t0, decel, xe, te):
    """
    ある瞬間（x0, v0, t0）から全力でteまで減速してもxeに当たってしまうかどうかを判定。当たればTrueを返す.
    """
    delta_t = te - t0
    if delta_t < 0:
        # raise ValueError("delta_t must be positive")
        return False
    # 速度が0にならない場合
    if delta_t < v0**2 / (2 * abs(decel)):
        cover_distance = v0 * delta_t - 0.5 * abs(decel) * delta_t**2
        print("cover_distance", cover_distance, x0 + cover_distance, xe)
        return x0 + cover_distance > xe
    # 速度が0になる場合
    cover_distance = v0**2 / (2 * abs(decel))
    print(x0, v0, t0, decel, xe, te, )
    return x0 + cover_distance > xe
