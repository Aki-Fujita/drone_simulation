def check_multiple_noise_effect(noiseList, eta_table):
    return any([check_single_noise_effect(noise, eta_table) for noise in noiseList])


def check_single_noise_effect(noise, eta_table):
    """
            returns if the ETAs will be blocked by the noise.
            waypoint間は等速で車が動くものと仮定している.
            """
    noise_x_range = noise["x"]
    noise_t_range = noise["t"]
    waypoint_x_list = sorted([wp["x"] for wp in eta_table])
    waypoint_eta_list = sorted([wp["eta"] for wp in eta_table])
    for i, x_coor in enumerate(waypoint_x_list):
        if i >= len(waypoint_x_list) - 1:  # 一番最後のwaypointに来た時
            return False

        x1, x2 = waypoint_x_list[i], waypoint_x_list[i+1]
        t1, t2 = waypoint_eta_list[i], waypoint_eta_list[i+1]
        # print(f"(x1,t1)=({x1}, {t1})")
        if (x1 < noise_x_range[0] and x2 < noise_x_range[0]) or (x1 > noise_x_range[1] and x2 > noise_x_range[1]):
            continue  # Both points are on the same side of the x range
        if (t1 < noise_t_range[0] and t2 < noise_t_range[0]) or (t1 > noise_t_range[1] and t2 > noise_t_range[1]):
            continue
        if x1 != x2:
            slope = (t2 - t1) / (x2 - x1)
            t_at_x_min = t1 + slope * (noise_x_range[0] - x1)
            t_at_x_max = t1 + slope * (noise_x_range[1] - x1)
            # print(f"{slope} = ({t2} - {t1}) / ({x2} - {x1})")
            # print(f"{t_at_x_min} = {t1} + {slope} * ({noise_x_range[0]} - {x1})")
            if (noise_t_range[0] <= t_at_x_min <= noise_t_range[1]) or (noise_t_range[0] <= t_at_x_max <= noise_t_range[1]):
                return True
