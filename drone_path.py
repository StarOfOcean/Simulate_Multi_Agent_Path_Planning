def find_drone_path(start_point, target_point, T):
    dist = abs(start_point[0]-target_point[0])+abs(start_point[1]-target_point[1])+abs(start_point[2]-target_point[2])
    path_dict = {}
    path = plan_manhattan_path_z_priority(start_point, target_point)
    for i in range(len(path)):
        path_dict[T+0.1*i] = path[i]
    return path_dict
    
def plan_manhattan_path_z_priority(start_position, target_position):
    """
    规划无人机最短曼哈顿路径，优先沿 z 轴移动，然后按 x 和 y 移动
    :param start_position: 起点 (x1, y1, z1)
    :param target_position: 终点 (x2, y2, z2)
    :return: 最短路径点序列（列表）
    """
    # 起点与终点
    x1, y1, z1 = start_position
    x2, y2, z2 = target_position

    # 初始化路径
    path = [(x1, y1, z1)]

    # 1. 优先沿 z 轴移动
    while z1 != z2:
        z1 += 1 if z2 > z1 else -1
        path.append((x1, y1, z1))

    # 2. 再沿 x 轴移动
    while x1 != x2:
        x1 += 1 if x2 > x1 else -1
        path.append((x1, y1, z1))

    # 3. 最后沿 y 轴移动
    while y1 != y2:
        y1 += 1 if y2 > y1 else -1
        path.append((x1, y1, z1))
    return path
