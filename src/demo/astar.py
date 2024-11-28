from queue import PriorityQueue
import random

# 定义方向常量
UP, DOWN, LEFT, RIGHT = (0, -1), (0, 1), (-1, 0), (1, 0)

# 定义A*算法，减少转弯次数的实现
def a_star_min_turns(start, goal, grid):
    open_set = PriorityQueue()
    open_set.put((0, start, None))  # None表示初始没有方向
    came_from = {}
    g_score = {start: 0}
    f_score = {start: manhattan_distance(start, goal)}
    
    while not open_set.empty():
        _, current, last_direction = open_set.get()

        # 如果到达目标
        if current == goal:
            return reconstruct_path(came_from, current)
        
        # 遍历四个曼哈顿方向的邻居
        for direction in [UP, DOWN, LEFT, RIGHT]:
            neighbor = (current[0] + direction[0], current[1] + direction[1])
            
            # 检查邻居是否在地图范围内且无障碍物
            if not is_valid(neighbor, grid):
                continue
            
            # 计算新路径的代价
            tentative_g_score = g_score[current] + 1
            if last_direction and last_direction != direction:
                tentative_g_score += 10  # 增加转弯惩罚
            
            # 如果找到更优的路径，或者还未访问该节点
            if tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = (current, direction)
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + manhattan_distance(neighbor, goal)
                open_set.put((f_score[neighbor], neighbor, direction))
    
    return None  # 未找到路径

# 曼哈顿距离
def manhattan_distance(point1, point2):
    return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])

# 检查是否为合法点
def is_valid(point, grid):
    x, y = point
    return 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] == 0

# 路径重建
def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current, _ = came_from[current]
        path.append(current)
    path.reverse()
    return path

import matplotlib.pyplot as plt
import numpy as np

def visualize_map_with_path(grid, path, start_coord, end_coord):
    # 将地图转换为 numpy 数组，以便使用 imshow 可视化
    grid_np = np.array(grid)
    
    # 绘制地图，设置颜色：障碍物为黑色，可通行区域为白色
    plt.imshow(grid_np, cmap="Greys", origin="upper")
    
    # 标记起点和终点
    plt.plot(start_coord[1]-181, start_coord[0]-421, marker="o", color="green", markersize=10, label="Start")  # 起点
    plt.plot(end_coord[1]-181, end_coord[0]-421, marker="o", color="red", markersize=10, label="Goal")   # 终点

    # 绘制路径
    if path:
        x_coords, y_coords = zip(*path)  # 分解路径点的x和y坐标
        plt.plot(y_coords, x_coords, marker="o", color="blue", linewidth=2, markersize=5, label="Path")
        # 标记起点和终点
        plt.plot(y_coords[0], x_coords[0], marker="o", color="green", markersize=10, label="Start")  # 起点
        plt.plot(y_coords[-1], x_coords[-1], marker="o", color="red", markersize=10, label="Goal")   # 终点

    # 添加图例和标题
    # plt.legend(loc="upper right")
    # plt.title(f"Static Map with Path: {start_coord} to {end_coord}")
    # plt.show()

import copy
def add_conflict(grid, points):
    copy_grid = copy.deepcopy(grid)
    for i in range(len(copy_grid)):
        for j in range(len(copy_grid[i])):
            if copy_grid[i][j] == 1: continue
            for p, q in points:
                p = p-421
                q = q-181
                if (i-p)**2+(j-q)**2 < 14:
                    copy_grid[i][j] = 1
                    break
    return copy_grid

def modify_point(path):
    new_path = []
    # for point1, point2 in zip(path[:-1], path[1:]):
    #     if point1[0] == point2[0]:
    #         p = point1[1]
    #         if point2[1] > p:
    #             while point2[1] >= p:
    #                 new_path.append(point1[0], p)
    #                 p+=1
    #         else:
    #             while point2[1] <= p:
    #                 new_path.append(point1[0], p)
    #                 p-=1
    for i, j in path:
        new_path.append((i+421, j+181))
    return new_path
                


# 示例应用
if __name__ == "__main__":
    # 定义地图，0表示可通行，1表示障碍物
    # 定义地图 (0: 可通行，1: 障碍物)
    grid = []
    for i in range(421, 450):
        temp_grid = []
        for j in range(181, 200):
            temp_grid.append(0)
        grid.append(temp_grid)
    
    for i in range(440, 445):
        for j in range(184, 189):
            grid[i-421][j-181] = 1

    for i in range(440, 445):
        for j in range(194, 199):
            grid[i-421][j-181] = 1

    for i in range(432, 435):
        for j in range(186, 197):
            grid[i-421][j-181] = 1

    start_point1, end_point1 = [190, 425], [190, 444]
    start_point2, end_point2 = [186, 430], [199, 431]
    start1 = (start_point1[1]-421, start_point1[0]-181)
    goal1 = (end_point1[1]-421, end_point1[0]-181)
    path1 = a_star_min_turns(start1, goal1, grid)
    start2 = (start_point2[1]-421, start_point2[0]-181)
    goal2 = (end_point2[1]-421, end_point2[0]-181)
    path2 = a_star_min_turns(start2, goal2, grid)

    # 添加网格线以方便观察
    grid_np = np.array(grid)
    x_start = 181
    y_start = 421
    # 创建图形
    plt.figure(figsize=(8, 8))
    visualize_map_with_path(grid, path1, start_point1[::-1], end_point1[::-1])
    visualize_map_with_path(grid, path2, start_point2[::-1], end_point2[::-1])
    plt.grid(True, which="both", color="black", linestyle="-", linewidth=0.5)# 设置自定义的坐标标签# 设置自定义的坐标标签
    plt.xticks(ticks=np.arange(grid_np.shape[1]), labels=[str(x_start + i) for i in range(grid_np.shape[1])], rotation=90)
    plt.yticks(ticks=np.arange(grid_np.shape[0]), labels=[str(y_start + i) for i in range(grid_np.shape[0])])
    plt.gca().invert_yaxis()  # 上下翻转y轴，使坐标和数组一致
    plt.show()


    # import json
    # with open("all_path_dict_2.json", 'r', encoding='utf-8') as f:
    #     all_path_dict = json.load(f)

    # for point1 in all_points:
    #     for point2 in all_points:
    #         if point1 == point2: continue
    #         i, j = point1
    #         p, q = point2
    #         if str(i)+","+str(j)+"|"+str(p)+","+str(q) not in all_path_dict.keys():
    #             print(point1, point2)
    #             start = (point1[0]-421, point1[1]-181)
    #             goal = (point2[0]-421, point2[1]-181)
    #             path = a_star_min_turns(start, goal, grid)
    #             path = modify_point(path)
    #             all_path_dict[str(i)+","+str(j)+"|"+str(p)+","+str(q)] = path
                

    # for point1 in all_points:
    #     for point2 in all_points:
    #         if point1 == point2: continue
    #         # if point1 in idle_point and point2 in idle_point: continue
    #         # if point1 == [421, 190] or point2 == [421, 190]: continue
    #         other_points = all_points.copy()
    #         other_points.remove(point1)
    #         other_points.remove(point2)
    #         new_grid = add_conflict(grid, other_points)
    #         start = (point1[0]-421, point1[1]-181)
    #         goal = (point2[0]-421, point2[1]-181)
    #         path = a_star_min_turns(start, goal, new_grid)
    #         visualize_map_with_path(grid, path, point1, point2, other_points)
    #         if not path:
    #             continue
    #         i, j = point1
    #         p, q = point2
    #         # plt.savefig(f"D:\Jiuzhang\wrj\pic\{str(j)},{str(i)};{str(q)},{str(p)}.png")
            
    #         path = modify_point(path)
    #         all_path_dict[str(i)+","+str(j)+"|"+str(p)+","+str(q)] = path

    


    # all_path_dict = json.dumps(all_path_dict, indent=4, ensure_ascii=False)
    # with open("all_path_dict_other.json", 'w', encoding='utf-8') as f:
    #     f.write(all_path_dict)


    # # 运行算法
    # path = a_star_min_turns(start, goal, grid)
    # print("Path with minimum turns:", path)

    # visualize_map_with_path(grid, path)
