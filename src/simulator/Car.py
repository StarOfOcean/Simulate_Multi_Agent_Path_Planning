from config import *
from enum import Enum
# 定义小车类
class Car:
    car_pic = car_rect
    car_with_drone_pic = car_with_drone_rect
    car_img_pic = car_img
    car_with_drone_img_pic = car_with_drone_img
    def __init__(self, sn, position, T, color):
        self.sn = sn
        self.position = position
        self.path = [position.to_tuple()]  # 未来规划的将要执行的路径
        self.path_dict = {T: position.to_tuple()}  # 例如：car.path_dict = {0: (1, 1), 1: (1, 2), 2: (2, 2)}
        self.next_path = None
        self.next_path_dict = None
        self.path_turns = 0 #暂未用到

        self.current_task = None
        self.next_task = None
        self.state = CarState.WAITING

        # 无人车路径绘制相关参数
        self.color = color
        self.car_img = car_img
        self.car_rect = car_rect
        self.trail = []
    
    def draw_path(self, screen): 
        if len(self.path) > 1: 
            points = [(20 * (p[0] - 179), screen_size[1] - 20 * (p[1] - 419)) for p in self.path] 
            pygame.draw.lines(screen, self.color, False, points, 2)
    
    def __str__(self):
        return self.sn

class CarState(Enum):
    WAITING = 1 # 待派单
    IDLE = 2 # 空驶接单
    LOADING = 3 # 在上货点上货中
    LOADED = 4 # 上货完毕，等待下发路径
    MOVING = 5 # 送货中
    UNLOADING = 6 # 卸货中