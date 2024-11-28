import os
import logging
import pygame

"""文件路径"""
root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ""))
config_path = f"{root_path}/data/config1.json"
car_pic_path = f"{root_path}/data/car.png"
car_with_drone_pic_path = f"{root_path}/data/car_with_drone.jpg"

"""仿真可视化"""
# 设置是否展示仿真过程
draw_map = True
# 加载小车图标
car_img = pygame.image.load(car_pic_path)
car_img = pygame.transform.scale(car_img, (50, 50))  # 调整小车图标大小
car_rect = car_img.get_rect()
car_with_drone_img = pygame.image.load(car_with_drone_pic_path)
car_with_drone_img = pygame.transform.scale(car_with_drone_img, (50, 50))  # 调整小车图标大小
car_with_drone_rect = car_with_drone_img.get_rect()
# 绘制网格和上货点
def draw_grid():
    screen.fill((255, 255, 255)) 
    # 绘制上货点（确保与网格节点对齐）
    cargo_rect = pygame.Rect((190 - 179 - 0.5) * 20, screen_size[1] - 20 * (425 - 419 + 0.5), 20, 20)
    pygame.draw.rect(screen, (255, 0, 0), cargo_rect)
if draw_map:
    # 初始化Pygame
    pygame.init()
    # 设置屏幕大小
    screen_size = (600, 800)
    screen = pygame.display.set_mode(screen_size)
    pygame.display.set_caption('多无人车路径规划仿真')
    # 主循环
    running = True
    clock = pygame.time.Clock()
    screen.fill((255, 255, 255))  # 填充背景颜色为白色
    draw_grid()  # 绘制网格和上货点
# 无人车路径的颜色列表
colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255), (255, 0, 255)]

"""日志设置"""
log_path = f"{root_path}/log/uav.log"
logging.basicConfig(level=logging.INFO, format="%(asctime)s-%(levelname)s-%(message)s", filename=log_path, filemode="w")
# 设置日志文件
logger = logging.getLogger(__name__)