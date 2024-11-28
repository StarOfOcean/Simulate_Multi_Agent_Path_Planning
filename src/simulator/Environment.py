import json
from config import *
from src.simulator.Position import Position
from src.simulator.Task import Task, TaskPhysicalStatus, TaskState
from src.simulator.Car import Car, CarState
from src.simulator.Drone import Drone
from src.path_schedule.Path_Scheculing import CBS
from src.process.process import process_moving2loading, process_moving2leaving, process_moving2landing, process_moving2unloading
from tqdm import tqdm
import math
import simpy
# 仿真环境类
class Environment():
    def __init__(self):
        with open(config_path, 'r', encoding="utf-8") as f:
            self.config = json.load(f)
        # 创建simpy仿真环境
        self.env = simpy.Environment()
        # 上货点
        self.loading_point = Position(190, 425, -16)
        # 定义六个起飞点
        self.leaving_points = []
        self._creat_leaving_points()
        # 定义五个idle点（目前并未使用）
        self.idle_points = []
        self._creat_idle_points()
        # 定义网格，为1说明不可通行
        self.grids = []
        self._creat_grids()
        # 初始化任务，并按任务开始时间排序
        self.all_tasks = []
        self._initialize_tasks()
        self.all_tasks.sort(key=lambda x:x.ordertime)
        self.return_tasks = [] # 维护返程任务
        self.delivery_tasks = [] # 维护去程任务
        # 定义当前仿真时间
        self.T = 0
        # 初始化无人车
        self.cars = [Car(car['magvSn'], Position(car['birthplace']["x"], car['birthplace']["y"], car['birthplace']["z"]), self.T, colors[idx]) for idx, car in enumerate(self.config["taskParam"]["magvParamList"])]
        self.cars = self.cars[:3]
        self.idle_cars = [car for car in self.cars] # 维护空闲的无人车
        # 初始化无人机（暂未使用无人机）
        self.drones = [Drone(drone['droneSn'], drone['birthplace'], 0.01) for drone in self.config["taskParam"]["droneParamList"]]
        # 定义当前任务的索引
        self.current_task_idx = 0

    def run_simulation(self):
        """运行仿真"""
        for T in tqdm(range(280, 3600, 1), desc=f'\t仿真进度', ncols=100):
            if draw_map:
                for car in self.cars: 
                    car.draw_path(screen)
                
                pygame.display.flip()
                clock.tick(30) # 每秒更新30次
            logger.info(self.T)
            for car in self.cars:
                logger.info(f"{car}:{car.position},{car.state}")
            logger.info(f"{self.T} {self.env.now}")
            self.T = T
            self.update_cars_position()
            self.get_unassigned_and_pending_tasks()
            self.assign_task_to_car()
            self.find_path_for_loaded_car()

            self.env.run(until=T)

    def update_cars_position(self):
        """更新无人车的实时位置"""
        if draw_map:
            draw_grid()
            for car in self.cars:
                if self.T in car.path_dict:
                    points = [(20 * (p[0] - 179), screen_size[1] - 20 * (p[1] - 419)) for p in car.path]
                    pygame.draw.lines(screen, car.color, False, points, 2)
        for car in self.cars:
            if self.T in car.path_dict:
                car.position = Position(car.path_dict[self.T][0], car.path_dict[self.T][1], car.path_dict[self.T][2])
                if draw_map:
                    car.car_rect.center = (20 * (car.position.x - 179), screen_size[1] - 20 * (car.position.y - 419))
                    car.trail.append(car.car_rect.center) 
            else:
                car.position = Position(car.path[-1][0], car.path[-1][1], car.path[-1][2])
                if draw_map:
                    car.car_rect.center = (20 * (car.position.x - 179), screen_size[1] - 20 * (car.position.y - 419))
                    car.trail = []
            if draw_map:
                screen.blit(car.car_img, car.car_rect.topleft)
                car.car_rect.center = (20 * (car.position.x - 179), screen_size[1] - 20 * (car.position.y - 419))

        # pygame.display.flip()
        if draw_map:
            clock.tick(5)  # 每秒更新5次

    def get_unassigned_and_pending_tasks(self):
        """更新任务列表"""
        # 去除已过期的task
        for task in self.delivery_tasks:
            if self.T > task.timeout:
                task.state = TaskState.TIME_OUT
                self.delivery_tasks.remove(task)

        # 添加新task
        while self.current_task_idx < len(self.all_tasks):
            if self.all_tasks[self.current_task_idx].ordertime <= self.T:
                self.delivery_tasks.append(self.all_tasks[self.current_task_idx])
                self.current_task_idx +=1
            else:
                break
    
    def assign_task_to_car(self):
        """派发任务给无人车"""
        # 优先接返航的无人机，因为无人机悬停会耗电
        for task in self.return_tasks:
            min_arrive_time_gap = math.inf
            min_arrive_car = None
            best_path = None
            car_time_gap = False
            for car in self.cars:
                if car.state == CarState.LOADED and car.next_task: continue
                if car.state == CarState.LOADING or car.state == CarState.UNLOADING or car.state == CarState.IDLE or car.state == CarState.MOVING: continue
                if car.state == CarState.WAITING:
                    pick_path = CBS(car, car.position.to_tuple(), task.landing_position.to_tuple(), self.cars, self.T)
                    if not pick_path: continue
                    time_gap = list(pick_path.keys())[-1] - list(task.path_to_landing.keys())[-1]
                    if time_gap >= 0: car_time_gap = True
                    if abs(time_gap) < min_arrive_time_gap:
                        min_arrive_car = car
                        min_arrive_time_gap = abs(time_gap)
                        best_path = pick_path
                # elif car.state == CarState.LOADED:
                #     pick_path = calculate_path(tuple(car.path[-1]), task.landing_position.to_tuple(), self.all_cars_path, self.T)
                #     time_gap = list(pick_path.keys())[-1] - list(task.path_to_landing().keys())[-1]
                #     if time_gap >= 0: car_time_gap = True
                #     if abs(time_gap) < min_arrive_time_gap:
                #         min_arrive_car = car
                #         min_arrive_time_gap = abs(time_gap)
                #         best_path = pick_path
            if not min_arrive_car:
                continue

            if not car_time_gap:
                min_arrive_car.current_task = task
                min_arrive_car.path_dict = best_path
                min_arrive_car.path = list(best_path.values())
                min_arrive_car.task = task
                min_arrive_car.state = CarState.IDLE
                self.idle_cars.remove(min_arrive_car)
                task.car = min_arrive_car
                task.state = TaskState.FLYING_TO_LANDING
                self.return_tasks.remove(task)
                logger.error(f"{self.T}-{task.car.sn}-{best_path}")
                self.env.process(process_moving2landing(min_arrive_car, task, best_path, self))
            else:
                if min_arrive_time_gap >= 10:
                    continue
                else:
                    min_arrive_car.current_task = task
                    min_arrive_car.path_dict = best_path
                    min_arrive_car.path = list(best_path.values())
                    min_arrive_car.task = task
                    min_arrive_car.state = CarState.IDLE
                    self.idle_cars.remove(min_arrive_car)
                    task.car = min_arrive_car
                    task.state = TaskState.FLYING_TO_LANDING
                    self.return_tasks.remove(task)
                    self.env.process(process_moving2landing(min_arrive_car, task, best_path, self))

        # 对任务进行派单
        for car in self.idle_cars:
            if not self.delivery_tasks: continue
            task = self.delivery_tasks[0]
            # print("task_index", task.index)
            pick_path = CBS(car, car.position.to_tuple(), task.loading_position.to_tuple(), self.cars, self.T)
            logger.info(f"{car.sn}-{pick_path}")
            # if pick_path:
            #     print("pick_path", pick_path)
            if not pick_path: continue
            car.current_task = task
            car.path_dict = pick_path
            car.path = list(pick_path.values())
            car.state = CarState.IDLE
            car.task = task
            self.idle_cars.remove(car)
            task.car = car
            task.state = TaskState.WAITING
            self.delivery_tasks.remove(task)
            self.env.process(process_moving2loading(car, task, pick_path, self))
    
    def find_path_for_loaded_car(self):
        """为已上货的无人车规划路径"""
        for car in self.cars:
            if car.state != CarState.LOADED: continue
            if car.task.type == TaskPhysicalStatus.DELIVERY:
                if car.position.to_tuple() != (190, 425, -16):
                    raise ValueError(f"{self.T}car{car}未在loading point:{car.position} {car.path_dict}")
                task = car.current_task
                best_arrive_time = math.inf
                best_leaving_point = None
                best_path = None
                leaving_position = None
                for point in self.leaving_points:
                    delivery_path = CBS(car, car.position.to_tuple(), point.to_tuple(), self.cars, self.T)
                    logger.info(f"{car}, {point}, {delivery_path}")
                    if not delivery_path: continue
                    if list(delivery_path.keys())[-1] < best_arrive_time:
                        best_arrive_time = list(delivery_path.keys())[-1]
                        best_leaving_point = point
                        best_path = delivery_path
                        leaving_position = point
                if not best_leaving_point:
                    continue
                car.state = CarState.MOVING
                car.path_dict = best_path
                car.path = list(best_path.values())
                car.task.state = TaskState.MOVING_TO_LEAVING
                # task.land_time = list(best_path.keys())[-1]+600+600 # 后续需要修改
                task.type = TaskPhysicalStatus.RETURN
                task.path_to_leaving = best_path
                task.leaving_position = leaving_position
                self.env.process(process_moving2leaving(car, task, best_path, self))

            elif car.task.type == TaskPhysicalStatus.RETURN:
                return_path = CBS(car, car.position.to_tuple(), car.current_task.loading_position.to_tuple(), self.cars, self.T)
                if not return_path: continue
                car.state = CarState.MOVING
                car.path_dict = return_path
                car.path = list(return_path.values())
                car.task.state = TaskState.MOVING_TO_RETURN
                car.task.path_to_return = return_path
                self.env.process(process_moving2unloading(car, car.task, return_path, self))
    
    def _find_leaving_point_for_delivery_point(self, position: Position):
        """根据目标卸货点位置找到对应的起飞点"""
        print(f'{position}')
        position_tuple = (position.x, position.y, position.z)
        print(position_tuple)
        # task 1
        if position_tuple == (146, 186, -34):
            return self.leaving_points[0]
        # task 2
        elif position_tuple == (430, 184, -10):
            return self.leaving_points[1]
        # task 3
        elif position_tuple == (528, 172, -20):
            return self.leaving_points[2]
        # task 4
        elif position_tuple == (508, 514, -22):
            return self.leaving_points[3]
        # task 5
        elif position_tuple == (564, 394, -16):
            return self.leaving_points[4]
        # task 6
        elif position_tuple == (490, 390, -22):
            return self.leaving_points[5]
        else:
            raise ValueError("Invalid position")
    
    # def _find_idle_points_for_delivery_point(self, position):
    #     """根据目标卸货点位置找到对应的idle点集合"""
    #     # task 1
    #     if position == (146, 186, -34):
    #         return self.idle_points[1]
    #     # task 2
    #     elif position == (430, 184, -10):
    #         return self.idle_points[2]
    #     # task 3
    #     elif position == (528, 172, -20):
    #         return self.idle_points[3]
    #     # task 4
    #     elif position == (508, 514, -22):
    #         return self.idle_points[4]
    #     # task 5
    #     elif position == (564, 394, -16):
    #         return self.idle_points[5]
    #     # task 6
    #     elif position == (490, 390, -22):
    #         return self.idle_points[6]
    #     # 上货点
    #     elif position.x == 190 and position.y == 425 and position.z == -16:
    #         return self.idle_points[0]
    #     else:
    #         raise ValueError("Invalid position")

    def _initialize_tasks(self):
        """初始化所有任务并根据任务的ordertime的优先级添加到任务队列"""
        for carge in self.config["taskParam"]["waybillParamList"]:
            target_pos = carge['targetPosition']
            position_tuple = (target_pos['x'], target_pos['y'], target_pos['z'])
            delivery_point = Position(position_tuple[0], position_tuple[1], position_tuple[2])
            # leaving_point = self._find_leaving_point_for_delivery_point(delivery_point)
            loading_point = Position(190.0, 425.0, -16.0)
            delivery_task = Task(carge['index'], TaskPhysicalStatus.DELIVERY, delivery_point, loading_point, carge['orderTime'], carge['betterTime'], carge['timeout'])
            self.all_tasks.append(delivery_task)

    def _creat_grids(self):
        for i in range(421, 450):
            temp_grid = []
            for j in range(181, 200):
                temp_grid.append(0)
            self.grids.append(temp_grid)

        # 障碍物
        for i in range(440, 445):
            for j in range(184, 189):
                self.grids[i-421][j-181] = 1

        for i in range(440, 445):
            for j in range(194, 199):
                self.grids[i-421][j-181] = 1

        for i in range(432, 435):
            for j in range(186, 197):
                self.grids[i-421][j-181] = 1
    
    def _creat_leaving_points(self):
        leaving_point_1 = Position(190, 431, -16)  # (146, 186, -34)
        leaving_point_2 = Position(196, 431, -16)  # (430, 184, -10)
        leaving_point_3 = Position(185, 432, -16)  # (528, 172, -20)
        leaving_point_4 = Position(189, 436, -16)  # (508, 514, -22)
        leaving_point_5 = Position(183, 440, -16)  # (564, 394, -16)
        leaving_point_6 = Position(199, 440, -16)  # (490, 390, -22)
        # 添加更多起降点
        self.leaving_points.append(leaving_point_1)
        self.leaving_points.append(leaving_point_2)
        self.leaving_points.append(leaving_point_3)
        self.leaving_points.append(leaving_point_4)
        self.leaving_points.append(leaving_point_5)
        self.leaving_points.append(leaving_point_6)

    def _creat_idle_points(self):
        idle_point_1 = Position(181, 421, -16)
        idle_point_2 = Position(185, 421, -16)
        idle_point_3 = Position(190, 421, -16)
        idle_point_4 = Position(195, 421, -16)
        idle_point_5 = Position(199, 421, -16)
        self.idle_points.append(idle_point_1)
        self.idle_points.append(idle_point_2)
        self.idle_points.append(idle_point_3)
        self.idle_points.append(idle_point_4)
        self.idle_points.append(idle_point_5)

if __name__ == "__main__":
    env = Environment()
    env.run_simulation()
