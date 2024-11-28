from src.simulator.Car import CarState, Car
from src.simulator.Task import TaskState
from src.path_schedule.Path_Scheculing import find_drone_path
def process_moving2loading(car, task, path, env):
    """无人车移动至上货点的进程"""
    yield env.env.timeout(list(path.keys())[-1]-list(path.keys())[0])
    if list(path.values())[-1] != (190, 425, -16):
        raise ValueError(f"path的终点不是loading point")
    car.state = CarState.LOADING
    car.car_img = Car.car_img_pic
    car.car_rect = Car.car_pic
    yield env.env.timeout(task.load_time)
    car.state = CarState.LOADED
    car.car_img = Car.car_with_drone_img_pic
    car.car_rect = Car.car_with_drone_pic
    task.state = TaskState.MOVING_TO_LEAVING

def process_moving2leaving(car, task, path, env):
    """无人车移动至无人机起飞点的进程"""
    yield env.env.timeout(list(path.keys())[-1]-list(path.keys())[0])
    car.state = CarState.WAITING
    car.car_img = Car.car_img_pic
    car.car_rect = Car.car_pic
    env.idle_cars.append(car)
    task.state = TaskState.FLYING_TO_CUSTOM
    # print(task.unloading_position.to_tuple())
    task.path_to_unloading = find_drone_path(task.leaving_position.to_tuple(), task.unloading_position.to_tuple(), env.env.now)
    yield env.env.timeout(list(task.path_to_unloading.keys())[-1]-list(task.path_to_unloading.keys())[0])
    task.state = TaskState.FLYING_TO_LANDING
    env.return_tasks.append(task)
    task.landing_position = task.leaving_position # 后续需优化
    task.path_to_landing = find_drone_path(task.unloading_position.to_tuple(), task.leaving_position.to_tuple(), env.env.now)
    task.land_time = list(task.path_to_landing.keys())[-1]
    yield env.env.timeout(list(task.path_to_landing.keys())[-1]-list(task.path_to_landing.keys())[0])
    task.state = TaskState.WAITING
    
def process_moving2landing(car, task, path, env):
    """无人车移动至无人机降落点的进程"""
    yield env.env.timeout(list(path.keys())[-1]-list(path.keys())[0])
    car.state = CarState.LOADING
    car.car_img = Car.car_img_pic
    car.car_rect = Car.car_pic
    if task.land_time > list(path.keys())[-1]:
        yield env.env.timeout(int(task.land_time-list(path.keys())[-1]+1))
        car.state = CarState.LOADED
        car.car_img = Car.car_with_drone_img_pic
        car.car_rect = Car.car_with_drone_pic
    else:
        car.state = CarState.LOADED
        car.car_img = Car.car_with_drone_img_pic
        car.car_rect = Car.car_with_drone_pic
    task.state = TaskState.MOVING_TO_RETURN

def process_moving2unloading(car, task, path, env):
    """无人车返回上货点卸货的进程"""
    yield env.env.timeout(list(path.keys())[-1]-list(path.keys())[0])
    task.state = TaskState.FINISHED
    car.state = CarState.WAITING
    car.car_img = Car.car_img_pic
    car.car_rect = Car.car_pic
    env.idle_cars.append(car)
