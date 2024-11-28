from enum import Enum
class Task:
    def __init__(self, index, task_type, delivery_point, loading_point, ordertime, bettertime=None, timeout=None, drone=None, car=None):
        self.index = index
        self.type = task_type  # DELIVERY or RETURN
        self.ordertime = ordertime # 订单出现时间
        if bettertime:
            self.bettertime = bettertime # 订单最佳送达时间
        if timeout:
            self.timeout = timeout # 订单超时时间

        self.drone = drone  # 一个Drone的对象 assigned to the task
        self.car = car  # 一个Car的对象 assigned to the task
        self.state = TaskState.UNPICKED

        self.load_time = 30 # 上货所需时间
        self.land_time = None # 到达降落点的时间，需要根据返航路径计算

        self.loading_position = loading_point # 后续如有多个上货点，需要更改
        self.leaving_position = None
        self.unloading_position = delivery_point
        self.landing_position = None

        self.path_to_leaving = None
        self.path_to_unloading = None
        self.path_to_landing = None
        self.path_to_return = None

class TaskPhysicalStatus(Enum):
    DELIVERY = 0
    RETURN = 1

class TaskState(Enum):
    UNPICKED = 1 # 待接单
    WAITING = 2 # 等待无人车来接
    MOVING_TO_LEAVING = 3 # 移动至起飞点中
    FLYING_TO_CUSTOM = 4 # 飞至客户地址中
    FLYING_TO_LANDING = 5 # 飞回降落点中
    MOVING_TO_RETURN = 6 # 返回上货点中
    FINISHED = 7 # 结束
    TIME_OUT = 8 # 超时