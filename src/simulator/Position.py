# 坐标类
class Position():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    
    def to_tuple(self):
        return (self.x, self.y, self.z)

    def __str__(self):
        return str((self.x, self.y, self.z))