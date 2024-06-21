import sys,glob,os
import random
import pygame
import time
import numpy as np
import threading
import math
import scipy.interpolate as scipy_interpolate
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

PIC_SIZE = 800
COEF = 25/141
HEIGHT = 50

# 渲染对象来保持和传递 PyGame 表面
class RenderObject(object):
    def __init__(self, width, height):
        init_image = np.random.randint(0, 255, (height, width, 3), dtype='uint8')
        self.surface = pygame.surfarray.make_surface(init_image.swapaxes(0, 1))

# 相机传感器回调，将相机的原始数据重塑为 2D RGB，并应用于 PyGame 表面
def pygame_callback(image, side):
    img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
    img = img[:, :, :3]
    img = img[:, :, ::-1]
    global Front
    Front = img
    renderObject.surface = pygame.surfarray.make_surface(Front.swapaxes(0, 1))

class cameraManage():
    def __init__(self, world, ego_vehicle, pygame_size):
        self.world = world
        self.cameras = {}
        self.ego_vehicle = ego_vehicle
        self.image_size_x = int(pygame_size.get("image_x"))
        self.image_size_y = int(pygame_size.get("image_y"))

    def camaraGenarate(self):
        camera_transform = (carla.Transform(carla.Location(x=self.ego_vehicle.get_transform().location.x,y=self.ego_vehicle.get_transform().location.y,z=HEIGHT), carla.Rotation(pitch=-90.0, yaw=0, roll=0)), "Front")
        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('fov', "110")
        camera_bp.set_attribute('image_size_x', str(self.image_size_x))
        camera_bp.set_attribute('image_size_y', str(self.image_size_y))
        camera = self.world.spawn_actor(camera_bp, camera_transform[0])
        self.cameras[camera_transform[1]] = camera
        return self.cameras

def line_points_move(line_points, axis, value):
    if axis == 'x':
        line_points = [carla.Location(point.x + value/COEF, point.y, point.z) for point in line_points]
    elif axis == 'y':
        line_points = [carla.Location(point.x, point.y + value/COEF, point.z) for point in line_points]
    return line_points

def convert(line_points, camera_transform):
    # PIL coordinate system to Carla coordinate system\
    result_points = []
    for point in line_points:
        x_0 = point.x
        y_0 = point.y
        x_1 = PIC_SIZE - y_0 - PIC_SIZE/2
        y_1 = x_0 - PIC_SIZE/2
        A = np.array([x_1,y_1])

        # Scale
        s_x = COEF
        s_y = COEF

        scale_matrix = np.array([[s_x, 0], 
                                    [0, s_y]])

        # Translation
        Location = camera_transform.location
        t_x = Location.x
        t_y = Location.y
        translation_matrix = np.array([[1, 0, t_x], 
                                        [0, 1, t_y], 
                                        [0, 0, 1]])

        # Rotation
        angle = np.radians(camera_transform.rotation.yaw)  # 角度转换为弧度
        rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)], 
                                    [np.sin(angle), np.cos(angle)]])

        # 缩放变换
        scaled_A = np.dot(scale_matrix, A)

        # 平移变换
        translated_A = np.dot(translation_matrix, np.append(scaled_A, 1))
        translated_A = translated_A[:2]  # 去掉齐次坐标

        # 旋转变换
        rotated_A = np.dot(rotation_matrix, translated_A)

        # Convert to ego_car coordinate system
        rotated_A = np.append(rotated_A, point.z)
        rotated_A = np.append(rotated_A, 1)
        
        result_points.append(rotated_A)

    return result_points

def draw_in_carla(world, reference_points):
    debug = world.debug
    # 绘制参考点
    for point in reference_points:
        debug.draw_point(carla.Location(x=point[0], y=point[1], z=point[2]), size=0.08, color=carla.Color(r=255, g=0, b=0), life_time=0.15)
    # 绘制连线
    for i in range(len(reference_points) - 1):
        debug.draw_line(
            carla.Location(x=reference_points[i][0], y=reference_points[i][1], z=reference_points[i][2]),
            carla.Location(x=reference_points[i + 1][0], y=reference_points[i + 1][1], z=reference_points[i + 1][2]),
            thickness=0.2,
            color=carla.Color(r=255, g=0, b=0),
            life_time = 0.15
        )

def convert_to_ego_car(line_points, ego_vehicle):
    ego_car_matrix = ego_vehicle.get_transform().get_matrix()
    ego_car_matrix = np.mat(ego_car_matrix)
    car_matrix = ego_car_matrix.I
    points_of_ego_vehicle_coordinate_system = []

    for point in line_points:
        car_P = np.dot(car_matrix, point)
        car_P = car_P[:2]
        points_of_ego_vehicle_coordinate_system.append((car_P[0,0],car_P[0,1]))

    return points_of_ego_vehicle_coordinate_system

def save_points(points, filename):
    with open(filename, 'w') as file:
        for point in points:
            file.write(f"{point.x}, {point.y}, {point.z}\n")

def save_yaws(yaws, points, filename):
    with open(filename, 'w') as file:
        for i in range(len(yaws)): 
            print(points[i])
            print(yaws[i])
            file.write(f"{points[i].x}, {point[i].y}, {point[i].z}, {yaws[i]}\n")
        length = len(points) - 1
        file.write(f"{points[length].x}, {point[length].y}, {point[length].z}\n")

def execute_function(world):
    line_points = None
    camera_transform = None
    while True:
        with lock:
            camera_transform = params['Camera_transform']
            line_points = params['Line_points']
        if not (line_points == None or len(line_points) == 0 or camera_transform == None):
            carla_points = convert(line_points, camera_transform)
            draw_in_carla(world, carla_points)
        time.sleep(0.075)

def calculate_yaws(points):
    yaws = []
    for i in range(len(points) - 1): 
        dx = points[i + 1] .x - points[i].x
        dy = points[i + 1] .y - points[i].y
        yaw = math.atan2(dy, dx)
        yaws.append(yaw)
    return yaws 

def interpolate_b_spline_path(x, y, n_path_points, degree=3):
    ipl_t = np.linspace(0.0, len(x) - 1, len(x))
    l,r=[(2,0.0)],[(2,0.0)]
    spl_i_x = scipy_interpolate.make_interp_spline(ipl_t, x, k=degree,bc_type=(l,r))
    spl_i_y = scipy_interpolate.make_interp_spline(ipl_t, y, k=degree,bc_type=(l,r))
    travel = np.linspace(0.0, len(x) - 1, n_path_points)
    return spl_i_x(travel), spl_i_y(travel)

def interpolate_path(path, sample_rate):
    if len(path) == 0:
        return path
    choices = np.arange(0,len(path),sample_rate)
    if len(path)-1 not in choices:
        choices =  np.append(choices , len(path)-1)

    way_point_x = [path[index].x for index in choices]
    way_point_y = [path[index].y for index in choices]

    n_course_point = len(path)*3
    rix, riy = interpolate_b_spline_path(way_point_x, way_point_y, n_course_point)
    new_path = np.vstack([rix,riy]).T
    return new_path

params = {
    'Line_points':None,
    'Camera_transform':None
}
lock = threading.Lock()

# 初始化视角高度变化
view_height = 0.0

# 设置缩放范围
min_view_height = -20
max_view_height = 20

if __name__ == "__main__":
    # 连接到客户端并检索世界对象
    client = carla.Client('localhost', 2000)
    client.load_world('Town01')
    world = client.get_world()

    # 获取地图的刷出点
    spawn_point = random.choice(world.get_map().get_spawn_points())

    # 生成车辆并设置自动驾驶
    vehicle_bp = world.get_blueprint_library().filter('*vehicle*').filter('vehicle.tesla.*')[0]
    ego_vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    #ego_vehicle.set_autopilot(True)
    world.get_spectator().set_transform(carla.Transform(ego_vehicle.get_transform().location+carla.Location(z=HEIGHT),carla.Rotation(pitch=-90)))

    pygame_size = {
        "image_x": PIC_SIZE,
        "image_y": PIC_SIZE
    }

    #调用cameraManage类，生成摄像头
    cameras = cameraManage(world, ego_vehicle, pygame_size).camaraGenarate()

    #采集carla世界中camera的图像
    camera = cameras.get("Front")
    camera.listen(lambda image: pygame_callback(image, 'Front'))
    camera_transform = camera.get_transform()
    # 为渲染实例化对象
    renderObject = RenderObject(pygame_size.get("image_x"), pygame_size.get("image_y"))

    # 初始化pygame显示
    pygame.init()
    gameDisplay = pygame.display.set_mode((pygame_size.get("image_x"), pygame_size.get("image_y")),
                                          pygame.HWSURFACE | pygame.DOUBLEBUF)

    line_points = []
    drawing = False

    thread = threading.Thread(target=execute_function, args=(world,))
    thread.start()
    # 循环执行
    crashed = False

    length = 0

    while not crashed:
        # 等待同步
        world.tick()
        # 按帧更新渲染的 Camera 画面
        gameDisplay.blit(renderObject.surface, (0, 0))
        # 获取 pygame 事件
        for event in pygame.event.get():
            # If the window is closed, break the while loop
            if event.type == pygame.QUIT:
                crashed = True

            key_list = pygame.key.get_pressed()

            if key_list[pygame.K_w]:
                camera_transform.location.x += 1
                line_points = line_points_move(line_points,'y',1)
            if key_list[pygame.K_s]:
                camera_transform.location.x -= 1
                line_points = line_points_move(line_points,'y',-1)
            if key_list[pygame.K_a]:
                camera_transform.location.y -= 1
                line_points = line_points_move(line_points,'x',1)
            if key_list[pygame.K_d]:
                camera_transform.location.y += 1
                line_points = line_points_move(line_points,'x',-1)
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_c:
                    carla_points = convert(line_points, camera_transform)
                    save_points(carla_points,"point_in_carla.txt")
                    ego_car_points = convert_to_ego_car(carla_points, ego_vehicle)
                    save_points(ego_car_points,"point_in_ego_car_system.txt")
                    print("data saved")
                elif event.key == pygame.K_h:
                    view_height = 0
                    camera_transform.location.z = HEIGHT
                elif event.key == pygame.K_p:
                    line_points = []
                elif event.key == pygame.K_y:
                    yaws = calculate_yaws(line_points)
                    save_yaws(yaws,line_points,"yaws.txt")
                    print("yaws saved")

            camera.set_transform(camera_transform)

            if event.type == pygame.MOUSEBUTTONDOWN and event.type != pygame.MOUSEWHEEL:
                drawing = True
            elif event.type == pygame.MOUSEBUTTONUP:
                drawing = False
                cur_len = len(line_points) - length
                if cur_len >= 0:

                    interpolated_path = interpolate_path(line_points[-cur_len:], sample_rate = 3)

                    final_interpolated_path = []
                    for point in interpolated_path:
                        point_lists = world.cast_ray(carla.Location(point[0], point[1], HEIGHT + view_height),carla.Location(point[0], point[1], -5))
                        if len(point_lists) != 0:
                            new_point = carla.Location(point[0], point[1], point_lists[0].location.z + 0.5)
                        else:
                            new_point = carla.Location(point[0], point[1], 0.5)
                        final_interpolated_path.append(new_point)

                    line_points[-cur_len:] = final_interpolated_path
                    length = len(line_points)
            elif event.type == pygame.MOUSEMOTION and drawing:
                new_pos = ((event.pos[0] - PIC_SIZE/2)*((HEIGHT + view_height)/HEIGHT) + PIC_SIZE/2 , 
                           (event.pos[1] - PIC_SIZE/2)*((HEIGHT + view_height)/HEIGHT) + PIC_SIZE/2)
                point_lists = world.cast_ray(carla.Location(new_pos[0], new_pos[1], HEIGHT + view_height),carla.Location(new_pos[0], new_pos[1], -5))

                if len(point_lists) != 0:
                    flag = False
                    for point in point_lists:
                        if point.label != "Vegetation":
                            point_xyz = carla.Location(new_pos[0], new_pos[1], point.location.z + 0.5)
                            flag = True
                            break
                    if flag == True:
                        point_xyz = carla.Location(new_pos[0], new_pos[1], 0.5)
                    print("^^^^^")
                    for point in point_lists:
                        print(point.location.z,point.label)
                else:
                    point_xyz = carla.Location(new_pos[0], new_pos[1], 0.5)
                
                line_points.append(point_xyz)

            if event.type == pygame.MOUSEWHEEL:
                # 根据鼠标滚轮的方向调整缩放因子
                if event.y == -1:  # 向上滚动（放大）
                    view_height += 0.5
                else:  # 向下滚动（缩小）
                    view_height -= 0.5
                view_height = max(min_view_height, min(view_height, max_view_height))
                camera_transform.location.z = HEIGHT + view_height

            with lock:
                params['Line_points'] = line_points
                params['Camera_transform'] = camera_transform

        pygame.display.flip()

    # 结束
    ego_vehicle.destroy()
    camera = cameras.values()
    for cam in camera:
        cam.stop
    pygame.quit()