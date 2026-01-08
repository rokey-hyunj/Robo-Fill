# main.py
from robot_env import RobotEnv, simulation_app
from car_env import CarEnv
import numpy as np
from isaacsim.sensors.camera import Camera
from markers import detect_aruco, get_customer_by_aruco
import cv2

'''
left lpg(0.9, 2.1)
left diesel(0.9, 1.8)
left gasoline(0.9, 1.5)

right lpg(0.9 -2.1)
right diesel(0.9, -1.8)
right gasoline(0.9, -1.5)

left arm(1.5, 1.3)
right arm(1.5, -1.3)
'''

def phase_logic(env, car_type):

    global car_phase, task_phase
    global direction, oil_type
    global controller, robot, y
    
    if car_phase == 1:
        target_x = 0.0
        
        if car_type == "car":
            arrived = env.car_env.move_car_vehicle(target_x_pos=target_x)
        elif car_type == "taxi":
            arrived = env.car_env.move_taxi_vehicle(target_x_pos=target_x-1)
        elif car_type == "benz":
            arrived = env.car_env.move_benz_vehicle(target_x_pos=target_x-2.9)
        else:
            print(f"[ERROR] 알 수 없는 차량 유형: {car_type}")
            return
    
        if arrived:
            rgba = camera.get_rgba()
            rgb = (rgba[:, :, :3]).astype(np.uint8)
            cv2.imwrite(f"./{car_type}.png", rgb)

            ids, corners = detect_aruco(rgb)

            if ids is not None:
                for ID in ids.flatten():
                    info = get_customer_by_aruco(int(ID))
                    if info is not None:
                        direction = info["oil_pose"]
                        oil_type = info["fuel_type"]
                        print("===================")
                        print(direction, oil_type)
                        print("===================")
            else:
                print("[WARN] No ArUco detected at frame")
            try:
                controller = getattr(env, f"{direction}_controller")
                robot = getattr(env, f"{direction}_robot")

            except AttributeError:
                print(f"[ERROR] '{direction}' 방향의 컨트롤러, 로봇 또는 주유 건({oil_type}) 객체를 찾을 수 없습니다.")
                return
            
            if direction == "left":
                if oil_type == "lpg":
                    y = 2.2
                elif oil_type == "diesel":
                    y = 1.85
                elif oil_type == "gasoline":
                    y =1.5
            elif direction == "right":
                if oil_type == "lpg":
                    y = -2.2
                elif oil_type == "diesel":
                    y = -1.85
                elif oil_type == "gasoline":
                    y =-1.5
            car_phase = 2
        return

    elif car_phase == 2:
        
        if task_phase == 1:
            goal = np.array((1.5, 1.3 if direction == 'left' else -1.3, 0.5), np.float32)
            if env.move_point(direction, goal):
                controller.reset()
                task_phase = 2

        elif task_phase == 2:
            goal = np.array((0.9, y, 0.03), np.float32)
            if env.move_point(direction, goal):
                controller.reset()
                task_phase = 3

        elif task_phase == 3:
            robot.gripper.close()
            task_phase = 4

        elif task_phase == 4:
            if car_type == "benz":
                goal = np.array((0.9, 1.5 if direction == 'left' else -1.4, 0.8), np.float32)
            else:    
                goal = np.array((0.9, 1.3 if direction == 'left' else -1.4, 0.8), np.float32)
            if env.move_point(direction, goal):
                controller.reset()
                task_phase = 5

        elif task_phase == 5:
            if car_type == "car":
                goal = np.array((1.2,-1.3,1.3), np.float32)
            elif car_type == "taxi":
                goal = np.array((1.07,-1.25,0.81), np.float32)
            elif car_type == "benz":
                goal = np.array((1.25, 1.19, 1.1), np.float32)
            
            if env.move_point(direction, goal):
                controller.reset()
                task_phase = 6

        elif task_phase == 6:
            
            goal = np.array((0.9, y, 0.5), np.float32)
            if env.move_point(direction, goal):
                controller.reset()
                task_phase = 7

        elif task_phase == 7:
            robot.gripper.open()
            car_phase = 3
        
        return

    elif car_phase == 3:
        target_x = -20 
        
        if car_type == 'benz':
            arrived = env.car_env.move_benz_vehicle(target_x_pos=target_x-100)
            arrived = env.car_env.move_taxi_vehicle(target_x_pos=target_x+22)
            arrived = env.car_env.move_car_vehicle(target_x_pos=target_x+30)
        elif car_type == 'taxi':
            arrived = env.car_env.move_taxi_vehicle(target_x_pos=target_x-100)
            arrived = env.car_env.move_car_vehicle(target_x_pos=target_x+20)
        elif car_type == 'car':
            arrived = env.car_env.move_car_vehicle(target_x_pos=target_x-100)
        else:
            print(f"[ERROR] 알 수 없는 차량 유형: {car_type}")
            return
        if arrived:
            car_phase = 4
            return
    
temp_car_env = CarEnv(stage=None)

env = RobotEnv(car_env=temp_car_env)

temp_car_env.stage = env.stage

temp_car_env.setup_car()

camera = Camera(prim_path="/World/Camera", frequency=20, resolution=(256, 256))

env.world.reset()
camera.initialize()

camera.add_motion_vectors_to_frame()

car_phase = 1
task_phase = 1
phase =1
while simulation_app.is_running():
    env.world.step()
    
    if phase ==1:
        phase_logic(env, "benz")
        if car_phase == 4 and task_phase ==7:
            phase = 2
            car_phase=1
            task_phase=1
        
    elif phase == 2:
        phase_logic(env, "taxi")
        if car_phase == 4 and task_phase ==7:
            phase = 3
            car_phase=1
            task_phase=1

    elif phase ==3:
        phase_logic(env, "car")
        if car_phase == 4 and task_phase ==7:
            phase = 2
            car_phase=1
            task_phase=1