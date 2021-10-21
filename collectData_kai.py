### Example program to save several sensor data including bounding box
### Sensors: RGB Camera (+BoundingBox), De[th Camera, Segmentation Camera, Lidar Camera
### By Mukhlas Adib
### 2020
### Last tested on CARLA 0.9.10.1

### CARLA Simulator is licensed under the terms of the MIT license
### For a copy, see <https://opensource.org/licenses/MIT>
### For more information about CARLA Simulator, visit https://carla.org/

import glob
import os
import sys
import time
from collectData_spawn import *

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    print('carla not found')
    pass

import carla

import argparse
import logging
import random
import queue
import numpy as np
from matplotlib import pyplot as plt
import cv2
import carla_vehicle_annotator as cva

def retrieve_data(sensor_queue, frame, timeout=5):
    while True:
        try:
            data = sensor_queue.get(True,timeout)
        except queue.Empty:
            return None
        if data.frame == frame:
            return data


save_rgb = True
save_depth = False
save_segm = True
save_lidar = False
#1 tick = 20 fps
tick_sensor = 15
img_x = 2048
img_y = 1024

save_path = 'dataset_2/town5/'

def main():

    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=60,
        type=int,
        help='number of vehicles (default: 10)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=60,
        type=int,
        help='number of walkers (default: 50)')
    argparser.add_argument(
        '-tm_p', '--tm_port',
        metavar='P',
        default=8000,
        type=int,
        help='port to communicate with TM (default: 8000)')


    args = argparser.parse_args()
    
    vehicles_list = []
    nonvehicles_list = []
    walkers_list = []
    all_id = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)


    try:

        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(2.0)
        world = client.get_world()        

        spawn = Spawn(world, args)   

        print('\nRUNNING in synchronous mode\n')
        settings = world.get_settings()
        traffic_manager.set_synchronous_mode(True)
        if not settings.synchronous_mode:
            synchronous_master = True
            settings.synchronous_mode = True
            #default delta_seconds = 0.05
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)
        else:
            synchronous_master = False

        spawn_points = world.get_map().get_spawn_points()

        # -----------------------------
        # Spawn ego vehicle and sensors
        # -----------------------------
        q_list = []
        idx = 0

        tick_queue = queue.Queue()
        world.on_tick(tick_queue.put)
        q_list.append(tick_queue)
        tick_idx = idx
        idx = idx+1

        # Spawn ego vehicle
        ego_bp = world.get_blueprint_library().find('vehicle.toyota.prius')
        ego_transform = random.choice(spawn_points)
        ego_vehicle = world.spawn_actor(ego_bp, ego_transform)
        vehicles_list.append(ego_vehicle)
        ego_vehicle.set_autopilot(True)
        print('Ego-vehicle ready')

        # Spawn RGB camera
        cam_transform = carla.Transform(carla.Location(x=2.0, z=1.2))
        cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        cam_bp.set_attribute('sensor_tick', str(tick_sensor))
        cam_bp.set_attribute('image_size_x', str(img_x))
        cam_bp.set_attribute('image_size_y', str(img_y))
        cam = world.spawn_actor(cam_bp, cam_transform, attach_to=ego_vehicle)
        nonvehicles_list.append(cam)
        cam_queue = queue.Queue()
        cam.listen(cam_queue.put)
        q_list.append(cam_queue)
        cam_idx = idx
        idx = idx+1
        print('RGB camera ready')

        # Spawn depth camera
        depth_bp = world.get_blueprint_library().find('sensor.camera.depth')
        depth_bp.set_attribute('sensor_tick', str(tick_sensor))
        depth_bp.set_attribute('image_size_x', str(img_x))
        depth_bp.set_attribute('image_size_y', str(img_y))
        depth = world.spawn_actor(depth_bp, cam_transform, attach_to=ego_vehicle)
        cc_depth_log = carla.ColorConverter.LogarithmicDepth
        nonvehicles_list.append(depth)
        depth_queue = queue.Queue()
        depth.listen(depth_queue.put)
        q_list.append(depth_queue)
        depth_idx = idx
        idx = idx+1
        print('Depth camera ready')

        # Spawn segmentation camera
        if save_segm:
            segm_bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
            segm_bp.set_attribute('sensor_tick', str(tick_sensor))
            segm_bp.set_attribute('image_size_x', str(img_x))
            segm_bp.set_attribute('image_size_y', str(img_y))
            segm_transform = carla.Transform(carla.Location(x=2.0, z=1.2))
            segm = world.spawn_actor(segm_bp, segm_transform, attach_to=ego_vehicle)
            cc_segm = carla.ColorConverter.CityScapesPalette
            nonvehicles_list.append(segm)
            segm_queue = queue.Queue()
            segm.listen(segm_queue.put)
            q_list.append(segm_queue)
            segm_idx = idx
            idx = idx+1
            print('Segmentation camera ready')

        # Spawn LIDAR sensor
        if save_lidar:
            lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('sensor_tick', str(tick_sensor))
            lidar_bp.set_attribute('channels', '64')
            lidar_bp.set_attribute('points_per_second', '1120000')
            lidar_bp.set_attribute('upper_fov', '30')
            lidar_bp.set_attribute('range', '100')
            lidar_bp.set_attribute('rotation_frequency', '20')
            lidar_transform = carla.Transform(carla.Location(x=0, z=4.0))
            lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=ego_vehicle)
            nonvehicles_list.append(lidar)
            lidar_queue = queue.Queue()
            lidar.listen(lidar_queue.put)
            q_list.append(lidar_queue)
            lidar_idx = idx
            idx = idx+1
            print('LIDAR ready')


        # Begin the loop
        time_sim = 0
        while True:
            # Extract the available data
            nowFrame = world.tick()
            #print(time_sim)
                                 
            # Check whether it's time to capture data
            if time_sim >= tick_sensor:
                data = [retrieve_data(q,nowFrame) for q in q_list]
                assert all(x.frame == nowFrame for x in data if x is not None)

                # Skip if any sensor data is not available
                if None in data:
                    continue
                
                #vehicles_raw = world.get_actors()
                vehicles_raw = world.get_actors().filter('vehicle.*') # + world.get_actors().filter('vehicle.*')
                walkers_raw = world.get_actors().filter('walker.*')

                #vehicles_raw.extend(walkers_raw)
                #print(vehicles_raw)
                snap = data[tick_idx]
                rgb_img = data[cam_idx]
                depth_img = data[depth_idx]
                segm_img = data[segm_idx]

                # Attach additional information to the snapshot
                # vehicles = cva.snap_processing(vehicles_raw, snap)
                actors = cva.snap_processing(vehicles_raw, walkers_raw, snap)
                
                # Save depth image, RGB image, and Bounding Boxes data
                if save_depth:
                    depth_img.save_to_disk(save_path + 'out_depth/%06d.png' % depth_img.frame, cc_depth_log)

                # location
                depth_location = depth.get_transform().location

                depth_meter = cva.extract_depth(depth_img)
                filtered, removed =  cva.auto_annotate(actors, cam, depth_meter, segm_img, depth_location, json_path='vehicle_class_json_file.txt')
                #filtered, removed =  cva.auto_annotate(vehicles, cam, depth_meter)
                cva.save_output(rgb_img, filtered, removed, path=save_path, save_patched=False, out_format='json')
                
                # Uncomment if you want to save the data in darknet format
                cva.save2darknet(filtered['bbox'], filtered['class'], rgb_img, data_path=save_path)

                # Save segmentation image
                if save_segm:
                    #segm_img = data[segm_idx]
                    segm_img.save_to_disk(save_path + 'out_segm/%06d.png' % segm_img.frame, cc_segm)
                    
                # Save LIDAR data
                if save_lidar:
                    lidar_data = data[lidar_idx]
                    lidar_data.save_to_disk(save_path + 'out_lidar/%06d.ply' % segm_img.frame)
                
                print("save image")
                time_sim = 0
            time_sim = time_sim + settings.fixed_delta_seconds

    finally:
        cva.save2darknet(None,None,None,data_path=save_path,save_train=True)
        try:
            cam.stop()
            depth.stop()
            spawn.destoroy()
            if save_segm:
                segm.stop()
            if save_lidar:
                lidar.stop()
        except:
            print("Simulation ended before sensors have been created")
        
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        print('destroying %d nonvehicles' % len(nonvehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in nonvehicles_list])

        time.sleep(0.5)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
