import glob
import os
import sys
import time

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

class Spawn(object):
    def __init__(self, world, args):
        self.vehicles_list = []
        self.walkers_list = []
        self.all_id = []
        self.client = None
        self.all_actors = [] 

        self.spawn(world, args)

    def spawn(self, world, args):
    
        vehicles_list = []
        nonvehicles_list = []
        walkers_list = []
        all_id = []
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)

        try:

            world = client.get_world()
       	    blueprints = world.get_blueprint_library().filter('vehicle.*')
            blueprintsWalkers = world.get_blueprint_library().filter('walker.pedestrian.*')

            spawn_points = world.get_map().get_spawn_points()
            number_of_spawn_points = len(spawn_points)

            if args.number_of_vehicles < number_of_spawn_points:
                random.shuffle(spawn_points)
            elif args.number_of_vehicles > number_of_spawn_points:
                msg = 'Requested %d vehicles, but could only find %d spawn points'
                logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
                args.number_of_vehicles = number_of_spawn_points

            SpawnActor = carla.command.SpawnActor
            SetAutopilot = carla.command.SetAutopilot
            FutureActor = carla.command.FutureActor

            # --------------
            # Spawn vehicles
            # --------------
            batch = []
            for n, transform in enumerate(spawn_points):
                if n >= args.number_of_vehicles:
                    break
                blueprint = random.choice(blueprints)
                if blueprint.has_attribute('color'):
                    color = random.choice(blueprint.get_attribute('color').recommended_values)
                    blueprint.set_attribute('color', color)
                if blueprint.has_attribute('driver_id'):
                    driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                    blueprint.set_attribute('driver_id', driver_id)
                blueprint.set_attribute('role_name', 'autopilot')
                batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True)))
                spawn_points.pop(0)

            for response in client.apply_batch_sync(batch):
                if response.error:
                    logging.error(response.error)
                else:
                    vehicles_list.append(response.actor_id)

            #print('Created %d npc vehicles \n' % len(vehicles_list))

            # -------------
            # Spawn Walkers
            # -------------
            # 1. take all the random locations to spawn
            spawn_points = []
            for i in range(args.number_of_walkers):
                spawn_point = carla.Transform()
                loc = world.get_random_location_from_navigation()
                if (loc != None):
                    spawn_point.location = loc
                    spawn_points.append(spawn_point)
            # 2. we spawn the walker object
            batch = []
            #walker_speed = []
            for spawn_point in spawn_points:
                walker_bp = random.choice(blueprintsWalkers)
                # set as not invincible
                if walker_bp.has_attribute('is_invincible'):
                    walker_bp.set_attribute('is_invincible', 'false')
                batch.append(SpawnActor(walker_bp, spawn_point))
            results = client.apply_batch_sync(batch, True)
            for i in range(len(results)):
                if results[i].error:
                    logging.error(results[i].error)
                else:
                    walkers_list.append({"id": results[i].actor_id})
            # 3. we spawn the walker controller
            batch = []
            walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
            for i in range(len(walkers_list)):
                batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
            results = client.apply_batch_sync(batch, True)
            for i in range(len(results)):
                if results[i].error:
                    logging.error(results[i].error)
                else:
                    walkers_list[i]["con"] = results[i].actor_id
            # 4. we put altogether the walkers and controllers id to get the objects from their id
            for i in range(len(walkers_list)):
                all_id.append(walkers_list[i]["con"])
                all_id.append(walkers_list[i]["id"])
            all_actors = world.get_actors(all_id)

            # wait for a tick to ensure client receives the last transform of the walkers we have just created
            world.wait_for_tick()

            for i in range(0, len(all_id), 2):
                # start walker
                all_actors[i].start()
                # set walk to random point
                all_actors[i].go_to_location(world.get_random_location_from_navigation())
                # max speed
                all_actors[i].set_max_speed(1 + random.random()/2)

            print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))

        finally:
            self.vehicles_list = vehicles_list
            self.walkers_list = walkers_list
            self.all_id = all_id
            self.client = client
            self.all_actors = all_actors

    def destoroy(self):

        print('\ndestroying %d vehicles' % len(self.vehicles_list))
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.vehicles_list])

        # stop walker controllers (list is [controler, actor, controller, actor ...])
        for i in range(0, len(self.all_id), 2):
            self.all_actors[i].stop()

        print('\ndestroying %d walkers' % len(self.walkers_list))
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.all_id])


if __name__ == '__main__':
    spawn()
