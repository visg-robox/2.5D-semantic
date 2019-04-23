#!/usr/bin/env python3

# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Basic CARLA client example."""

from __future__ import print_function

import argparse
import logging
import random
import time
import os
import numpy as np

from carla.client import make_carla_client
from carla.sensor import Camera, Lidar
from carla.settings import CarlaSettings
from carla.tcp import TCPConnectionError
from carla.util import print_over_same_line
from carla.transform import Transform,Scale


def makeadir(filename):
    dirname = os.path.dirname(filename)
    if not os.path.isdir(dirname):
        os.makedirs(dirname)




def run_carla_client(args):
    # Here we will run 3 episodes with 300 frames each.
    number_of_episodes = 1
    frames_per_episode =600

    # We assume the CARLA server is already waiting for a client to connect at
    # host:port. To create a connection we can use the `make_carla_client`
    # context manager, it creates a CARLA client object and starts the
    # connection. It will throw an exception if something goes wrong. The
    # context manager makes sure the connection is always cleaned up on exit.
    with make_carla_client(args.host, args.port) as client:
        print('CarlaClient connected')

        for episode in range(0, number_of_episodes):
            # Start a new episode.



            if 1:

                # Create a CarlaSettings object. This object is a wrapper around
                # the CarlaSettings.ini file. Here we set the configuration we
                # want for the new episode.
                settings = CarlaSettings()
                settings.set(
                    SynchronousMode=True,
                    SendNonPlayerAgentsInfo=True,
                    NumberOfVehicles=0,
                    NumberOfPedestrians=0,
                    WeatherId=1,
                    QualityLevel='low')
                settings.randomize_seeds()

                # Now we want to add a couple of cameras to the player vehicle.
                # We will collect the images produced by these cameras every
                # frame.



                #add camera easily,ver1.0
                Cameralist={}
                Lidarlist={}



                def add_camera(cameraname,pitch,yaw,roll):

                    def Add_camera(full_name,pitch,yaw,roll,type):
                        camera0 = Camera(full_name)
                        # Set image resolution in pixels.
                        camera0.set_image_size(1024,1024)
                        # Set its position relative to the car in meters.
                        camera0.set_position(5,0,4)
                        camera0.set_rotation(pitch, yaw, roll)
                        camera0.set_PostProcessing(type)
                        settings.add_sensor(camera0)
                        Cameralist[full_name]=camera0


                    Add_camera(os.path.join('RGB',cameraname),pitch,yaw,roll,'None')
                    Add_camera(os.path.join('Semantic',cameraname),pitch,yaw,roll,'SemanticSegmentation')
                    # Add_camera(os.path.join('Depth',cameraname),pitch,yaw,roll,'Depth')

                    instrincs_name = args.out_filename_format.format('pose/instrincs/'+cameraname , 0)

                    cam_instrincs=Cameralist[os.path.join('RGB',cameraname)].get_instrincs()
                    makeadir(instrincs_name)
                    np.savetxt(instrincs_name, cam_instrincs)


                add_camera('Camera_F0',0,0,0)
                add_camera('Camera_F1',-90,0,0)
                add_camera('Camera_L0',0,-90,0)
                add_camera('Camera_R0',0,90,0)
                add_camera('Camera_B0',0,180,0)
                # add_camera('Camera_A',0,45,0)
                # add_camera('Camera_B',0,135,0)
                # add_camera('Camera_C',0,-45,0)
                # add_camera('Camera_D',0,-135,0)
                #add_camera('Camera_F1',0,0,0)
                #add_camera('Camera_L1',0,-90, 0)
                #add_camera('Camera_R1',0,90, 0)








                #
                #
                # # Let's add another camera producing ground-truth depth.
                # camera1 = Camera('CameraDepth', PostProcessing='Depth')
                # camera1.set_image_size(800, 600)
                # camera1.set_position(0.30, 0, 1.30)
                # settings.add_sensor(camera1)

                Lidarname='Lidar/Lidar64_0'
                lidar = Lidar(Lidarname)
                lidar.set_position(5,0,4)
                lidar.set_rotation(-80, 0, 0)
                lidar.set(
                    Channels=64,
                    Range=30,
                    PointsPerSecond=2000000,
                    RotationFrequency=10,
                    UpperFovLimit=30,
                    LowerFovLimit=-30)
                settings.add_sensor(lidar)
                Lidarlist[Lidarname]=lidar


                Lidarname = 'Lidar/Lidar64_1'
                lidar = Lidar(Lidarname)
                lidar.set_position(5, 0, 4) #(5,0,4)
                lidar.set_rotation(0, 0, 0)
                lidar.set(
                    Channels=64,
                    Range=30,
                    PointsPerSecond=2000000,
                    RotationFrequency=10,
                    UpperFovLimit=30,
                    LowerFovLimit=-30)
                settings.add_sensor(lidar)
                Lidarlist[Lidarname] = lidar




            else:

                # Alternatively, we can load these settings from a file.
                with open(args.settings_filepath, 'r') as fp:
                    settings = fp.read()

            # Now we load these settings into the server. The server replies
            # with a scene description containing the available start spots for
            # the player. Here we can provide a CarlaSettings object or a
            # CarlaSettings.ini file as string.
            scene = client.load_settings(settings)

            # Choose one player start at random.
            number_of_player_starts = len(scene.player_start_spots)
            #player_start = random.randint(0, max(0, number_of_player_starts - 1))
            player_start = 133

            # Notify the server that we want to start the episode at the
            # player_start index. This function blocks until the server is ready
            # to start the episode.
            print('Starting new episode at %r...' % scene.map_name)
            client.start_episode(player_start)




            
            
            
            # Iterate every frame in the episode.
            for frame in range(0, frames_per_episode):

                # Read the data produced by the server this frame.
                measurements, sensor_data = client.read_data()

                # Print some of the measurements.
                if frame>=50:
                    frame-=50
                    posefilename = args.out_filename_format.format('pose/txt/Lidar', frame)
                    print_and_save_measurements(measurements,posefilename)
                    # Save the images to disk if requested.
                    if args.save_images_to_disk:
                        for name, measurement in sensor_data.items():
                            extrincs_name = args.out_filename_format.format('pose/extrincs/'+name.split('/')[1], frame)
                            makeadir(extrincs_name)
                            filename = args.out_filename_format.format('Data/'+name, frame)


                            if 'Lidar' in name:
                                world_transform = Transform(
                                measurements.player_measurements.transform,'l'
                                )
                                sensor_to_car_transform=Lidarlist[name].get_unreal_transform()
                            else:
                                world_transform = Transform(
                                    measurements.player_measurements.transform, 'c'
                                )
                                sensor_to_car_transform = Cameralist[name].get_unreal_transform()

                            Camera_extrincs=Transform(Scale(x=-1))*world_transform*sensor_to_car_transform
                            np.savetxt(extrincs_name,Camera_extrincs.matrix)


                            if 'Lidar' in name:
                                measurement.apply_transform(Camera_extrincs)


                            measurement.save_to_disk(filename)




                # We can access the encoded data of a given image as numpy
                # array using its "data" property. For instance, to get the
                # depth value (normalized) at pixel X, Y
                #
                #     depth_array = sensor_data['CameraDepth'].data
                #     value_at_pixel = depth_array[Y, X]
                #

                # Now we have to send the instructions to control the vehicle.
                # If we are in synchronous mode the server will pause the
                # simulation until we send this control.

                if not args.autopilot:

                    client.send_control(
                        steer=random.uniform(-1.0, 1.0),
                        throttle=0.5,
                        brake=0.0,
                        hand_brake=False,
                        reverse=False)

                else:

                    # Together with the measurements, the server has sent the
                    # control that the in-game autopilot would do this frame. We
                    # can enable autopilot by sending back this control to the
                    # server. We can modify it if wanted, here for instance we
                    # will add some noise to the steer.

                    control = measurements.player_measurements.autopilot_control
                    #control.steer += random.uniform(-0.1, 0.1)
                    control.throttle = 0.4
                    client.send_control(control)


def  print_and_save_measurements(measurements,filename):
    folder = os.path.dirname(filename)
    if not os.path.isdir(folder):
        os.makedirs(folder)

    filename=filename+'.txt'
    with open(filename,'w') as fp:

        number_of_agents = len(measurements.non_player_agents)
        player_measurements = measurements.player_measurements
        message = 'Vehicle at ({pos_x:.1f}, {pos_y:.1f}), '
        message += 'rotation:{pitch:.2f},{roll:.2f},{yaw:.2f} '
        message += '{speed:.0f} km/h, '
        message += 'Collision: {{vehicles={col_cars:.0f}, pedestrians={col_ped:.0f}, other={col_other:.0f}}}, '
        message += '{other_lane:.0f}% other lane, {offroad:.0f}% off-road, '
        message += '({agents_num:d} non-player agents in the scene)'
        message = message.format(
            pos_x=-player_measurements.transform.location.x,
            pos_y=player_measurements.transform.location.y,
            pitch=player_measurements.transform.rotation.pitch,
            roll=player_measurements.transform.rotation.roll,
            yaw=player_measurements.transform.rotation.yaw,
            speed=player_measurements.forward_speed * 3.6, # m/s -> km/h
            col_cars=player_measurements.collision_vehicles,
            col_ped=player_measurements.collision_pedestrians,
            col_other=player_measurements.collision_other,
            other_lane=100 * player_measurements.intersection_otherlane,
            offroad=100 * player_measurements.intersection_offroad,
            agents_num=number_of_agents)
        print_over_same_line(message)


        write_message = 'x:{pos_x:.2f}\ny:{pos_y:.2f}\nz:{pos_z:.2f}\npitch:{pitch:.2f}\nroll:{roll:.2f}\nyaw:{yaw:.2f}\n'
        write_message = write_message.format(
            pos_x=-player_measurements.transform.location.x,
            pos_y=player_measurements.transform.location.y,
            pos_z=player_measurements.transform.location.z,
            pitch=player_measurements.transform.rotation.pitch,
            roll=player_measurements.transform.rotation.roll,
            yaw=player_measurements.transform.rotation.yaw,
        )
        fp.write(write_message)


def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host server (default: localhost)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '-l', '--lidar',
        action='store_true',
        help='enable Lidar')
    argparser.add_argument(
        '-q', '--quality-level',
        choices=['Low', 'Epic'],
        type=lambda s: s.title(),
        default='low',
        help='graphics quality level, a lower level makes the simulation run considerably faster.')
    argparser.add_argument(
        '-i', '--images-to-disk',
        action='store_true',
        dest='save_images_to_disk',
        help='save images (and Lidar data if active) to disk')
    argparser.add_argument(
        '-c', '--carla-settings',
        metavar='PATH',
        dest='settings_filepath',
        default='',
        help='Path to a "CarlaSettings.ini" file')

    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    args.out_filename_format = 'Data/out_lidar_vedio_test/episode_18/{:s}/{:0>6d}'

    while True:
        try:

            run_carla_client(args)

            print('Done.')
            return

        except TCPConnectionError as error:
            logging.error(error)
            time.sleep(1)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
