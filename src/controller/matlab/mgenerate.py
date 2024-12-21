#!/usr/bin/env python3

# Copyright 1996-2024 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Generate the matlab files.

Execute with '-update' to update the list of generated functions"""

import os
import re
import sys

PROC = 0
FUNC = 1
FOLDER = os.path.join(os.path.normpath(os.environ['WEBOTS_HOME']), 'lib', 'controller', 'matlab')
GITIGNORE = os.path.join(FOLDER, '.gitignore')


class MatlabApiGenerator:
    def __init__(self, update):
        self.update = update
        if self.update:
            if os.path.exists(GITIGNORE):
                os.remove(GITIGNORE)
            self.gitignore_file = open(GITIGNORE, 'w')
        else:
            self.gitignore_file = None

    def close(self):
        if self.gitignore_file:
            self.gitignore_file.close()

    def gen_with_doc(self, type, line, doc_url=None):
        match = re.match(r'(.*)\((.*?)\)', line)
        assert match
        function = match.group(1)
        arguments = match.group(2)
        with open(os.path.join(FOLDER, function + '.m'), 'w', newline='\n') as file:
            result = 'result = ' if type == FUNC else ''
            file.write('function %s%s(%s)\n' % (result, function, arguments))
            file.write('%% Usage: %s(%s)\n' % (function, arguments))
            file.write('% Matlab API for Webots\n')
            if doc_url is not None:
                file.write(
                    '%% Online documentation is available <a href=\"%s\">here</a>\n' % doc_url)
            file.write('\n')
            file.write("%scalllib('libController', '%s'%s%s);\n" %
                       (result, function, ', ' if arguments else '', arguments))
            if self.update:
                self.gitignore_file.write(function + '.m\n')

    def gen(self, type, line, doc_page=None):
        self.gen_with_doc(type, line, 'https://www.cyberbotics.com/doc/reference/' +
                          doc_page if doc_page is not None else None)

    def gen_const(self, name, value):
        with open(os.path.join(FOLDER, name + '.m'), 'w', newline='\n') as file:
            file.write('function value = %s\n' % name)
            file.write('value = %s;\n' % value)
            if self.update:
                self.gitignore_file.write(name + '.m\n')

    def gen_consts_from_list(self, list):
        items = list.split(',')
        for i in range(len(items)):
            self.gen_const(items[i].strip(), i)


def main(args=None):
    if args:
        update = (args[0] == "-update")
    else:
        update = False

    generator = MatlabApiGenerator(update)
    # accelerometer.h
    generator.gen(PROC, "wb_accelerometer_disable(tag)", "accelerometer")
    generator.gen(PROC, "wb_accelerometer_enable(tag, sampling_period))", "accelerometer")
    # generator.gen(FUNC, "wb_accelerometer_get_lookup_table(tag)", "accelerometer")
    # generator.gen(FUNC, "wb_accelerometer_get_lookup_table_size(tag)", "accelerometer")
    # generator.gen(FUNC, "wb_accelerometer_get_values(tag)", "accelerometer")
    generator.gen(FUNC, "wb_accelerometer_get_sampling_period(tag)", "accelerometer")

    # altimeter.h
    generator.gen(PROC, "wb_altimeter_disable(tag)", "altimeter")
    generator.gen(PROC, "wb_altimeter_enable(tag, sampling_period)", "altimeter")
    generator.gen(FUNC, "wb_altimeter_get_value(tag)", "altimeter")
    generator.gen(FUNC, "wb_altimeter_get_sampling_period(tag)", "altimeter")

    # brake.h
    generator.gen(FUNC, "wb_brake_get_motor(tag)", "brake")
    generator.gen(FUNC, "wb_brake_get_position_sensor(tag)", "brake")
    generator.gen(FUNC, "wb_brake_get_type(tag)", "brake")
    generator.gen(PROC, "wb_brake_set_damping_constant(tag, damping_constant)", "brake")

    # camera.h
    generator.gen(PROC, "wb_camera_disable(tag)", "camera")
    generator.gen(PROC, "wb_camera_enable(tag, sampling_period)", "camera")
    generator.gen(FUNC, "wb_camera_get_exposure(tag)", "camera")
    generator.gen(FUNC, "wb_camera_get_focal_distance(tag)", "camera")
    generator.gen(FUNC, "wb_camera_get_focal_length(tag)", "camera")
    generator.gen(FUNC, "wb_camera_get_fov(tag)", "camera")
    generator.gen(FUNC, "wb_camera_get_height(tag)", "camera")
    # generator.gen(FUNC, "wb_camera_get_image(tag)", "camera")
    generator.gen(FUNC, "wb_camera_get_max_focal_distance(tag)", "camera")
    generator.gen(FUNC, "wb_camera_get_max_fov(tag)", "camera")
    generator.gen(FUNC, "wb_camera_get_min_focal_distance(tag)", "camera")
    generator.gen(FUNC, "wb_camera_get_min_fov(tag)", "camera")
    generator.gen(FUNC, "wb_camera_get_near(tag)", "camera")
    generator.gen(FUNC, "wb_camera_get_sampling_period(tag)", "camera")
    generator.gen(FUNC, "wb_camera_get_width(tag)", "camera")
    generator.gen(FUNC, "wb_camera_has_recognition(tag)", "camera")
    generator.gen(PROC, "wb_camera_recognition_disable(tag)", "camera")
    generator.gen(PROC, "wb_camera_recognition_disable_segmentation(tag)", "camera")
    generator.gen(PROC, "wb_camera_recognition_enable(tag, sampling_period)", "camera")
    generator.gen(PROC, "wb_camera_recognition_enable_segmentation(tag)", "camera")
    generator.gen(FUNC, "wb_camera_recognition_get_number_of_objects(tag)", "camera")
    #  generator.gen(FUNC, "wb_camera_recognition_get_objects(tag)", "camera")
    generator.gen(FUNC, "wb_camera_recognition_get_sampling_period(tag)", "camera")
    # generator.gen(FUNC, "wb_camera_recognition_get_segmentation_image(tag)", "camera")")
    generator.gen(FUNC, "wb_camera_recognition_has_segmentation(tag)", "camera")
    generator.gen(PROC, "wb_camera_recognition_is_segmentation_enabled(tag)", "camera")
    generator.gen(FUNC, "wb_camera_recognition_save_segmentation_image(tag, filename, quality)", "camera")
    generator.gen(FUNC, "wb_camera_save_image(tag, filename, quality)", "camera")
    generator.gen(PROC, "wb_camera_set_exposure(tag, exposure)", "camera")
    generator.gen(PROC, "wb_camera_set_focal_distance(tag, focal_distance)", "camera")
    generator.gen(PROC, "wb_camera_set_fov(tag, fov)", "camera")

    # compass.h
    generator.gen(PROC, "wb_compass_disable(tag)", "compass")
    generator.gen(PROC, "wb_compass_enable(tag, sampling_period)", "compass")
    # generator.gen(FUNC, "wb_compass_get_lookup_table(tag)", "compass")
    # generator.gen(FUNC, "wb_compass_get_lookup_table_size(tag)", "compass")
    generator.gen(FUNC, "wb_compass_get_sampling_period(tag)", "compass")
    # generator.gen(FUNC, "wb_compass_get_values(tag)", "compass")

    # connector.h
    generator.gen(PROC, "wb_connector_disable_presence(tag)", "connector")
    generator.gen(PROC, "wb_connector_enable_presence(tag, sampling_period)", "connector")
    generator.gen(FUNC, "wb_connector_get_presence(tag)", "connector")
    generator.gen(FUNC, "wb_connector_get_presence_sampling_period(tag)", "connector")
    generator.gen(FUNC, "wb_connector_is_locked(tag)", "connector")
    generator.gen(PROC, "wb_connector_lock(tag)", "connector")
    generator.gen(PROC, "wb_connector_unlock(tag)", "connector")

    # console.h
    generator.gen_with_doc(PROC, "wb_console_print(txt, stream)",
                           "https://www.cyberbotics.com/doc/guide/using-matlab")

    # display
    generator.gen(PROC, "wb_display_attach_camera(tag, camera_tag)", "display")
    generator.gen(PROC, "wb_display_detach_camera(tag)", "display")
    generator.gen(PROC, "wb_display_draw_line(tag, x1, y1, x2, y2)", "display")
    generator.gen(PROC, "wb_display_draw_oval(tag, cx, cy, a, b)", "display")
    generator.gen(PROC, "wb_display_draw_pixel(tag, x, y)", "display")
    # generator.gen(PROC, "wb_display_draw_polygon(tag, x, y)", "display")
    generator.gen(PROC, "wb_display_draw_rectangle(tag, x, y, width, height)", "display")
    generator.gen(PROC, "wb_display_draw_text(tag, txt, x, y)", "display")
    generator.gen(PROC, "wb_display_fill_oval(tag, cx, cy, a, b)", "display")
    # generator.gen(PROC, "wb_display_fill_polygon(tag, x, y)", "display")
    generator.gen(PROC, "wb_display_fill_rectangle(tag, x, y, width, height)", "display")
    generator.gen(FUNC, "wb_display_get_height(tag)", "display")
    generator.gen(FUNC, "wb_display_get_width(tag)", "display")
    generator.gen(FUNC, "wb_display_image_copy(tag, x, y, width, height)", "display")
    generator.gen(PROC, "wb_display_image_delete(tag, imageref)", "display")
    generator.gen(FUNC, "wb_display_image_load(tag, filename)", "display")
    # generator.gen(PROC, "wb_display_image_new(tag, width, height, rgb, format)", "display")
    generator.gen(PROC, "wb_display_image_paste(tag, imageref, x, y, blend)", "display")
    generator.gen(PROC, "wb_display_image_save(tag, imageref, filename)", "display")
    generator.gen(PROC, "wb_display_set_alpha(tag, alpha)", "display")
    # generator.gen(PROC, "wb_display_set_color(tag, color)", "display")
    generator.gen(PROC, "wb_display_set_font(tag, font, size, anti_aliasing)", "display")
    generator.gen(PROC, "wb_display_set_opacity(tag, opacity)", "display")

    # device.h
    generator.gen(FUNC, "wb_device_get_model(tag)", "device")
    generator.gen(FUNC, "wb_device_get_name(tag)", "device")
    generator.gen(FUNC, "wb_device_get_node_type(tag)", "device")

    # distance_sensor.h
    generator.gen(PROC, "wb_distance_sensor_disable(tag)", "distancesensor")
    generator.gen(PROC, "wb_distance_sensor_enable(tag, sampling_period)", "distancesensor")
    generator.gen(FUNC, "wb_distance_sensor_get_aperture(tag)", "distancesensor")
    # generator.gen(FUNC, "wb_distance_sensor_get_lookup_table(tag)", "distancesensor")
    # generator.gen(FUNC, "wb_distance_sensor_get_lookup_table_size(tag)", "distancesensor")
    generator.gen(FUNC, "wb_distance_sensor_get_max_value(tag)", "distancesensor")
    generator.gen(FUNC, "wb_distance_sensor_get_min_value(tag)", "distancesensor")
    generator.gen(FUNC, "wb_distance_sensor_get_sampling_period(tag)", "distancesensor")
    generator.gen(FUNC, "wb_distance_sensor_get_type(tag)", "distancesensor")
    generator.gen(FUNC, "wb_distance_sensor_get_value(tag)", "distancesensor")

    # emitter.h
    generator.gen(FUNC, "wb_emitter_get_buffer_size(tag)", "emitter")
    generator.gen(FUNC, "wb_emitter_get_channel(tag)", "emitter")
    generator.gen(FUNC, "wb_emitter_get_range(tag)", "emitter")
    # generator.gen(FUNC, "wb_emitter_send(tag, data, size)", "emitter")
    generator.gen(PROC, "wb_emitter_set_channel(tag, channel)", "emitter")
    generator.gen(PROC, "wb_emitter_set_range(tag, range)", "emitter")

    # gps.h
    generator.gen(FUNC, "wb_gps_convert_to_degrees_minutes_seconds(decimal_degrees)", "gps")
    generator.gen(PROC, "wb_gps_disable(tag)", "gps")
    generator.gen(PROC, "wb_gps_enable(tag, sampling_period)", "gps")
    generator.gen(FUNC, "wb_gps_get_coordinate_system(tag)", "gps")
    generator.gen(FUNC, "wb_gps_get_sampling_period(tag)", "gps")
    generator.gen(FUNC, "wb_gps_get_speed(tag)", "gps")
    # generator.gen(FUNC, "wb_gps_get_speed_vector(tag)", "gps")
    # generator.gen(FUNC, "wb_gps_get_values(tag)", "gps")

    # gyro.h
    generator.gen(PROC, "wb_gyro_disable(tag)", "gyro")
    generator.gen(PROC, "wb_gyro_enable(tag, sampling_period)", "gyro")
    # generator.gen(FUNC, "wb_gyro_get_lookup_table(tag)", "gyro")
    # generator.gen(FUNC, "wb_gyro_get_lookup_table_size(tag)", "gyro")
    generator.gen(FUNC, "wb_gyro_get_sampling_period(tag)", "gyro")
    # generator.gen(FUNC, "wb_gyro_get_values(tag)", "gyro")

    # inertial_unit.h
    generator.gen(PROC, "wb_inertial_unit_disable(tag)", "inertialunit")
    generator.gen(PROC, "wb_inertial_unit_enable(tag, sampling_period)", "inertialunit")
    generator.gen(FUNC, "wb_inertial_unit_get_noise(tag)", "inertialunit")
    # generator.gen(FUNC, "wb_inertial_unit_get_quaternion(tag)", "inertialunit")
    # generator.gen(FUNC, "wb_inertial_unit_get_roll_pitch_yaw(tag)", "inertialunit")
    generator.gen(FUNC, "wb_inertial_unit_get_sampling_period(tag)", "inertialunit")

    # joystick.h
    generator.gen(PROC, "wb_joystick_disable()", "joystick")
    generator.gen(PROC, "wb_joystick_enable(sampling_period)", "joystick")
    generator.gen(FUNC, "wb_joystick_get_axis_value(axis)", "joystick")
    generator.gen(FUNC, "wb_joystick_get_model()", "joystick")
    generator.gen(FUNC, "wb_joystick_get_number_of_axes()", "joystick")
    generator.gen(FUNC, "wb_joystick_get_number_of_povs()", "joystick")
    generator.gen(FUNC, "wb_joystick_get_pov_value(pov)", "joystick")
    generator.gen(FUNC, "wb_joystick_get_pressed_button()", "joystick")
    generator.gen(FUNC, "wb_joystick_get_sampling_period()", "joystick")
    generator.gen(FUNC, "wb_joystick_is_connected()", "joystick")
    generator.gen(PROC, "wb_joystick_set_auto_centering_gain(gain)", "joystick")
    generator.gen(PROC, "wb_joystick_set_constant_force(level)", "joystick")
    generator.gen(PROC, "wb_joystick_set_constant_force_duration(duration)", "joystick")
    generator.gen(PROC, "wb_joystick_set_force_axis(axis)", "joystick")
    generator.gen(PROC, "wb_joystick_set_resistance_gain(gain)", "joystick")

    # keyboard.h
    generator.gen(PROC, "wb_keyboard_disable()", "keyboard")
    generator.gen(PROC, "wb_keyboard_enable(sampling_period)", "keyboard")
    generator.gen(FUNC, "wb_keyboard_get_key()", "keyboard")
    generator.gen(FUNC, "wb_keyboard_get_sampling_period()", "keyboard")

    # led.h
    generator.gen(FUNC, "wb_led_get(tag)", "led")
    generator.gen(PROC, "wb_led_set(tag, value)", "led")

    # lidar.h
    generator.gen(PROC, "wb_lidar_disable(tag)", "lidar")
    generator.gen(PROC, "wb_lidar_disable_point_cloud(tag)", "lidar")
    generator.gen(PROC, "wb_lidar_enable(tag, sampling_period)", "lidar")
    generator.gen(PROC, "wb_lidar_enable_point_cloud(tag)", "lidar")
    generator.gen(FUNC, "wb_lidar_get_fov(tag)", "lidar")
    generator.gen(FUNC, "wb_lidar_get_frequency(tag)", "lidar")
    generator.gen(FUNC, "wb_lidar_get_horizontal_resolution(tag)", "lidar")
    # generator.gen(FUNC, "wb_lidar_get_layer_point_cloud(tag,layer)", "lidar")
    # generator.gen(FUNC, "wb_lidar_get_layer_range_image(tag,layer)", "lidar")
    generator.gen(FUNC, "wb_lidar_get_max_frequency(tag)", "lidar")
    generator.gen(FUNC, "wb_lidar_get_max_range(tag)", "lidar")
    generator.gen(FUNC, "wb_lidar_get_min_frequency(tag)", "lidar")
    generator.gen(FUNC, "wb_lidar_get_min_range(tag)", "lidar")
    generator.gen(FUNC, "wb_lidar_get_number_of_layers(tag)", "lidar")
    generator.gen(FUNC, "wb_lidar_get_number_of_points(tag)", "lidar")
    # generator.gen(FUNC, "wb_lidar_get_point_cloud(tag)", "lidar")
    # generator.gen(FUNC, "wb_lidar_get_range_image(tag)", "lidar")
    generator.gen(FUNC, "wb_lidar_get_sampling_period(tag)", "lidar")
    generator.gen(FUNC, "wb_lidar_get_vertical_fov(tag)", "lidar")
    generator.gen(FUNC, "wb_lidar_is_point_cloud_enabled(tag)", "lidar")
    generator.gen(PROC, "wb_lidar_set_frequency(tag,frequency)", "lidar")

    # light_sensor.h
    generator.gen(PROC, "wb_light_sensor_disable(tag)", "lightsensor")
    generator.gen(PROC, "wb_light_sensor_enable(tag, sampling_period)", "lightsensor")
    # generator.gen(FUNC, "wb_light_sensor_get_lookup_table(tag)", "lightsensor")
    # generator.gen(FUNC, "wb_light_sensor_get_lookup_table_size(tag)", "lightsensor")
    generator.gen(FUNC, "wb_light_sensor_get_sampling_period(tag)", "lightsensor")
    generator.gen(FUNC, "wb_light_sensor_get_value(tag)", "lightsensor")

    # motor.h
    generator.gen(PROC, "wb_motor_disable_force_feedback(tag)", "motor")
    generator.gen(PROC, "wb_motor_disable_torque_feedback(tag)", "motor")
    generator.gen(PROC, "wb_motor_enable_force_feedback(tag, sampling_period)", "motor")
    generator.gen(PROC, "wb_motor_enable_torque_feedback(tag, sampling_period)", "motor")
    generator.gen(FUNC, "wb_motor_get_acceleration(tag)", "motor")
    generator.gen(FUNC, "wb_motor_get_available_force(tag)", "motor")
    generator.gen(FUNC, "wb_motor_get_available_torque(tag)", "motor")
    generator.gen(FUNC, "wb_motor_get_brake(tag)", "motor")
    generator.gen(FUNC, "wb_motor_get_force_feedback(tag)", "motor")
    generator.gen(FUNC, "wb_motor_get_force_feedback_sampling_period(tag)", "motor")
    generator.gen(FUNC, "wb_motor_get_max_force(tag)", "motor")
    generator.gen(FUNC, "wb_motor_get_max_position(tag)", "motor")
    generator.gen(FUNC, "wb_motor_get_max_torque(tag)", "motor")
    generator.gen(FUNC, "wb_motor_get_max_velocity(tag)", "motor")
    generator.gen(FUNC, "wb_motor_get_min_position(tag)", "motor")
    generator.gen(FUNC, "wb_motor_get_multiplier(tag)", "motor")
    generator.gen(FUNC, "wb_motor_get_position_sensor(tag)", "motor")
    generator.gen(FUNC, "wb_motor_get_target_position(tag)", "motor")
    generator.gen(FUNC, "wb_motor_get_torque_feedback(tag)", "motor")
    generator.gen(FUNC, "wb_motor_get_torque_feedback_sampling_period(tag)", "motor")
    generator.gen(FUNC, "wb_motor_get_type(tag)", "motor")
    generator.gen(FUNC, "wb_motor_get_velocity(tag)", "motor")
    generator.gen(PROC, "wb_motor_set_acceleration(tag, acceleration)", "motor")
    generator.gen(PROC, "wb_motor_set_available_force(tag, force)", "motor")
    generator.gen(PROC, "wb_motor_set_available_torque(tag, torque)", "motor")
    generator.gen(PROC, "wb_motor_set_control_pid(tag, p, i, d)", "motor")
    generator.gen(PROC, "wb_motor_set_force(tag, force)", "motor")
    generator.gen(PROC, "wb_motor_set_position(tag, position)", "motor")
    generator.gen(PROC, "wb_motor_set_torque(tag, torque)", "motor")
    generator.gen(PROC, "wb_motor_set_velocity(tag, velocity)", "motor")

    # mouse.h
    generator.gen(PROC, "wb_mouse_disable()", "mouse")
    generator.gen(PROC, "wb_mouse_disable_3d_position()", "mouse")
    generator.gen(PROC, "wb_mouse_enable(sampling_period)", "mouse")
    generator.gen(PROC, "wb_mouse_enable_3d_position()", "mouse")
    generator.gen(FUNC, "wb_mouse_get_sampling_period()", "mouse")
    # generator.gen(FUNC, "wb_mouse_get_state()", "mouse")
    generator.gen(FUNC, "wb_mouse_is_3d_position_enabled()", "mouse")

    # pen.h
    generator.gen(PROC, "wb_pen_write(tag, write)", "pen")
    # generator.gen(PROC, "wb_pen_set_ink_color(tag, color, density)", "pen")

    # position_sensor.h
    generator.gen(PROC, "wb_position_sensor_disable(tag)", "positionsensor")
    generator.gen(PROC, "wb_position_sensor_enable(tag, sampling_period)", "positionsensor")
    generator.gen(FUNC, "wb_position_sensor_get_brake(tag)", "positionsensor")
    generator.gen(FUNC, "wb_position_sensor_get_motor(tag)", "positionsensor")
    generator.gen(FUNC, "wb_position_sensor_get_sampling_period(tag)", "positionsensor")
    generator.gen(FUNC, "wb_position_sensor_get_type(tag)", "positionsensor")
    generator.gen(FUNC, "wb_position_sensor_get_value(tag)", "positionsensor")

    # radar.h
    generator.gen(PROC, "wb_radar_disable(tag)", "radar")
    generator.gen(PROC, "wb_radar_enable(tag, sampling_period)", "radar")
    generator.gen(FUNC, "wb_radar_get_horizontal_fov(tag)", "radar")
    generator.gen(FUNC, "wb_radar_get_max_range(tag)", "radar")
    generator.gen(FUNC, "wb_radar_get_min_range(tag)", "radar")
    generator.gen(FUNC, "wb_radar_get_number_of_targets(tag)", "radar")
    # generator.gen(FUNC, "wb_radar_get_targets(tag)", "radar")
    generator.gen(FUNC, "wb_radar_get_vertical_fov(tag)", "radar")
    generator.gen(FUNC, "wb_radar_get_sampling_period(tag)", "radar")

    # range_finder.h
    generator.gen(PROC, "wb_range_finder_disable(tag)", "rangefinder")
    generator.gen(PROC, "wb_range_finder_enable(tag, sampling_period)", "rangefinder")
    generator.gen(FUNC, "wb_range_finder_get_fov(tag)", "rangefinder")
    generator.gen(FUNC, "wb_range_finder_get_height(tag)", "rangefinder")
    generator.gen(FUNC, "wb_range_finder_get_max_range(tag)", "rangefinder")
    generator.gen(FUNC, "wb_range_finder_get_min_range(tag)", "rangefinder")
    # generator.gen(FUNC, "wb_range_finder_get_range_image(tag)", "rangefinder")
    generator.gen(FUNC, "wb_range_finder_get_sampling_period(tag)", "rangefinder")
    generator.gen(FUNC, "wb_range_finder_get_width(tag)", "rangefinder")
    # generator.gen(FUNC, "wb_range_finder_image_get_depth(image, width, x, y)", "rangefinder")
    generator.gen(FUNC, "wb_range_finder_save_image(tag, filename, quality)", "rangefinder")

    # receiver.h
    generator.gen(PROC, "wb_receiver_disable(tag)", "receiver")
    generator.gen(PROC, "wb_receiver_enable(tag, sampling_period)", "receiver")
    generator.gen(FUNC, "wb_receiver_get_channel(tag)", "receiver")
    # generator.gen(FUNC, "wb_receiver_get_data(tag)", "receiver")
    generator.gen(FUNC, "wb_receiver_get_data_size(tag)", "receiver")
    # generator.gen(FUNC, "wb_receiver_get_emitter_direction(tag)", "receiver")
    generator.gen(FUNC, "wb_receiver_get_queue_length(tag)", "receiver")
    generator.gen(FUNC, "wb_receiver_get_sampling_period(tag)", "receiver")
    generator.gen(FUNC, "wb_receiver_get_signal_strength(tag)", "receiver")
    generator.gen(PROC, "wb_receiver_next_packet(tag)", "receiver")
    generator.gen(PROC, "wb_receiver_set_channel(tag, channel)", "receiver")

    # robot.h
    # generator.gen(PROC, "wb_robot_init()", "robot")
    # generator.gen(PROC, "wb_robot_cleanup()", "robot")
    generator.gen(FUNC, "wb_robot_step(duration)", "robot")
    generator.gen(FUNC, "wb_robot_step_begin(duration)", "robot")
    generator.gen(FUNC, "wb_robot_step_end()", "robot")
    generator.gen(FUNC, "wb_robot_wait_for_user_input_event(event_type, timeout)", "robot")
    generator.gen(PROC, "wb_robot_battery_sensor_enable(sampling_period)", "robot")
    generator.gen(PROC, "wb_robot_battery_sensor_disable()", "robot")
    generator.gen(PROC, "wb_robot_battery_sensor_enable(sampling_period)", "robot")
    generator.gen(FUNC, "wb_robot_battery_sensor_get_sampling_period()", "robot")
    generator.gen(FUNC, "wb_robot_battery_sensor_get_value()", "robot")
    # generator.gen(PROC, "wb_robot_cleanup()", "robot")
    generator.gen(FUNC, "wb_robot_get_basic_time_step()", "robot")
    generator.gen(FUNC, "wb_robot_get_custom_data()", "robot")
    generator.gen(FUNC, "wb_robot_get_data()", "robot")
    generator.gen(FUNC, "wb_robot_get_device(name)", "robot")
    generator.gen(FUNC, "wb_robot_get_device_by_index(index)", "robot")
    generator.gen(FUNC, "wb_robot_get_mode()", "robot")
    generator.gen(FUNC, "wb_robot_get_name()", "robot")
    generator.gen(FUNC, "wb_robot_get_number_of_devices()", "robot")
    generator.gen(FUNC, "wb_robot_get_project_path()", "robot")
    generator.gen(FUNC, "wb_robot_get_supervisor()", "robot")
    generator.gen(FUNC, "wb_robot_get_synchronization()", "robot")
    generator.gen(FUNC, "wb_robot_get_time()", "robot")
    generator.gen(FUNC, "wb_robot_get_urdf(prefix)", "robot")
    generator.gen(FUNC, "wb_robot_get_world_path()", "robot")
    # generator.gen(PROC, "wb_robot_init()", "robot")
    generator.gen(PROC, "wb_robot_set_custom_data(data)", "robot")
    generator.gen(PROC, "wb_robot_set_data(data)", "robot")
    generator.gen(PROC, "wb_robot_set_mode(mode, arg)", "robot")
    generator.gen(FUNC, "wb_robot_step(duration)", "robot")
    generator.gen(FUNC, "wb_robot_wait_for_user_input_event(event_type, timeout)", "robot")
    generator.gen(FUNC, "wb_robot_wwi_receive_text()", "robot")
    generator.gen(PROC, "wb_robot_wwi_send_text(text)", "robot")

    # skin.h
    generator.gen(FUNC, "wb_skin_get_bone_count(tag)")
    generator.gen(FUNC, "wb_skin_get_bone_name(tag, index)")
    # generator.gen(FUNC, "wb_skin_get_bone_orientation(tag, index, absolute)")
    # generator.gen(FUNC, "wb_skin_get_bone_position(tag, index, absolute)")
    # generator.gen(PROC, "wb_skin_set_bone_orientation(tag, index, values, absolute)")
    # generator.gen(PROC, "wb_skin_set_bone_position(tag, index, values, absolute)")

    # speaker.h
    generator.gen(FUNC, "wb_speaker_get_engine(tag)", "speaker")
    generator.gen(FUNC, "wb_speaker_get_language(tag)", "speaker")
    generator.gen(FUNC, "wb_speaker_is_sound_playing(tag, sound)", "speaker")
    generator.gen(FUNC, "wb_speaker_is_speaking(tag)", "speaker")
    generator.gen(PROC, "wb_speaker_play_sound(left, right, sound, volume, pitch, balance, loop)", "speaker")
    generator.gen(FUNC, "wb_speaker_set_engine(tag, language)", "speaker")
    generator.gen(FUNC, "wb_speaker_set_language(tag, language)", "speaker")
    generator.gen(PROC, "wb_speaker_speak(tag, text, volume)", "speaker")
    generator.gen(PROC, "wb_speaker_stop(tag, sound)", "speaker")

    # supervisor.h
    generator.gen(FUNC, "wb_supervisor_animation_start_recording(filename)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_animation_stop_recording()", "supervisor")
    generator.gen(PROC, "wb_supervisor_export_image(filename, quality)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_field_disable_sf_tracking(field)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_field_enable_sf_tracking(field, sampling_period)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_field_get_actual_field(fieldref)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_field_get_count(fieldref)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_field_get_mf_bool(fieldref, index)", "supervisor")
    # generator.gen(FUNC, "wb_supervisor_field_get_mf_color(fieldref, index)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_field_get_mf_float(fieldref, index)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_field_get_mf_int32(fieldref, index)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_field_get_mf_node(fieldref, index)", "supervisor")
    # generator.gen(FUNC, "wb_supervisor_field_get_mf_rotation(fieldref, index)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_field_get_mf_string(fieldref, index)", "supervisor")
    # generator.gen(FUNC, "wb_supervisor_field_get_mf_vec2f(fieldref, index)", "supervisor")
    # generator.gen(FUNC, "wb_supervisor_field_get_mf_vec3f(fieldref, index)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_field_get_name(fieldref)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_field_get_sf_bool(fieldref)", "supervisor")
    # generator.gen(FUNC, "wb_supervisor_field_get_sf_color(fieldref)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_field_get_sf_float(fieldref)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_field_get_sf_int32(fieldref)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_field_get_sf_node(fieldref)", "supervisor")
    # generator.gen(FUNC, "wb_supervisor_field_get_sf_rotation(fieldref)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_field_get_sf_string(fieldref)", "supervisor")
    # generator.gen(FUNC, "wb_supervisor_field_get_sf_vec2f(fieldref)", "supervisor")
    # generator.gen(FUNC, "wb_supervisor_field_get_sf_vec3f(fieldref)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_field_get_type(fieldref)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_field_get_type_name(fieldref)", "supervisor")
    generator.gen(PROC, "wb_supervisor_field_import_mf_node_from_string(fieldref, position, node_string)", "supervisor")
    generator.gen(PROC, "wb_supervisor_field_import_sf_node_from_string(fieldref, node_string)", "supervisor")
    generator.gen(PROC, "wb_supervisor_field_insert_mf_bool(fieldref, index, value)", "supervisor")
    # generator.gen(PROC, "wb_supervisor_field_insert_mf_color(fieldref, index, values)", "supervisor")
    generator.gen(PROC, "wb_supervisor_field_insert_mf_float(fieldref, index, value)", "supervisor")
    generator.gen(PROC, "wb_supervisor_field_insert_mf_int32(fieldref, index, value)", "supervisor")
    # generator.gen(PROC, "wb_supervisor_field_insert_mf_rotation(fieldref, index, values)", "supervisor")
    generator.gen(PROC, "wb_supervisor_field_insert_mf_string(fieldref, index, string)", "supervisor")
    # generator.gen(PROC, "wb_supervisor_field_insert_mf_vec2f(fieldref, index, values)", "supervisor")
    # generator.gen(PROC, "wb_supervisor_field_insert_mf_vec3f(fieldref, index, values)", "supervisor")
    generator.gen(PROC, "wb_supervisor_field_remove_mf(fieldref, index)", "supervisor")
    generator.gen(PROC, "wb_supervisor_field_remove_mf_node(fieldref, position)", "supervisor")
    generator.gen(PROC, "wb_supervisor_field_remove_sf(fieldref)", "supervisor")
    generator.gen(PROC, "wb_supervisor_field_set_mf_bool(fieldref, index, value)", "supervisor")
    # generator.gen(PROC, "wb_supervisor_field_set_mf_color(fieldref, index, values)", "supervisor")
    generator.gen(PROC, "wb_supervisor_field_set_mf_float(fieldref, index, value)", "supervisor")
    generator.gen(PROC, "wb_supervisor_field_set_mf_int32(fieldref, index, value)", "supervisor")
    # generator.gen(PROC, "wb_supervisor_field_set_mf_rotation(fieldref, index, values)", "supervisor")
    generator.gen(PROC, "wb_supervisor_field_set_mf_string(fieldref, index, string)", "supervisor")
    # generator.gen(PROC, "wb_supervisor_field_set_mf_vec2f(fieldref, index, values)", "supervisor")
    # generator.gen(PROC, "wb_supervisor_field_set_mf_vec3f(fieldref, index, values)", "supervisor")
    generator.gen(PROC, "wb_supervisor_field_set_sf_bool(fieldref, value)", "supervisor")
    # generator.gen(PROC, "wb_supervisor_field_set_sf_color(fieldref, values)", "supervisor")
    generator.gen(PROC, "wb_supervisor_field_set_sf_float(fieldref, value)", "supervisor")
    generator.gen(PROC, "wb_supervisor_field_set_sf_int32(fieldref, value)", "supervisor")
    # generator.gen(PROC, "wb_supervisor_field_set_sf_rotation(fieldref, values)", "supervisor")
    generator.gen(PROC, "wb_supervisor_field_set_sf_string(fieldref, string)", "supervisor")
    # generator.gen(PROC, "wb_supervisor_field_set_sf_vec2f(fieldref, values)", "supervisor")
    # generator.gen(PROC, "wb_supervisor_field_set_sf_vec3f(fieldref, values)", "supervisor")
    # generator.gen(FUNC, "wb_supervisor_get_movie_status()", "supervisor"); # DEPRECATED
    # generator.gen(PROC, "wb_supervisor_load_world(filename)", "supervisor"); # DEPRECATED
    generator.gen(FUNC, "wb_supervisor_movie_failed()", "supervisor")
    # generator.gen(FUNC, "wb_supervisor_movie_get_status()", "supervisor"); # DEPRECATED
    generator.gen(FUNC, "wb_supervisor_movie_is_ready()", "supervisor")
    generator.gen(PROC, "wb_supervisor_movie_start_recording(filename, width, height, codec, quality, acceleration, caption)",
                        "supervisor")
    generator.gen(PROC, "wb_supervisor_movie_stop_recording()", "supervisor")
    generator.gen(PROC, "wb_supervisor_node_add_force(noderef, force, relative)", "supervisor")
    generator.gen(PROC, "wb_supervisor_node_add_force_with_offset(noderef, force, offset, relative)", "supervisor")
    generator.gen(PROC, "wb_supervisor_node_add_torque(noderef, torque, relative)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_disable_contact_points_tracking(noderef)", "supervisor")
    # DEPRECATED
    generator.gen(FUNC, "wb_supervisor_node_disable_contact_point_tracking(noderef, include_descendants)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_disable_pose_tracking(node, from_node)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_enable_contact_points_tracking(noderef, sampling_period, include_descendants)",
                        "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_enable_contact_point_tracking(noderef, sampling_period, include_descendants)",
                        "supervisor")  # DEPRECATED
    generator.gen(FUNC, "wb_supervisor_node_enable_pose_tracking(sampling_period, node, from_node)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_export_string(noderef)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_get_base_node_field(noderef, fieldname)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_get_base_node_field_by_index(noderef, fieldindex)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_get_base_type_name(noderef)", "supervisor")
    # generator.gen(FUNC, "wb_supervisor_node_get_center_of_mass(noderef)", "supervisor")
    # generator.gen(FUNC, "wb_supervisor_node_get_contact_point(noderef, index)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_get_contact_point_node(noderef, index)", "supervisor")
    # generator.gen(PROC, "wb_supervisor_node_get_contact_points(noderef, include_descendants)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_get_def(noderef)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_get_field(noderef, fieldname)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_get_field_by_index(noderef, fieldindex)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_get_from_def(defname)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_get_from_device(tag)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_get_from_id(id)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_get_from_proto_def(noderef, defname)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_get_id(noderef)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_get_number_of_base_node_fields(noderef)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_get_number_of_contact_points(noderef, include_descendants)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_get_number_of_fields(noderef)", "supervisor")
    # generator.gen(FUNC, "wb_supervisor_node_get_orientation(noderef)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_get_parent_node(noderef)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_get_proto(noderef)", "supervisor")
    # generator.gen(FUNC, "wb_supervisor_node_get_pose(noderef, noderef_from)", "supervisor")
    # generator.gen(FUNC, "wb_supervisor_node_get_position(noderef)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_get_root()", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_get_selected()", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_get_self()", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_get_static_balance(noderef)", "supervisor")
    # generator.gen(FUNC, "wb_supervisor_node_get_velocity(noderef)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_get_type(noderef)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_get_type_name(noderef)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_is_proto(noderef)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_load_state(noderef, state_name)", "supervisor")
    generator.gen(PROC, "wb_supervisor_node_move_viewpoint(node)", "supervisor")
    generator.gen(PROC, "wb_supervisor_node_remove(noderef)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_reset_physics(noderef)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_restart_controller(noderef)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_save_state(noderef, state_name)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_node_set_joint_position(noderef, position, index)", "supervisor")
    generator.gen(PROC, "wb_supervisor_node_set_velocity(noderef, velocity)", "supervisor")
    generator.gen(PROC, "wb_supervisor_node_set_visibility(node, from, visible)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_proto_get_field(protoref, fieldname)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_proto_get_field_by_index(protoref, fieldindex)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_proto_get_number_of_fields(protoref)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_proto_get_parent(protoref)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_proto_get_type_name(protoref)", "supervisor")
    generator.gen(FUNC, "wb_supervisor_proto_is_derived(protoref)", "supervisor")
    # generator.gen(FUNC, "wb_supervisor_save_world(filename)", "supervisor"); # DEPRECATED
    # generator.gen(PROC, "wb_supervisor_set_label()", "supervisor")
    generator.gen(FUNC, "wb_supervisor_simulation_get_mode()", "supervisor")
    # generator.gen(PROC, "wb_supervisor_simulation_physics_reset()", "supervisor"); # DEPRECATED
    generator.gen(PROC, "wb_supervisor_simulation_quit(status)", "supervisor")
    generator.gen(PROC, "wb_supervisor_simulation_reset()", "supervisor")
    generator.gen(PROC, "wb_supervisor_simulation_reset_physics()", "supervisor")
    # generator.gen(PROC, "wb_supervisor_simulation_revert()", "supervisor"); # DEPRECATED
    generator.gen(PROC, "wb_supervisor_simulation_set_mode(mode)", "supervisor")
    # DEPRECATED
    # generator.gen(PROC, "wb_supervisor_start_movie(filename, width, height, codec, quality, acceleration, caption)");
    # generator.gen(PROC, "wb_supervisor_stop_movie()", "supervisor"); # DEPRECATED
    # generator.gen(FUNC, "wb_supervisor_virtual_reality_headset_get_orientation()", "supervisor")
    # generator.gen(FUNC, "wb_supervisor_virtual_reality_headset_get_position()", "supervisor")
    generator.gen(FUNC, "wb_supervisor_virtual_reality_headset_is_used()", "supervisor")
    generator.gen(PROC, "wb_supervisor_world_load(filename)", "supervisor")
    generator.gen(PROC, "wb_supervisor_world_reload()", "supervisor")
    # implemented due to the default arguments management
    # generator.gen(FUNC, "wb_supervisor_world_save(filename)", "supervisor");

    # touch_sensor.h
    generator.gen(PROC, "wb_touch_sensor_disable(tag)", "touchsensor")
    generator.gen(PROC, "wb_touch_sensor_enable(tag, sampling_period)", "touchsensor")
    # generator.gen(FUNC, "wb_touch_sensor_get_lookup_table(tag)", "touchsensor")
    # generator.gen(FUNC, "wb_touch_sensor_get_lookup_table_size(tag)", "touchsensor")
    generator.gen(FUNC, "wb_touch_sensor_get_sampling_period(tag)", "touchsensor")
    generator.gen(FUNC, "wb_touch_sensor_get_type(tag)", "touchsensor")
    generator.gen(FUNC, "wb_touch_sensor_get_value(tag)", "touchsensor")
    # generator.gen(FUNC, "wb_touch_sensor_get_values(tag)", "touchsensor")

    # vacuum_gripper.h
    generator.gen(PROC, "wb_vacuum_gripper_disable_presence(tag)", "vacuum_gripper")
    generator.gen(PROC, "wb_vacuum_gripper_enable_presence(tag, sampling_period)", "vacuum_gripper")
    generator.gen(FUNC, "wb_vacuum_gripper_get_presence(tag)", "vacuum_gripper")
    generator.gen(FUNC, "wb_vacuum_gripper_get_presence_sampling_period(tag)", "vacuum_gripper")
    generator.gen(FUNC, "wb_vacuum_gripper_is_on(tag)", "vacuum_gripper")
    generator.gen(PROC, "wb_vacuum_gripper_turn_on(tag)", "vacuum_gripper")
    generator.gen(PROC, "wb_vacuum_gripper_turn_off(tag)", "vacuum_gripper")

    # utils/motion.h
    generator.gen(PROC, "wbu_motion_delete(motionref)", "motion")
    generator.gen(FUNC, "wbu_motion_get_duration(motionref)", "motion")
    generator.gen(FUNC, "wbu_motion_get_time(motionref)", "motion")
    generator.gen(FUNC, "wbu_motion_is_over(motionref)", "motion")
    generator.gen(FUNC, "wbu_motion_new(filename)", "motion")
    generator.gen(PROC, "wbu_motion_play(motionref)", "motion")
    generator.gen(PROC, "wbu_motion_set_loop(motionref, loop)", "motion")
    generator.gen(PROC, "wbu_motion_set_reverse(motionref, reverse)", "motion")
    generator.gen(FUNC, "wbu_motion_set_time(motionref, time)", "motion")
    generator.gen(PROC, "wbu_motion_stop(motionref)", "motion")

    # utils/system.h
    generator.gen(FUNC, "wbu_system_getenv(variable)")
    generator.gen(FUNC, "wbu_system_short_path(path)")

    # constants
    generator.gen_const("WB_STDOUT", "1")
    generator.gen_const("WB_STDERR", "2")

    generator.gen_const("WB_CHANNEL_BROADCAST", "-1")

    generator.gen_const("WB_IMAGE_RGB",  "3")
    generator.gen_const("WB_IMAGE_RGBA", "4")
    generator.gen_const("WB_IMAGE_ARGB", "5")
    generator.gen_const("WB_IMAGE_BGRA", "6")
    generator.gen_const("WB_IMAGE_ABGR", "7")

    generator.gen_const("WB_KEYBOARD_KEY",        "65535")
    generator.gen_const("WB_KEYBOARD_SHIFT",      "65536")
    generator.gen_const("WB_KEYBOARD_CONTROL",   "131072")
    generator.gen_const("WB_KEYBOARD_ALT",       "262144")
    generator.gen_const("WB_KEYBOARD_LEFT",         "314")
    generator.gen_const("WB_KEYBOARD_UP",           "315")
    generator.gen_const("WB_KEYBOARD_RIGHT",        "316")
    generator.gen_const("WB_KEYBOARD_DOWN",         "317")
    generator.gen_const("WB_KEYBOARD_PAGEUP",       "366")
    generator.gen_const("WB_KEYBOARD_PAGEDOWN",     "367")
    generator.gen_const("WB_KEYBOARD_HOME",         "313")
    generator.gen_const("WB_KEYBOARD_END",          "312")
    generator.gen_const("WB_KEYBOARD_NUMPAD_UP",    "377")
    generator.gen_const("WB_KEYBOARD_NUMPAD_DOWN",  "379")
    generator.gen_const("WB_KEYBOARD_NUMPAD_LEFT",  "376")
    generator.gen_const("WB_KEYBOARD_NUMPAD_RIGHT", "378")
    generator.gen_const("WB_KEYBOARD_NUMPAD_HOME",  "375")
    generator.gen_const("WB_KEYBOARD_NUMPAD_END",   "382")

    generator.gen_const("WB_NO_FIELD",     "0")
    generator.gen_const("WB_SF_BOOL",      "1")
    generator.gen_const("WB_SF_INT32",     "2")
    generator.gen_const("WB_SF_FLOAT",     "3")
    generator.gen_const("WB_SF_VEC2F",     "4")
    generator.gen_const("WB_SF_VEC3F",     "5")
    generator.gen_const("WB_SF_ROTATION",  "6")
    generator.gen_const("WB_SF_COLOR",     "7")
    generator.gen_const("WB_SF_STRING",    "8")
    generator.gen_const("WB_SF_NODE",      "9")
    generator.gen_const("WB_MF",          "16")
    generator.gen_const("WB_MF_BOOL",     "17")
    generator.gen_const("WB_MF_INT32",    "18")
    generator.gen_const("WB_MF_FLOAT",    "19")
    generator.gen_const("WB_MF_VEC2F",    "20")
    generator.gen_const("WB_MF_VEC3F",    "21")
    generator.gen_const("WB_MF_ROTATION", "22")
    generator.gen_const("WB_MF_COLOR",    "23")
    generator.gen_const("WB_MF_STRING",   "24")
    generator.gen_const("WB_MF_NODE",     "25")

    generator.gen_const("WB_EVENT_QUIT", "-1")
    generator.gen_const("WB_EVENT_NO_EVENT", "0")
    generator.gen_const("WB_EVENT_MOUSE_CLICK", "1")
    generator.gen_const("WB_EVENT_MOUSE_MOVE", "2")
    generator.gen_const("WB_EVENT_KEYBOARD", "4")
    generator.gen_const("WB_EVENT_JOYSTICK_BUTTON", "8")
    generator.gen_const("WB_EVENT_JOYSTICK_AXIS", "16")
    generator.gen_const("WB_EVENT_JOYSTICK_POV", "32")

    # ANSI codes
    generator.gen_const("ANSI_RESET", "strcat(27, '[0m')")
    generator.gen_const("ANSI_BOLD", "strcat(27, '[1m')")
    generator.gen_const("ANSI_UNDERLINE", "strcat(27, '[4m')")
    generator.gen_const("ANSI_BLACK_FOREGROUND", "strcat(27, '[30m')")
    generator.gen_const("ANSI_RED_FOREGROUND", "strcat(27, '[31m')")
    generator.gen_const("ANSI_GREEN_FOREGROUND", "strcat(27, '[32m')")
    generator.gen_const("ANSI_YELLOW_FOREGROUND", "strcat(27, '[33m')")
    generator.gen_const("ANSI_BLUE_FOREGROUND", "strcat(27, '[34m')")
    generator.gen_const("ANSI_MAGENTA_FOREGROUND", "strcat(27, '[35m')")
    generator.gen_const("ANSI_CYAN_FOREGROUND", "strcat(27, '[36m')")
    generator.gen_const("ANSI_WHITE_FOREGROUND", "strcat(27, '[37m')")
    generator.gen_const("ANSI_BLACK_BACKGROUND", "strcat(27, '[40m')")
    generator.gen_const("ANSI_RED_BACKGROUND", "strcat(27, '[41m')")
    generator.gen_const("ANSI_GREEN_BACKGROUND", "strcat(27, '[42m')")
    generator.gen_const("ANSI_YELLOW_BACKGROUND", "strcat(27, '[43m')")
    generator.gen_const("ANSI_BLUE_BACKGROUND", "strcat(27, '[44m')")
    generator.gen_const("ANSI_MAGENTA_BACKGROUND", "strcat(27, '[45m')")
    generator.gen_const("ANSI_CYAN_BACKGROUND", "strcat(27, '[46m')")
    generator.gen_const("ANSI_WHITE_BACKGROUND", "strcat(27, '[47m')")
    generator.gen_const("ANSI_CLEAR_SCREEN", "strcat(27, '[2J')")

    # these lists are exact copy paste from the doc (without the commented lines and with addition of the experimental nodes)
    generator.gen_consts_from_list(
        'WB_DISTANCE_SENSOR_GENERIC, WB_DISTANCE_SENSOR_INFRA_RED, WB_DISTANCE_SENSOR_SONAR, WB_DISTANCE_SENSOR_LASER')
    generator.gen_consts_from_list('WB_GPS_LOCAL_COORDINATE, WB_GPS_WGS84_COORDINATE')
    generator.gen_consts_from_list(
        'WB_MODE_SIMULATION, WB_MODE_CROSS_COMPILATION, WB_MODE_REMOTE_CONTROL')
    generator.gen_consts_from_list("""
        WB_NODE_NO_NODE,
        WB_NODE_APPEARANCE, WB_NODE_BACKGROUND, WB_NODE_BILLBOARD, WB_NODE_BOX, WB_NODE_CAD_SHAPE, WB_NODE_CAPSULE,
        WB_NODE_COLOR, WB_NODE_CONE, WB_NODE_COORDINATE,
        WB_NODE_CYLINDER, WB_NODE_DIRECTIONAL_LIGHT, WB_NODE_ELEVATION_GRID,
        WB_NODE_FOG, WB_NODE_GROUP, WB_NODE_IMAGE_TEXTURE, WB_NODE_INDEXED_FACE_SET,
        WB_NODE_INDEXED_LINE_SET, WB_NODE_MATERIAL, WB_NODE_MESH, WB_NODE_MUSCLE, WB_NODE_NORMAL,
        WB_NODE_PBR_APPEARANCE, WB_NODE_PLANE, WB_NODE_POINT_LIGHT, WB_NODE_POINT_SET, WB_NODE_POSE, WB_NODE_SHAPE,
        WB_NODE_SPHERE, WB_NODE_SPOT_LIGHT, WB_NODE_TEXTURE_COORDINATE,
        WB_NODE_TEXTURE_TRANSFORM, WB_NODE_TRANSFORM, WB_NODE_VIEWPOINT,
        WB_NODE_ROBOT,
        WB_NODE_ACCELEROMETER, WB_NODE_ALTIMETER, WB_NODE_BRAKE, WB_NODE_CAMERA, WB_NODE_COMPASS,
        WB_NODE_CONNECTOR, WB_NODE_DISPLAY, WB_NODE_DISTANCE_SENSOR, WB_NODE_EMITTER,
        WB_NODE_GPS, WB_NODE_GYRO, WB_NODE_INERTIAL_UNIT, WB_NODE_LED, WB_NODE_LIDAR,
        WB_NODE_LIGHT_SENSOR, WB_NODE_LINEAR_MOTOR, WB_NODE_PEN,
        WB_NODE_POSITION_SENSOR, WB_NODE_PROPELLER, WB_NODE_RADAR,
        WB_NODE_RANGE_FINDER, WB_NODE_RECEIVER, WB_NODE_ROTATIONAL_MOTOR,
        WB_NODE_SKIN, WB_NODE_SPEAKER, WB_NODE_TOUCH_SENSOR, WB_NODE_VACUUM_GRIPPER,
        WB_NODE_BALL_JOINT, WB_NODE_BALL_JOINT_PARAMETERS, WB_NODE_CHARGER,
        WB_NODE_CONTACT_PROPERTIES, WB_NODE_DAMPING, WB_NODE_FLUID,
        WB_NODE_FOCUS, WB_NODE_HINGE_JOINT, WB_NODE_HINGE_JOINT_PARAMETERS,
        WB_NODE_HINGE_2_JOINT, WB_NODE_IMMERSION_PROPERTIES, WB_NODE_JOINT_PARAMETERS,
        WB_NODE_LENS, WB_NODE_LENS_FLARE, WB_NODE_PHYSICS, WB_NODE_RECOGNITION,
        WB_NODE_SLIDER_JOINT, WB_NODE_SLOT, WB_NODE_SOLID, WB_NODE_SOLID_REFERENCE,
        WB_NODE_TRACK, WB_NODE_TRACK_WHEEL, WB_NODE_WORLD_INFO, WB_NODE_ZOOM,
        WB_NODE_MICROPHONE, WB_NODE_RADIO
        """)
    generator.gen_consts_from_list(
        'WB_SUPERVISOR_SIMULATION_MODE_PAUSE, WB_SUPERVISOR_SIMULATION_MODE_REAL_TIME, WB_SUPERVISOR_SIMULATION_MODE_FAST')
    generator.gen_consts_from_list('WB_ROTATIONAL, WB_LINEAR')
    generator.gen_consts_from_list(
        'WB_TOUCH_SENSOR_BUMPER, WB_TOUCH_SENSOR_FORCE, WB_TOUCH_SENSOR_FORCE3D')
    generator.close()


if __name__ == '__main__':
    main(sys.argv[1:])
