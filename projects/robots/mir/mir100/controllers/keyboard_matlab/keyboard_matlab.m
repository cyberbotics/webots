%
%  Copyright 1996-2022 Cyberbotics Ltd.
%
%  Licensed under the Apache License, Version 2.0 (the "License");
%  you may not use this file except in compliance with the License.
%  You may obtain a copy of the License at
%
%      http://www.apache.org/licenses/LICENSE-2.0
%
%  Unless required by applicable law or agreed to in writing, software
%  distributed under the License is distributed on an "AS IS" BASIS,
%  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
%  See the License for the specific language governing permissions and
%  limitations under the License.
%

%
%  Description:  This controller is used to move the six-wheeled (2 actuated) robot MiR100 in an industrial environment
%                using the keyboard. The keys are the following:
%
%                vx         : ↑/↓
%                ω          : ←/→
%                Reset      : Space bar
%
function keyboard_control

WHEEL_RADIUS = 0.0625;
SPEED_MAX = 0.95;
SPEED_MIN = -0.3;
ANGULAR_SPEED_MAX = 0.3;
ANGULAR_SPEED_MIN = -0.3;
SPEED_INCREMENT = 0.05;
ANGULAR_SPEED_INCREMENT = 0.05;
DISTANCE_TO_CENTER = 0.2226;

time_step = wb_robot_get_basic_time_step();

motor_left_wheel = wb_robot_get_device('middle_left_wheel_joint');
motor_right_wheel = wb_robot_get_device('middle_right_wheel_joint');

% Wheels in velocity control, so position must be set to infinity.
wb_motor_set_position(motor_left_wheel, Inf);
wb_motor_set_position(motor_right_wheel, Inf);
wb_motor_set_velocity(motor_left_wheel, 0.0);
wb_motor_set_velocity(motor_right_wheel, 0.0);

target_speed = 0.0;  % forwards speed [m].
target_omega = 0.0;  % angular speed [rad/s].

depth_camera = wb_robot_get_device('depth_camera');
wb_range_finder_enable(depth_camera, time_step);

wb_keyboard_enable(time_step);

wb_console_print(sprintf(['To move the Mir100 with your keyboard, click first inside the simulation window and press:\n', ...
    'vx   : U/D \n', ...
    'w    : L/R \n', ...
    'Reset: Space bar']), WB_STDOUT);

while (wb_robot_step(time_step) ~= -1)
    key = wb_keyboard_get_key();
    is_key_valid = 1;

    switch key
        case WB_KEYBOARD_UP
            target_speed = min(target_speed + SPEED_INCREMENT, SPEED_MAX);

        case WB_KEYBOARD_DOWN
            target_speed = max(target_speed - SPEED_INCREMENT, SPEED_MIN);

        case WB_KEYBOARD_LEFT
            target_omega = min(target_omega + ANGULAR_SPEED_INCREMENT, ANGULAR_SPEED_MAX);

        case WB_KEYBOARD_RIGHT
            target_omega = max(target_omega - ANGULAR_SPEED_INCREMENT, ANGULAR_SPEED_MIN);

        case ' '
            target_speed = 0;
            target_omega = 0;

        otherwise
            is_key_valid = 0;
    end

    if (is_key_valid)
        wb_console_print(sprintf('vx:%.2f[m/s] w:%.2f[rad/s]', target_speed, target_omega), WB_STDOUT);
        % Computes the wheel motors speeds from vx and ω.
        wb_motor_set_velocity(motor_left_wheel, (target_speed - target_omega * DISTANCE_TO_CENTER) / WHEEL_RADIUS);
        wb_motor_set_velocity(motor_right_wheel, (target_speed + target_omega * DISTANCE_TO_CENTER) / WHEEL_RADIUS);
    end
end
