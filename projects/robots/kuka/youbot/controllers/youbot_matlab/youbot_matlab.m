% Description: MATLAB controller example for Webots

% uncomment the next two lines if you want to use
% MATLAB's desktop and interact with the controller
%desktop;
%keyboard;

TIME_STEP = 64; % time in [ms] of a simulation step

global MAX_SPEED;
MAX_SPEED = 0.30;
global SPEED_INCREMENT;
SPEED_INCREMENT = 0.1;
global WHEEL_RADIUS;
WHEEL_RADIUS = 0.05;
global LX;
LX = 0.158;
global LY;
LY= 0.228;

global robot_vx;
robot_vx = 0.0;
global robot_vy;
robot_vy = 0.0;
global robot_omega;
robot_omega = 0.0;

previous_key = 0;
wb_keyboard_enable(1);

% Initialization
arm_init
base_init
gripper_init
passive_wait(2.2)

% Short Demo
automatic_behavior

% Keyboard control
previous_key = 0;
wb_keyboard_enable(1);

wb_console_print(sprintf(['\nControl commands:\n', ...
  '  Arrows:       Move the robot\n', ...
  '  Page Up/Down: Rotate the robot\n', ...
  '  +/-:          (Un)grip\n', ...
  '  Shift + arrows:   Handle the arm\n', ...
  '  Space: Reset']), WB_STDOUT);

while wb_robot_step(TIME_STEP) ~= -1
  key = char(wb_keyboard_get_key());
  if ((key > 0) && key ~= previous_key)
    switch key
      case WB_KEYBOARD_UP
        base_forwards_increment
      case WB_KEYBOARD_DOWN
        base_backwards_increment
      case WB_KEYBOARD_LEFT
        base_strafe_left_increment
      case WB_KEYBOARD_RIGHT
        base_strafe_right_increment
      case WB_KEYBOARD_PAGEUP
        base_turn_left_increment
      case WB_KEYBOARD_PAGEDOWN
        base_turn_right_increment
      case {WB_KEYBOARD_END, ' '}
        base_reset
        arm_perform_reset
      case {'+', 388, 65585}
        wb_console_print('Grip', WB_STDOUT)
        gripper_grip
      case {'-', 390}
        wb_console_print('Ungrip', WB_STDOUT)
        gripper_release
      case {332, WB_KEYBOARD_UP | WB_KEYBOARD_SHIFT}
        wb_console_print('Increase arm height', WB_STDOUT)
        arm_increase_height
      case {326, WB_KEYBOARD_DOWN | WB_KEYBOARD_SHIFT}
        wb_console_print('Decrease arm height', WB_STDOUT)
        arm_decrease_height
      case {330,  WB_KEYBOARD_RIGHT | WB_KEYBOARD_SHIFT}
        wb_console_print('Increase arm orientation', WB_STDOUT)
        arm_increase_orientation
      case {328, WB_KEYBOARD_LEFT | WB_KEYBOARD_SHIFT}
        wb_console_print('Decrease arm orientation', WB_STDOUT)
        arm_decrease_orientation
      otherwise
        wb_console_print(sprintf('Wrong keyboard input (ASCII=%d)', key), WB_STDOUT)
    end
  end
  previous_key = key;
end
