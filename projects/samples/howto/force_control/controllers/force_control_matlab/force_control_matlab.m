% Example of spring and dampers simulation implemented with force control
% To try this example you need to change the Robot.controller field 
% to "force_control_matlab" in the force_control.wbt example

function force_control_matlab

CONTROL_STEP = 4;
SPRING_CONSTANT = 40;
DAMPING_CONSTANT = 0.2;

motor = wb_robot_get_device('slider');
sensor = wb_robot_get_device('position sensor');
wb_position_sensor_enable(sensor, CONTROL_STEP);  

% forever...
while true
  % move red and blue blocks appart
  wb_motor_set_position(motor, 0.3);
  for i = [1:500]  % for 2 seconds
    wb_robot_step(CONTROL_STEP);
  end

  % spring simulation
  previous_pos = 0;
  for i = [1:2500]  % for 10 seconds
    % effective position
    pos = wb_position_sensor_get_value(sensor);

    % compute velocity in [m/s]
    vel = (pos - previous_pos) / (CONTROL_STEP / 1000); 
    previous_pos = pos;

    % spring force
    sf = -SPRING_CONSTANT * pos;

    % damping force
    df = -DAMPING_CONSTANT * vel;

    % add spring and damping forces
    wb_motor_set_force(motor, sf + df);

    % run physics simulation and return
    wb_robot_step(CONTROL_STEP);
  end
end
