% Description: MATLAB controller example for Webots
%              This example does not need the Image Processing Toolbox
function matlab

% uncomment the next two lines if you want to use
% MATLAB's desktop and interact with the controller
%desktop;
%keyboard;

% control step
TIME_STEP=64;

% number of distance sensors
N=8;

% motor speed unit
SPEED_UNIT=0.00628;

% for collision avoidance
braitenberg_matrix = [
 150 -35;
 100 -15;
  80 -10;
 -10 -10;
 -10 -10;
 -10  80;
 -30 100;
 -20 150 ];

% edge detection matrix
conv_matrix = [
 -1 -1 -1;
 -1  8 -1;
 -1 -1 -1 ];

% get and enable all distance sensors
for i=1:N
  ps(i) = wb_robot_get_device(['ps' int2str(i-1)]);
  wb_distance_sensor_enable(ps(i),TIME_STEP);
end

% get the display (not a real e-puck device !)
display_device = wb_robot_get_device('display');
wb_display_set_color(display_device, [0 0 0]);

% get and enable camera
camera = wb_robot_get_device('camera');
wb_camera_enable(camera,TIME_STEP);

% get and enable emitter and receiver
emitter = wb_robot_get_device('emitter');
receiver = wb_robot_get_device('receiver');
wb_receiver_enable(receiver,TIME_STEP);

% get the motors and set target position to infinity (speed control)
left_motor = wb_robot_get_device('left wheel motor');
right_motor = wb_robot_get_device('right wheel motor');
wb_motor_set_position(left_motor, inf);
wb_motor_set_position(right_motor, inf);
wb_motor_set_velocity(left_motor, 0);
wb_motor_set_velocity(right_motor, 0);

% avoid dummy values on first device query
step = 0;
samples = 0;

wb_console_print('Hello!', WB_STDOUT);
wb_console_print(strcat('Running', ANSI_RED_FOREGROUND, ' Matlab', ANSI_RESET, ' sample Webots controller.'), WB_STDOUT);

while wb_robot_step(TIME_STEP) ~= -1

  step = step + 1;

  % convert to null-terminated 8 bit ascii string
  message = uint8([datestr(now) 0]);
  wb_emitter_send(emitter, message);

  % read all distance sensors
  for i=1:N
    sensor_values(i) = wb_distance_sensor_get_value(ps(i));
  end

  % get camera RGB image
  % this function return an image in true color (RGB) uint8 format
  rgb = wb_camera_get_image(camera);

  % display inversed rgb image with Display device
  inverse = 255 - rgb;
  imageref = wb_display_image_new(display_device, inverse, WB_IMAGE_RGB);
  wb_display_image_paste(display_device, imageref, 0, 0, false);
  wb_display_image_delete(display_device, imageref);

  % add gunsight lines
  wb_display_draw_line(display_device, 0, 19, 51, 19);
  wb_display_draw_line(display_device, 25, 0, 25, 38);

  % create intensity image from RGB image
  intens = double((rgb(:,:,1)+rgb(:,:,2)+rgb(:,:,3))/3);

  % edge detection
  edges = conv2(intens, conv_matrix, 'valid');

  % display camera image
  subplot(2,2,1);
  image(rgb);
  title('RGB Camera');

  % display 'canny' image
  subplot(2,2,2);
  image(edges);
  colormap('gray');
  title('Edge detection');

  % get position from Supervisor
  while wb_receiver_get_queue_length(receiver) > 0
    pos = wb_receiver_get_data(receiver,'double');
    wb_receiver_next_packet(receiver);

    % store position
    samples = samples + 1;
    p(samples,:) = pos;

    % plot latest trajectory segment
    subplot(2,2,3);
    if (samples > 100)
      plot(-p(samples-100:samples,2),p(samples-100:samples,1));
    else
      plot(-p(1:samples,2),p(1:samples,1));
    end

    % plot current e-puck position
    hold on;
    plot(-p(samples,2),p(samples,1),'ro');
    axis([-1 1 -1 1]);
    title('Trajectory (Supervisor)');
    hold off;
  end

  % plot distance sensor values
  subplot(2,2,4);
  bar([1:8], sensor_values);
  title('Distance sensors');
  axis([1 8 0 2000]);

  % flush graphics
  drawnow;

  % braitenberg collision avoidance
  speed = (1 - (sensor_values / 500)) * braitenberg_matrix * 5;

  % avoid Webots warning
  speed = min(speed, 1000);

  % actuate wheels
  wb_motor_set_velocity(left_motor, SPEED_UNIT * speed(1));
  wb_motor_set_velocity(right_motor, SPEED_UNIT * speed(2));
end

% your cleanup code goes here
