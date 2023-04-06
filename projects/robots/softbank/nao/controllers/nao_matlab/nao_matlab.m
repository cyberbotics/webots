% Description: MATLAB controller example for Webots
function nao_matlab

% uncomment the next two lines if you want to use
% MATLAB's desktop and interact with the controller
%desktop;
%keyboard;

% control step
TIME_STEP = 40;

% Laplacian edge detection matrix
conv_matrix = [
 -1 -1 -1;
 -1  8 -1;
 -1 -1 -1 ];

% get and enable camera
camera = wb_robot_get_device('CameraTop');
wb_camera_enable(camera,TIME_STEP);

% get and enable accelerometer
accelerometer = wb_robot_get_device('accelerometer');
wb_accelerometer_enable(accelerometer,TIME_STEP);

% make the robot walk continuously
forwards_motion = wbu_motion_new('../../motions/Forwards.motion');
wbu_motion_set_loop(forwards_motion, true);
wbu_motion_play(forwards_motion);

% get and enable foot sensors
fsr(1) = wb_robot_get_device('LFsr');
fsr(2) = wb_robot_get_device('RFsr');
wb_touch_sensor_enable(fsr(1), TIME_STEP);
wb_touch_sensor_enable(fsr(2), TIME_STEP);

% get and enable receiver
receiver = wb_robot_get_device('receiver');
wb_receiver_enable(receiver,TIME_STEP);

% count the steps
steps = 0;

while wb_robot_step(TIME_STEP) ~= -1
  steps = steps + 1;
  
  % receive text messages from Supervisor
  while wb_receiver_get_queue_length(receiver) > 0
    message = wb_receiver_get_data(receiver,'string');
    disp(['Received message: "' message '"']);
    wb_receiver_next_packet(receiver);
  end

  % get camera RGB image
  % this function return an image in true color (RGB) uint8 format
  rgb = wb_camera_get_image(camera);

  % display camera image
  subplot(2,2,1);
  image(rgb);
  title('RGB Camera');

  % every 10 time step
  if mod(steps,10) == 0
    % create intensity image from RGB image
    intens = double((rgb(:,:,1)+rgb(:,:,2)+rgb(:,:,3))/3);

    % edge detection
    edges = conv2(intens, conv_matrix, 'valid');

    % display edges image
    subplot(2,2,2);
    image(edges);
    colormap('gray');
    title('Edge detection');
  end

  % compute total foot pressures
  fsv(1,:) = wb_touch_sensor_get_values(fsr(1));
  fsv(2,:) = wb_touch_sensor_get_values(fsr(2));

  % The coefficients were calibrated against the real
  % robot so as to obtain realistic sensor values.
  l(1) = fsv(1,3)/3.4 + 1.5*fsv(1,1) + 1.15*fsv(1,2); % Left Foot Front Left
  l(2) = fsv(1,3)/3.4 + 1.5*fsv(1,1) - 1.15*fsv(1,2); % Left Foot Front Right
  l(3) = fsv(1,3)/3.4 - 1.5*fsv(1,1) - 1.15*fsv(1,2); % Left Foot Rear Right
  l(4) = fsv(1,3)/3.4 - 1.5*fsv(1,1) + 1.15*fsv(1,2); % Left Foot Rear Left

  r(1) = fsv(2,3)/3.4 + 1.5*fsv(2,1) + 1.15*fsv(2,2); % Right Foot Front Left
  r(2) = fsv(2,3)/3.4 + 1.5*fsv(2,1) - 1.15*fsv(2,2); % Right Foot Front Right
  r(3) = fsv(2,3)/3.4 - 1.5*fsv(2,1) - 1.15*fsv(2,2); % Right Foot Rear Right
  r(4) = fsv(2,3)/3.4 - 1.5*fsv(2,1) + 1.15*fsv(2,2); % Right Foot Rear Left

  left(steps) = 0;
  right(steps) = 0;
  for i = [1:4]
    l(i) = min(25, max(0, l(i)));
    r(i) = min(25, max(0, r(i)));
    left(steps) = left(steps) + l(i);
    right(steps) = right(steps) + r(i);
  end

  % plot foot pressures
  subplot(2,2,3);
  if steps <= 100
    time = [1:steps] * TIME_STEP / 1000;
    plot(time,left(),'b',time,right(),'r');
  else
    time = [steps-100:steps] * TIME_STEP / 1000;
    plot(time,left(steps-100:steps),'b',time,right(steps-100:steps),'r');
  end
  title('Left/right foot pressure');
  xlabel('time [s]');
  ylabel('Newtons [N]');

  % plot accelerometer values
  acc = wb_accelerometer_get_values(accelerometer);
  subplot(2,2,4);
  bar([1:3], acc);
  title('Accelerometers');
  xlabel('Axes X Y Z');
  ylabel('Acceleration [m/s^2]');
  axis([0.5 3.5 -5 15]);

  % flush graphics
  drawnow;
end

% your cleanup code goes here
