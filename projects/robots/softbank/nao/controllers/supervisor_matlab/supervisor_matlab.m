% MATLAB controller for Webots
% Description:   An example of a Supervisor in MATLAB for the Nao Soccer
%                This MATLAB code reads the current robot and ball positions and plots them in 2d soccer field depiction
% Prerequisite: -This works only for nao2_matlab.wbt file or .wbt files that do contain the corresponding robot and the ball
%               -MATLAB must be installed and the "matlab" command must be in the PATH environment variable
function supervisor_matlab

% controller time step
TIME_STEP = 40;

% indices
X=1;
Z=3;

% get nodes: these nodes must exist!
red_nao = wb_supervisor_node_get_from_def('RED_PLAYER_1');
blue_nao = wb_supervisor_node_get_from_def('BLUE_GOAL_KEEPER');
ball = wb_supervisor_node_get_from_def('BALL');

% get fields
red_field = wb_supervisor_node_get_field(red_nao,'translation');
blue_field = wb_supervisor_node_get_field(blue_nao,'translation');
ball_field = wb_supervisor_node_get_field(ball,'translation');

% get device tag
emitter = wb_robot_get_device('emitter');

% count the steps
steps = 0;

% main loop
while wb_robot_step(TIME_STEP) ~= -1
  steps = steps + 1;

  % read current positions
  red_pos = wb_supervisor_field_get_sf_vec3f(red_field);
  blue_pos = wb_supervisor_field_get_sf_vec3f(blue_field);
  ball_pos = wb_supervisor_field_get_sf_vec3f(ball_field);

  % every 10 time step
  if mod(steps,10) == 0
    % send an example text message
    wb_emitter_send(emitter, uint8(['the ball is located at ' num2str(ball_pos,'%g %g %g')]));
  end
  
  % plot ball position (yellow)
  plot(ball_pos(X),-ball_pos(Z),'yo','MarkerSize',10,'MarkerFaceColor','y');
  hold on;
  
  % plot robot positions
  plot(red_pos(X),-red_pos(Z),'rs','MarkerSize',10,'MarkerFaceColor','r');
  plot(blue_pos(X),-blue_pos(Z),'bs','MarkerSize',10,'MarkerFaceColor','b');
  
  % draw soccer field
  rectangle('Position',[-3,-2,6,4]);  % end lines
  rectangle('Position',[-3,-1.5,0.6,3]);  % blue goal penalty area
  rectangle('Position',[2.4,-1.5,0.6,3]);  % red goal penalty area
  line([0 0],[-2 2],'color','k');  % halfway line
  rectangle('Position',[-0.6,-0.6,1.2,1.2],'Curvature',[1,1]);  % central circle

  % draw goals
  line([-3 -3],[-0.7 0.7],'Color','c','LineWidth',4);
  line([3 3],[-0.7 0.7],'Color','y','LineWidth',4);

  % entire field limits
  axis([-3.7 3.7 -2.7 2.7]);
  hold off;

  % flush graphics
  drawnow;
end

% cleanup code goes here: write data to files, etc.
