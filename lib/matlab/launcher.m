% Launcher script for MATLAB Webots controllers

% useful env variables supplied by webots
WEBOTS_HOME = getenv('WEBOTS_HOME');
WEBOTS_CONTROLLER_NAME = getenv('WEBOTS_CONTROLLER_NAME');
WEBOTS_VERSION = getenv('WEBOTS_VERSION');

if isempty(WEBOTS_CONTROLLER_NAME)
  disp('Entering test mode (normally launcher.m should be called by Webots, not from the MATLAB command line)');
  disp(['Using MATLAB R' version('-release')]);
  cd('../..');
  WEBOTS_HOME = pwd;
  [status, cmdout] = system('msys64/mingw64/bin/webots.exe --version');
  WEBOTS_VERSION = strrep(cmdout(17:end-1),'.','_');
  cd('lib/matlab');
  disp(['Using Webots ' strrep(WEBOTS_VERSION,'_','.') ' from ' pwd ])
  test_mode = true;
else
  test_mode = false;
end

% add path to Webots API m-files
addpath([WEBOTS_HOME '/lib/matlab']);

if ispc
  setenv('MINGWROOT', strcat(WEBOTS_HOME,'\\msys64\\mingw64'));
  libname = 'Controller';
  % Matlab 64-bits on Windows is not provided with a C compiler. We should provide the one of Webots (gcc).
  file = [ winqueryreg('HKEY_CURRENT_USER', ['Software\Microsoft\Windows\CurrentVersion\Explorer\Shell Folders'], 'AppData') '\MathWorks\MATLAB\R' version('-release') '\mex_C_win64.xml' ];
  if exist(file, 'file')             % If this file exists, the compiler was already setup
    disp(['Using MEX file: ' file]); % no need to do anything
  else                               % otherwise, we need to set it up.
    mex -setup:mex_mingw64_c.xml C;
  end
  addpath([WEBOTS_HOME '/msys64/mingw64/bin']);
else
  libname = 'libController';
  % add path to libController
  addpath([WEBOTS_HOME '/lib']);
end

try
  % work in the system's temp dir so we have write access
  cd(tempdir); % that supports non-ASCII character since MATLAB 2016a

  % creates the base name of the protofile (without .m extension)
  % we make the name dependant on matlab and webots versions, so a new file will be generated for any version change
  % we need to replace the dots by underscores, because dots cause problems in the protofile
  protofile = strrep(strrep(['protofile_matlab_' version('-release') '_webots_' WEBOTS_VERSION], '.', '_'), ' ', '_');

  % another controller is currently generating the proto file: need to wait
  lockfile = 'webots_matlab_lock';
  counter = 1;
  while exist(lockfile, 'file')
    if (counter == 1)
      disp(['Waiting up to 5 seconds for ' tempdir lockfile ' to be deleted by another MATLAB instance.']);
    end
    pause(1);
    if (counter == 5)
      disp(['Deleting ' tempdir lockfile '...'])
      delete(lockfile);
    end
    counter = counter + 1;
  end

  libcontrollerloaded = false;

  if ~exist([protofile '.m'], 'file')

    % create a lock to prevent any other matlab controller from generating the proto file simultaneously
    fid = fopen(lockfile, 'w');
    fclose(fid);

    disp(['Creating: ' tempdir protofile '.m']);
    try
      loadlibrary( ...
        libname,'allincludes.h', ...
        'mfilename',protofile, ...
        'alias','libController', ...
        'addheader','accelerometer.h', ...
        'addheader','brake.h', ...
        'addheader','camera.h', ...
        'addheader','compass.h', ...
        'addheader','connector.h', ...
        'addheader','console.h', ...
        'addheader','device.h', ...
        'addheader','differential_wheels.h', ...
        'addheader','display.h', ...
        'addheader','distance_sensor.h', ...
        'addheader','emitter.h', ...
        'addheader','gps.h', ...
        'addheader','gyro.h', ...
        'addheader','inertial_unit.h', ...
        'addheader','joystick.h', ...
        'addheader','keyboard.h', ...
        'addheader','led.h', ...
        'addheader','lidar.h', ...
        'addheader','light_sensor.h', ...
        'addheader','motor.h', ...
        'addheader','mouse.h', ...
        'addheader','pen.h', ...
        'addheader','position_sensor.h', ...
        'addheader','radar.h', ...
        'addheader','range_finder.h', ...
        'addheader','receiver.h', ...
        'addheader','robot.h', ...
        'addheader','skin.h', ...
        'addheader','speaker.h', ...
        'addheader','supervisor.h', ...
        'addheader','touch_sensor.h', ...
        'addheader',['utils' filesep 'motion.h'], ...
        'addheader',['utils' filesep 'system.h']);
      disp('Load Library successful');
      libcontrollerloaded = true;
    catch ME % this happens with MATLAB R2015a only but seems to be harmless
      if (version('-release') == '2015a')
        loadlibrary(libname,protofile,'alias','libController');
        libcontrollerloaded = true;
      else
        disp('Load Library failed.');
        rethrow(ME);
        loadlibrary = false;
      end
    end
    delete(lockfile);
  else
    disp(['Using prototype file: ' tempdir protofile '.m']);
    loadlibrary(libname,protofile,'alias','libController');
    libcontrollerloaded = true;
  end

  if test_mode == true
    unloadlibrary('libController');
    cd([WEBOTS_HOME '/lib/matlab']);
    disp('Test successful.');
    return
  end

  % initialize libController and redirect stdout/stderr
  try
    calllib('libController', 'wb_robot_init');
  catch
    calllib('libController', 'wb_robot_init_msvc');
  end

  % start controller
  WEBOTS_PROJECT_UTF8 = wbu_system_getenv('WEBOTS_PROJECT');
  WEBOTS_PROJECT = wbu_system_short_path(WEBOTS_PROJECT_UTF8);

  cd([WEBOTS_PROJECT '/controllers/' WEBOTS_CONTROLLER_NAME]);
  eval(WEBOTS_CONTROLLER_NAME);

catch ME
  % display error message in Webots console
  % use stderr to display message in red (this does not work on Windows)
  err = getReport(ME, 'extended');
  fprintf(2,'%s\n\n',err);
  if ispc
    if libcontrollerloaded
      % only try to put the error on the console if the library has been loaded
      calllib('libController', 'wb_console_print', err, 1);
      exit(-1);
    end
    % on Windows, exiting systematically would imply to lose the error message
  else
    exit(-1);
  end
end
