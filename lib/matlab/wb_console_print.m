function wb_console_print(txt, stream)
% Usage: wb_console_print(txt, stream)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/guide/using-matlab">here</a>

calllib('libController', 'wb_console_print', txt, stream);
