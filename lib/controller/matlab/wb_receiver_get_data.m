function result = wb_receiver_get_data(tag,type)
% Usage: wb_receiver_get_data(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/receiver">here</a>

pointer = calllib('libController', 'wb_receiver_get_data', tag);
size = calllib('libController', 'wb_receiver_get_data_size', tag);
if nargin < 2
  result = pointer;
elseif strcmp(type,'uint8')
  setdatatype(pointer, 'uint8Ptr', size);
  result = get(pointer, 'Value');
elseif strcmp(type,'double')
  size = size/8; % sizeof(double)
  setdatatype(pointer, 'doublePtr', size);
  result = get(pointer, 'Value');
elseif strcmp(type,'string')
  setdatatype(pointer, 'uint8Ptr', 1, size);
  result = char(get(pointer, 'Value'));
else
  result = '';
end
