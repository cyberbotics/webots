function base_init
  global wheel
  for i=1:4
    wheel(i) = wb_robot_get_device(sprintf('wheel%d', i));
  end
end
