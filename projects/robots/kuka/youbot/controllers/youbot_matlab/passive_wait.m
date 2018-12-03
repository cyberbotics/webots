function passive_wait(sec)
  start_time = wb_robot_get_time();
  while (start_time + sec > wb_robot_get_time())
    if (wb_robot_step(32)==-1)
      quit
  end
end
