function base_move(vx, vy, omega)
  global WHEEL_RADIUS;
  global LX;
  global LY;
  speeds = [ 1 / WHEEL_RADIUS * (vx + vy - (LX + LY) * omega),
             1 / WHEEL_RADIUS * (vx - vy + (LX + LY) * omega),
             1 / WHEEL_RADIUS * (vx - vy - (LX + LY) * omega),
             1 / WHEEL_RADIUS * (vx + vy + (LX + LY) * omega)];
  base_set_wheel_speeds_helper(speeds);
  wb_console_print(sprintf('Speeds [m/s]: vx=%.2f vy=%.2f ω=%.2f', vx, vy, omega), WB_STDOUT)
end
