function arm_decrease_height()
  global HEIGHTS_ID;
  global HEIGHTS;
  global current_height;
  new_height_id = HEIGHTS_ID(current_height) -1;
  if (new_height_id < 1)
    new_height_id = 1;
  end
  arm_set_height(HEIGHTS(new_height_id));
end
