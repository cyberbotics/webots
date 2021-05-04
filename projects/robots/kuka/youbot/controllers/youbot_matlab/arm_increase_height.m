function arm_increase_height()
  global HEIGHTS_ID;
  global HEIGHTS;
  global current_height;
  new_height_id = HEIGHTS_ID(current_height) + 1;
  if (new_height_id > HEIGHTS_ID.Count)
    new_height_id = HEIGHTS_ID.Count;
  end
  arm_set_height(HEIGHTS(new_height_id));
end
