function arm_decrease_orientation()
  global ORIENTATIONS_ID;
  global ORIENTATIONS;
  global current_orientation;
  new_orientation_id = ORIENTATIONS_ID(current_orientation) -1;
  if (new_orientation_id < 1)
    new_orientation_id = 1;
  end
  arm_set_orientation(ORIENTATIONS(new_orientation_id));
end
