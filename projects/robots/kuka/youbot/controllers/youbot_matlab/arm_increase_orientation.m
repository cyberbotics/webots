function arm_increase_orientation()
  global ORIENTATIONS_ID;
  global ORIENTATIONS;
  global current_orientation;
  new_orientation_id = ORIENTATIONS_ID(current_orientation) + 1;
  if (new_orientation_id > ORIENTATIONS_ID.Count)
    new_orientation_id = ORIENTATIONS_ID.Count;
  end
  arm_set_orientation(ORIENTATIONS(new_orientation_id));
end
