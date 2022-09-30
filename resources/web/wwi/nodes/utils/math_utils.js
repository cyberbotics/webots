function clampedAcos(value) {
  if (value >= 1.0)
    return 0.0;
  if (value <= -1.0)
    return 2.0 * Math.PI;
  return Math.acos(value);
}

export {clampedAcos};
