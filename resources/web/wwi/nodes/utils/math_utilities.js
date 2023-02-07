export function clampedAcos(value) {
  if (value >= 1)
    return 0;
  if (value <= -1)
    return 2 * Math.PI;
  return Math.acos(value);
}

export function isZeroAngle(angle) {
  const TWO_PI = 2 * Math.PI;
  const ZERO_ANGLE_THRESHOLD = 1e-10;
  const reminder = frac(angle / TWO_PI);
  if (Math.abs(reminder) < ZERO_ANGLE_THRESHOLD)
    return true;

  return false;
}

function trunc(n) {
  return (n < 0) ? Math.ceil(n) : Math.floor(n);
}

function frac(n) {
  return n - trunc(n);
}
