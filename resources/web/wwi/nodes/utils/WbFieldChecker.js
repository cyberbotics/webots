import WbVector3 from './WbVector3.js';

function resetIfNonPositive(value, defaultValue) {
  if (value <= 0) {
    console.warn('Invalid ' + value + ' changed to ' + defaultValue + '. The value should be positive.');
    return defaultValue;
  }
  return false;
}

function resetIfNegative(value, defaultValue) {
  if (value < 0) {
    console.warn('Invalid ' + value + ' changed to ' + defaultValue + '. The value should be non-negative.');
    return defaultValue;
  }
  return false;
}

function resetVector2IfNonPositive(value, defaultValue) {
  if (value.x <= 0 || value.y <= 0) {
    console.warn('Invalid (' + value.x + ' ' + value.y + ') changed to (' + defaultValue.x + ' ' + defaultValue.y +
      '). The values should be positive.');
    return defaultValue;
  }
  return false;
}

function resetVector3IfNonPositive(value, defaultValue) {
  if (value.x <= 0 || value.y <= 0 || value.z <= 0) {
    console.warn('Invalid "size" changed to ' + defaultValue.x + ' ' + defaultValue.y + ' ' + defaultValue.z +
      '. The value should be positive.');
    return defaultValue;
  }
  return false;
}

function resetVector3IfNegative(value, defaultValue) {
  if (value.x < 0 || value.y < 0 || value.z < 0) {
    console.warn('Invalid value changed to ' + defaultValue.x + ' ' + defaultValue.y + ' ' + defaultValue.z +
      '. The value should be positive.');
    return defaultValue;
  }
  return false;
}

function resetIfNotInRangeWithIncludedBounds(value, min, max, defaultValue) {
  if (value < min || value > max) {
    console.warn('Invalid ' + value + ' changed to ' + defaultValue + '. The value should be in range [' + min + ', ' + max +
      '].');
    return defaultValue;
  }
  return false;
}

function resetMultipleColorIfInvalid(colors) {
  let changed = false;
  for (let i = 0; i < colors.length; i++) {
    let newValue = resetColorIfInvalid(colors[i]);
    if (newValue !== false) {
      colors[i] = newValue;
      changed = true;
    }
  }

  return changed ? colors : false;
}

function resetColorIfInvalid(value) {
  const clampedColor = clampValuesIfNeeded(value);
  if (value.x !== clampedColor.x || value.y !== clampedColor.y || value.z !== clampedColor.z) {
    console.warn('Invalid color ' + value + ' changed to ' + clampedColor);
    return clampedColor;
  }
  return false;
}

function clampValuesIfNeeded(value) {
  return new WbVector3(clampValue(value.x), clampValue(value.y), clampValue(value.z));
}

function clampValue(value) {
  if (value < 0.0)
    return 0.0;
  if (value > 1.0)
    return 1.0;
  return value;
}

export {resetIfNegative, resetIfNonPositive, resetVector2IfNonPositive, resetVector3IfNonPositive, clampValuesIfNeeded,
  resetIfNotInRangeWithIncludedBounds, resetColorIfInvalid, resetMultipleColorIfInvalid, resetVector3IfNegative};
