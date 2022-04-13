function resetDoubleIfNonPositive(value, defaultValue) {
  if (value <= 0) {
    console.error('Invalid ' + value + ' changed to ' + defaultValue + '. The value should be positive.');
    value = defaultValue;
    return true;
  }
  return false;
}

function resetVector3IfNonPositive(value, defaultValue) {
  if (value.x <= 0 || value.y <= 0 || value.z <= 0) {
    console.error('Box: Invalid "size" changed to ' + defaultValue.x + ' ' + defaultValue.y + ' ' + defaultValue.z + '. The value should be positive.');
    return defaultValue;
  }
}

function resetIntIfNotInRangeWithIncludedBounds(value, min, max, defaultValue) {
  if (value < min || value > max) {
    console.error('Invalid ' + value + ' changed to ' + defaultValue + '. The value should be in range [' + min + ', ' + max + '].');
    return defaultValue;
  }
}

export {resetDoubleIfNonPositive, resetVector3IfNonPositive, resetIntIfNotInRangeWithIncludedBounds};
