function resetVector3IfNonPositive(value, defaultValue) {
  if (value.x <= 0 || value.y <= 0 || value.z <= 0) {
    console.error('Box: Invalid "size" changed to ' + defaultValue.x + ' ' + defaultValue.y + ' ' + defaultValue.z + '. The value should be positive.');
    return defaultValue;
  }
}

export {resetVector3IfNonPositive};
