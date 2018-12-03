<?php
$useragent = isset($_SERVER['HTTP_USER_AGENT']) ? strtolower($_SERVER['HTTP_USER_AGENT']) : '';

function isWin() {
  return stripos($useragent, "windows nt ");
}

function isMac() {
  global $useragent;
  return stripos($useragent, "macintosh;");
}

function isLinux() {
  global $useragent;
  return stripos($useragent, "linux;");
}

function isIOS() {
  global $useragent;
  return stripos($useragent, 'iphone') || stripos($useragent, 'ipad') || stripos($useragent, 'ipod');
}

function isAndroid() {
  global $useragent;
  return stripos($useragent, 'android');
}

function isMobileDevice() {
  global $useragent;
  return isAndroid() || isIOS() || preg_match('/webOS|BlackBerry|IEMobile|Opera Mini/i', $useragent);
}
?>
