<?php
# Script called by the 'webots.min.js' library to save the controller files modified by the user from the editor in the web interface.
header('X-Robots-Tag: noindex');

function folder_size($dir) {
  $count_size = 0;
  $count = 0;
  $dir_array = scandir($dir);
  foreach($dir_array as $key=>$filename) {
    if ($filename != '..' && $filename != '.') {
      if (is_dir($dir.'/'.$filename)){
        $new_foldersize = folder_size($dir.'/'.$filename);
        $count_size = $count_size + $new_foldersize;
      } else if (is_file($dir.'/'.$filename)) {
        $count_size = $count_size + filesize($dir."/".$filename);
        $count++;
      }
    }
  }
  return $count_size;
}
if (!isset($_COOKIE['email']))
  die("Error: Missing email cookie.");
if (!isset($_COOKIE['password']))
  die("Error: Missing password cookie.");
$id = ''; # TODO retrieve user id from database given the email and the password of the logged in user

if (!isset($_POST['dirname']))
  die("Error: missing dirname.");
if (!isset($_POST['filename']))
  die("Error: missing filename.");
if (!isset($_POST['content']))
  die("Error: missing dirname.");
$size = strlen($_POST['content']);
$dirname = $_POST['dirname'];
$filename = $_POST['filename'];
$content = $_POST['content'];
if (strpos($filename, '/') !== false)
  die("Error: wrong filename.");
if (strpos($dirname, '..') !== false)
  die("Error: wrong dirname.");

# Store the modified file in the user project directory.
$path = "user/$id/projects/$dirname";
if (!file_exists($path))
  mkdir($path, 0777, true);
$size += folder_size("user/$id/projects");
$size /= 1024 * 1024; // expressed in MB
$size = round($size, 1);
$quota = 10; // expressed in MB
if ($size > $quota)
  die("Error: Disk quota exceeded:<br>$size MB > $quota MB.<br><br>File $filename was not saved on server.");
$filename = $path.'/'.$filename;
$fp = fopen($filename, 'w');
fwrite($fp, $content);
fclose($fp);
die("OK");
?>
