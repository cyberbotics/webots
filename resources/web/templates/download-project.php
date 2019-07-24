<?php
# Script called by the 'simulation_server.py' script to download the project data from the session server machine
# and copy it on the simulation server machine in the temporary directory for the newly instantiated Webots instance.
header('X-Robots-Tag: noindex');

class DirZipArchive extends ZipArchive {
  # Add directory recursively in the ZIP archive.
  public function addDir($location, $name) {
    if (!empty($name)) {
      $this->addEmptyDir($name);
      $name .= '/';
    }
    $location .= '/';
    $dir = opendir($location);
    while ($file = readdir($dir)) {
      if ($file == '.' || $file == '..')
        continue;
      $do = (filetype($location.$file) == 'dir') ? 'addDir' : 'addFile';
      $this->$do($location.$file, $name.$file);
    }
  }
}

$mysqli->set_charset('utf8');
if (isset($_POST['user1Id']))
  $user1Id = $mysqli->escape_string($_POST['user1Id']);
else
  die("Error: Missing user1Id.");
if (isset($_POST['user1Authentication']))
  $user1Authentication = $mysqli->escape_string($_POST['user1Authentication']);
else
  die("Error: Missing user1Authentication.");
if (isset($_POST['key']))
  $key = $mysqli->escape_string($_POST['key']);
else
  die("Error: Missing key.");
if (isset($_POST['project']))
  $project = $mysqli->escape_string($_POST['project']);
else
  die("Error: Missing project.");

# Set a unique key to identify the host and authenticate the request.
# The host key should match the one passed to the 'session_server.py' and located in the path specified in the "keyDir" config parameter.
$host_key = ''; # Replace this empty string with the actual host key value
if ($key != $host_key) {
  sleep(1); # to prevent brute force attacks
  die("Error: Wrong key." );
}

# Check authentication of user1 given $user1Authentication and $user1Id.
if (!empty($user1Authentication)) {
  # TODO
}

# Retrieve project path to be downloaded.
if (strpos($project, '/') !== false || strpos($project, '..') !== false)
  die("Error: Wrong project.");
if (empty($user1Authentication))
  $dir = "user/$user1Id/projects/$project"; # TODO adjust to application directory hierarchy
else
  $dir = "projects/$project"; # TODO adjust to application directory hierarchy
if (!file_exists($dir))
  die("Error: no such directory: '$dir'.");

# Create and return a ZIP Archive containing the requested project.
$zip = new DirZipArchive();
$zip_name = tempnam(sys_get_temp_dir(), 'project').'.zip';
$zip->open($zip_name, ZipArchive::CREATE);
$zip->addDir($dir, '');
$zip->close();
header('Content-Type: application/zip');
header('Content-disposition: attachment; filename='.$project.'.zip');
header('Content-Length: '.filesize($zip_name));
readfile($zip_name);
unlink($zip_name)
?>
