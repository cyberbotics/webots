#!/usr/bin/php
<?php
function ftp_reconnect($conn_id, $password) {
  echo "reconnecting...\n";
  ftp_close($conn_id);
  $conn_id = ftp_connect("www.cyberbotics.com");
  ftp_login($conn_id,"contest",$password);
  ftp_pasv($conn_id, true);
  ftp_chdir($conn_id,"robotstadium");
  return $conn_id;

}

echo "webcam.php: started...\n";
$password = getenv("CYBERBOTICS_FTP_PASSWORD")

if ($password !== false) {
  $conn_id = ftp_connect("www.cyberbotics.com");
  ftp_login($conn_id, "contest", $password);
  ftp_pasv($conn_id, true);
  ftp_chdir($conn_id,"robotstadium");

  // most recent existing image #
  $count=-1;

  while (1) {
    //echo $count."\n";

    usleep(40000);

    // find the most recent image
    while (1) {
      $count++;
      $image=sprintf("/tmp/movie%06d.tga", $count);
      //echo $image;
      if (! file_exists($image)) {
        $count--;
        break;
      }
    }

    if ($count > 0) {
      // use image #count-1 because image #count may not be valid yet
      $image=sprintf("/tmp/movie%06d.tga",$count-1);
      system("convert -quality 65 ".$image." webcam.jpg");
      while (!ftp_put($conn_id,"webcam-tmp.jpg","webcam.jpg",FTP_BINARY)) $conn_id=ftp_reconnect($conn_id,$password);
      while (!ftp_rename($conn_id,"webcam-tmp.jpg","webcam.jpg")) $conn_id=ftp_reconnect($conn_id,$password);
    }
  }

  ftp_close($conn_id);
} else
    echo "'CYBERBOTICS_FTP_PASSWORD' environmental variable not set."

echo "webcam.php: stopped."
?>
