function requestFullscreen(view) {
  const elem = view.view3D;
  if (elem.requestFullscreen)
    elem.requestFullscreen();
  else if (elem.msRequestFullscreen)
    elem.msRequestFullscreen();
  else if (elem.mozRequestFullScreen)
    elem.mozRequestFullScreen();
  else if (elem.webkitRequestFullscreen)
    elem.webkitRequestFullscreen();
}

function exitFullscreen() {
  if (document.exitFullscreen)
    document.exitFullscreen();
  else if (document.msExitFullscreen)
    document.msExitFullscreen();
  else if (document.mozCancelFullScreen)
    document.mozCancelFullScreen();
  else if (document.webkitExitFullscreen)
    document.webkitExitFullscreen();
}

function onFullscreenChange(fullscreenButton, exitFullscreenButton) {
  const element = document.fullScreenElement || document.mozFullScreenElement || document.webkitFullScreenElement || document.msFullScreenElement || document.webkitCurrentFullScreenElement;
  if (element != null) {
    fullscreenButton.style.display = 'none';
    exitFullscreenButton.style.display = 'inline';
  } else {
    fullscreenButton.style.display = 'inline';
    exitFullscreenButton.style.display = 'none';
  }
}

export {requestFullscreen, exitFullscreen, onFullscreenChange};
