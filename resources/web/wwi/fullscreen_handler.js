function requestFullscreen(view) {
  const elem = view.view3D;
  if (typeof elem.requestFullscreen !== 'undefined')
    elem.requestFullscreen();
}

function exitFullscreen() {
  if (typeof document.exitFullscreen !== 'undefined')
    document.exitFullscreen();
}

function onFullscreenChange(fullscreenButton, exitFullscreenButton) {
  const element = document.fullScreenElement || document.mozFullScreenElement || document.webkitCurrentFullScreenElement;
  if (element != null) {
    fullscreenButton.style.display = 'none';
    exitFullscreenButton.style.display = 'inline';
  } else {
    fullscreenButton.style.display = 'inline';
    exitFullscreenButton.style.display = 'none';
  }
}

export {requestFullscreen, exitFullscreen, onFullscreenChange};
