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
  if (isFullscreen() != null) {
    fullscreenButton.style.display = 'none';
    exitFullscreenButton.style.display = 'inline';
  } else {
    fullscreenButton.style.display = 'inline';
    exitFullscreenButton.style.display = 'none';
  }
}

function isFullscreen() {
  return (document.fullscreenElement || document.mozFullScreenElement || document.webkitCurrentFullScreenElement);
}

export {requestFullscreen, exitFullscreen, onFullscreenChange, isFullscreen};
