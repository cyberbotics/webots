// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
