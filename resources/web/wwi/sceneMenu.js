import informationPanel from 'https://cyberbotics.com/wwi/R2022b/informationPanel.js';

function createSceneMenu() {
  let webotsView = document.getElementsByTagName('webots-view')[0];
  if (!webotsView)
    return;

  let div = document.createElement('div');
  div.id = 'informationPlaceHolder';
  div.innerHTML = informationPanel;
  div.style.width = '100%';
  div.style.height = '100%';
  div.style.position = 'absolute';
  div.style.top = '0px';
  div.style.left = '0px';
  div.style.pointerEvents = 'none';
  div.style.zIndex = 1;
  webotsView.appendChild(div);

  let menuItems = document.createElement('div');
  menuItems.className = 'menu-items';

  let infoButton = document.createElement('button');
  infoButton.className = 'info-button ui-icon';
  infoButton.title = 'Simulation information';
  menuItems.appendChild(infoButton);

  let resetButton = document.createElement('button');
  resetButton.className = 'reset-button ui-icon';
  resetButton.title = 'Reset Viewpoint and sliders';
  resetButton.onclick = () => webotsView.resetViewpoint();
  menuItems.appendChild(resetButton);

  let fullscreenButton = document.createElement('button');
  fullscreenButton.className = 'fullscreen-button ui-icon';
  fullscreenButton.title = 'Enter fullscreen';
  fullscreenButton.onclick = () => toggleRobotComponentFullScreen();
  menuItems.appendChild(fullscreenButton);

  let exitFullscreenButton = document.createElement('button');
  exitFullscreenButton.className = 'exit-fullscreen-button ui-icon';
  exitFullscreenButton.title = 'Leave fullscreen';
  exitFullscreenButton.onclick = () => toggleRobotComponentFullScreen();
  exitFullscreenButton.style.display = 'none';
  menuItems.appendChild(exitFullscreenButton);

  webotsView.appendChild(menuItems);

  webotsView.onmouseenter = () => showButtons();
  webotsView.onmouseleave = () => hideButtons();

  if (document.getElementsByClassName('info-button').length !== 0)
    document.getElementsByClassName('info-button')[0].onclick = () => displayInformationWindow();

  window.addEventListener('click', closeInfoOnClick);
}

function closeSceneMenu() {
  let informationPlaceHolder = document.getElementById('informationPlaceHolder');
  if (informationPlaceHolder)
    informationPlaceHolder.parentNode.removeChild(informationPlaceHolder);

  let menuItems = document.getElementsByClassName('menu-items');
  if (menuItems)
    menuItems.parentNode.removeChild(menuItems);
  else
    console.log('CRASH');

  window.removeEventListener('click', closeInfoOnClick);
}

function closeInfoOnClick(event) {
  let infoPanel = document.getElementsByClassName('information-panel')[0];
  if (infoPanel && !infoPanel.contains(event.target) && !document.getElementsByClassName('info-button')[0].contains(event.target))
    infoPanel.style.display = 'none';
}

function showButtons() {
  if (document.getElementsByClassName('info-button').length !== 0)
    document.getElementsByClassName('info-button')[0].style.display = '';

  if (document.getElementsByClassName('reset-button').length !== 0)
    document.getElementsByClassName('reset-button')[0].style.display = '';

  if (document.getElementsByClassName('fullscreen-button').length !== 0 && !document.fullscreenElement)
    document.getElementsByClassName('fullscreen-button')[0].style.display = '';

  if (document.getElementsByClassName('exit-fullscreen-button').length !== 0 && document.fullscreenElement)
    document.getElementsByClassName('exit-fullscreen-button')[0].style.display = '';
}

function hideButtons() {
  if (document.getElementsByClassName('info-button').length !== 0)
    document.getElementsByClassName('info-button')[0].style.display = 'none';

  if (document.getElementsByClassName('reset-button').length !== 0)
    document.getElementsByClassName('reset-button')[0].style.display = 'none';

  if (document.getElementsByClassName('fullscreen-button').length !== 0)
    document.getElementsByClassName('fullscreen-button')[0].style.display = 'none';

  if (document.getElementsByClassName('exit-fullscreen-button').length !== 0)
    document.getElementsByClassName('exit-fullscreen-button')[0].style.display = 'none';
}

function displayInformationWindow() {
  let infoPanel = document.getElementsByClassName('information-panel')[0];

  if (infoPanel) {
    if (infoPanel.style.display === 'block')
      infoPanel.style.display = 'none';
    else
      infoPanel.style.display = 'block';
  }
}

function toggleRobotComponentFullScreen(robot) {
  if (document.fullscreenElement) {
    document.getElementsByClassName('fullscreen-button')[0].style.display = '';
    document.getElementsByClassName('exit-fullscreen-button')[0].style.display = 'none';

    if (document.exitFullscreen)
      document.exitFullscreen();
  } else {
    document.getElementsByClassName('fullscreen-button')[0].style.display = 'none';
    document.getElementsByClassName('exit-fullscreen-button')[0].style.display = '';
    if (document.getElementsByTagName('webots-view')[0].requestFullscreen)
      document.getElementsByTagName('webots-view')[0].requestFullscreen();
  }
}

export { createSceneMenu, closeSceneMenu};
