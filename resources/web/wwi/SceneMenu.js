import InformationPanel from './InformationPanel.js';
import WbWorld from './nodes/WbWorld.js';

export default class SceneMenu {
  constructor() {
    this.webotsView = document.getElementsByTagName('webots-view')[0];
    if (!this.webotsView)
      return;

    this.informationPlaceHolder = document.createElement('div');
    this.informationPlaceHolder.id = 'informationPlaceHolder';
    this.informationPanel = new InformationPanel(this.informationPlaceHolder);
    this.informationPlaceHolder.style.width = '100%';
    this.informationPlaceHolder.style.height = '100%';
    this.informationPlaceHolder.style.position = 'absolute';
    this.informationPlaceHolder.style.top = '0px';
    this.informationPlaceHolder.style.left = '0px';
    this.informationPlaceHolder.style.pointerEvents = 'none';
    this.informationPlaceHolder.style.zIndex = 1;
    this.webotsView.appendChild(this.informationPlaceHolder);

    this.menuItems = document.createElement('div');
    this.menuItems.className = 'menu-items';

    let infoButton = document.createElement('button');
    infoButton.className = 'info-button ui-icon';
    infoButton.title = 'Simulation information';
    this.menuItems.appendChild(infoButton);

    let fullscreenButton = document.createElement('button');
    fullscreenButton.className = 'fullscreen-button ui-icon';
    fullscreenButton.title = 'Enter fullscreen';
    fullscreenButton.onclick = () => this._toggleRobotComponentFullScreen();
    this.menuItems.appendChild(fullscreenButton);

    let exitFullscreenButton = document.createElement('button');
    exitFullscreenButton.className = 'exit-fullscreen-button ui-icon';
    exitFullscreenButton.title = 'Leave fullscreen';
    exitFullscreenButton.onclick = () => this._toggleRobotComponentFullScreen();
    exitFullscreenButton.style.display = 'none';
    this.menuItems.appendChild(exitFullscreenButton);

    this.webotsView.appendChild(this.menuItems);

    this.webotsView.onmouseenter = () => this._showButtons();
    this.webotsView.onmouseleave = () => this._hideButtons();

    if (document.getElementsByClassName('info-button').length !== 0)
      document.getElementsByClassName('info-button')[0].onclick = () => this._displayInformationWindow();

    window.addEventListener('click', this._closeInfoOnClick);

    this._setTitleAndDescription();
  }

  close() {
    if (typeof this.informationPlaceHolder !== 'undefined' && this.informationPlaceHolder.parentNode)
      this.informationPlaceHolder.parentNode.removeChild(this.informationPlaceHolder);

    if (typeof this.menuItems !== 'undefined' && this.menuItems.parentNode)
      this.menuItems.parentNode.removeChild(this.menuItems);

    window.removeEventListener('click', this._closeInfoOnClick);
  }

  _setTitleAndDescription() {
    if (typeof WbWorld.instance !== 'undefined' && WbWorld.instance.readyForUpdates) {
      this.informationPanel.setTitle(WbWorld.instance.title);
      this.informationPanel.setDescription(WbWorld.instance.description);
    } else
      setTimeout(() => this._setTitleAndDescription(), 100);
  }

  _closeInfoOnClick(event) {
    let infoPanel = document.getElementsByClassName('information-panel')[0];
    if (infoPanel && !infoPanel.contains(event.target) && !document.getElementsByClassName('info-button')[0].contains(event.target))
      infoPanel.style.display = 'none';
  }

  _showButtons() {
    if (document.getElementsByClassName('info-button').length !== 0)
      document.getElementsByClassName('info-button')[0].style.display = '';

    if (document.getElementsByClassName('reset-button').length !== 0)
      document.getElementsByClassName('reset-button')[0].style.display = '';

    if (document.getElementsByClassName('fullscreen-button').length !== 0 && !document.fullscreenElement)
      document.getElementsByClassName('fullscreen-button')[0].style.display = '';

    if (document.getElementsByClassName('exit-fullscreen-button').length !== 0 && document.fullscreenElement)
      document.getElementsByClassName('exit-fullscreen-button')[0].style.display = '';
  }

  _hideButtons() {
    if (document.getElementsByClassName('info-button').length !== 0)
      document.getElementsByClassName('info-button')[0].style.display = 'none';

    if (document.getElementsByClassName('reset-button').length !== 0)
      document.getElementsByClassName('reset-button')[0].style.display = 'none';

    if (document.getElementsByClassName('fullscreen-button').length !== 0)
      document.getElementsByClassName('fullscreen-button')[0].style.display = 'none';

    if (document.getElementsByClassName('exit-fullscreen-button').length !== 0)
      document.getElementsByClassName('exit-fullscreen-button')[0].style.display = 'none';
  }

  _displayInformationWindow() {
    let infoPanel = document.getElementsByClassName('information-panel')[0];

    if (infoPanel) {
      if (infoPanel.style.display === 'block')
        infoPanel.style.display = 'none';
      else
        infoPanel.style.display = 'block';
    }
  }

  _toggleRobotComponentFullScreen(robot) {
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
}
