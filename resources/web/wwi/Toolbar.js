import Animation from './Animation.js';
import AnimationSlider from './AnimationSlider.js';
import {requestFullscreen, exitFullscreen, onFullscreenChange, isFullscreen} from './fullscreen_handler.js';
import InformationPanel from './InformationPanel.js';
import FloatingIde from './FloatingIde.js';
import FloatingRobotWindow from './FloatingRobotWindow.js';
import {changeShadows, changeGtaoLevel, GtaoLevel} from './nodes/wb_preferences.js';
import SystemInfo from './system_info.js';
import WbWorld from './nodes/WbWorld.js';

export default class Toolbar {
  constructor(view, type, parentNode) {
    this._view = view;
    this.type = type;
    this.parentNode = parentNode;
    this.minWidth = 0;
    this._scale = 1;

    this._createToolbar(parentNode);
    if (type === 'animation')
      this.createAnimationToolbar();
    else if (type === 'scene')
      this.createSceneToolbar();
    else if (type === 'streaming')
      this.createStreamingToolbar();
    this._resizeToolbar();
    this.toolbar.style.minWidth = this.minWidth + 'px';

    if (SystemInfo.isMobileDevice() && !SystemInfo.isIOS() && !SystemInfo.isSafari()) {
      // Warning: window.orientation is deprecated but screen.orientation is not supported by iOS so we use it in this case.
      if (!screen.orientation || screen.orientation === 'undefined')
        this._previousScreenOrientation = window.orientation % 180 === 0 ? 'portrait' : 'landscape';
      else
        this._previousScreenOrientation = screen.orientation.type.includes('portrait') ? 'portrait' : 'landscape';

      screen.orientation.addEventListener("change", this._mobileOrientationChangeFullscreen.bind(this));

      this._fullscreenButton.style.animation = 'animation-scale-up-lg 2s infinite forwards';
      this._fullscreenButton.addEventListener('click', () => this._removeFullscreenAnimation(this._fullscreenButton),
        {once: true});
    }
  }

  createAnimationToolbar() {
    if (this.type !== 'animation' || typeof this._view === 'undefined' || typeof this._view.animation === 'undefined')
      return;

    this._createSlider();

    // Left part
    this._createPlayButton();
    this._createAnimationTimeIndicator();
    this._checkLeftTooltips();

    // Right part
    this._createInfoButton();
    this._createSettings();
    if (!SystemInfo.isIOS() && !SystemInfo.isSafari())
      this._createFullscreenButtons();
  }

  createSceneToolbar() {
    if (this.type !== 'scene' || typeof this._view === 'undefined')
      return;

    this._createInfoButton();
    this._createSettings();
    if (!SystemInfo.isIOS() && !SystemInfo.isSafari())
      this._createFullscreenButtons();
  }

  createStreamingToolbar() {
    // Left part
    this._createQuitButton();
    this._createReloadButton();
    this._createWorldSelectionButton();
    this._createResetButton();
    this._createStepButton();
    this._createPlayButton();
    this._createRunButton();
    this._createStreamingTimeIndicator();
    this._checkLeftTooltips();
    if (this._view.broadcast) {
      this.toolbarLeft.style.visibility = 'hidden';
      this.minWidth = 0;
    }

    // Right part
    this._createIdeButton();
    this._createRobotWindowButton();
    this._createInfoButton();
    if (this._view.mode !== 'mjpeg')
      this._createSettings();
    if (!SystemInfo.isIOS() && !SystemInfo.isSafari())
      this._createFullscreenButtons();
  }

  _resizeToolbar() {
    this._toolbarResizeObserver = new ResizeObserver(() => {
      this._scale = this.minWidth > this.parentNode.offsetWidth ? this.parentNode.offsetWidth / this.minWidth : 1;
      this.toolbar.style.transformOrigin = 'bottom left';
      this.toolbar.style.transform = 'scale(' + this._scale + ')';
      if (typeof this.robotWindowPane !== 'undefined') {
        const offset = this._scale === 1 ? 0 : this.parentNode.offsetWidth * (1 - this._scale);
        this.robotWindowPane.style.transform = 'translateX(' + offset + 'px)';
      }
    });
    this._toolbarResizeObserver.observe(document.getElementById('view3d'));
  }

  _mobileOrientationChangeFullscreen() {
      let screenOrientation;
      // Warning: window.orientation is deprecated but screen.orientation is not supported by iOS so we use it in this case.
      if (!screen.orientation || screen.orientation === 'undefined')
        screenOrientation = window.orientation % 180 === 0 ? 'portrait' : 'landscape';
      else
        screenOrientation = screen.orientation.type.includes('portrait') ? 'portrait' : 'landscape';

      if (this._previousScreenOrientation == 'portrait' && screenOrientation === 'landscape' && !isFullscreen())
        requestFullscreen(this._view);
      else if (this._previousScreenOrientation == 'landscape' && screenOrientation === 'portrait' && isFullscreen())
        exitFullscreen();

      this._previousScreenOrientation = screenOrientation;
  }

  removeToolbar() {
    if (typeof this._view.mouseEvents !== 'undefined' && typeof this._view.mouseEvents.hideToolbar) {
      this._view.mouseEvents.hideToolbar = undefined;
      if (typeof this._view.mouseEvents.showToolbar !== 'undefined') {
        this._view.mouseEvents.showToolbar();
        this._view.mouseEvents.showToolbar = undefined;
      }
    }

    document.removeEventListener('fullscreenchange', this.fullscreenRef);
    this.fullscreenRef = undefined;
    document.removeEventListener('keydown', this.keydownRefF);
    this.keydownRefF = undefined;

    window.removeEventListener('click', this._closeInfoOnClick);

    if (this._toolbarResizeObserver)
      this._toolbarResizeObserver.disconnect();

    if (typeof this.toolbar !== 'undefined') {
      this.parentNode.removeChild(this.toolbar);
      this.toolbar = undefined;
    }
  }

  _createToolBarButton(name, tooltipText, click) {
    const button = document.createElement('button');
    button.id = name + '-button';
    button.className = 'toolbar-btn';
    if (typeof click === 'function')
      button.onclick = () => click();

    if (name === 'play' || name === 'run') {
      const buttonElement = document.createElement('div');
      buttonElement.className = 'icon-' + name;
      buttonElement.id = name + '-button-id';
      button.appendChild(buttonElement);
    } else {
      const buttonElement = document.createElement('span');
      buttonElement.className = 'icon icon-' + name;
      buttonElement.id = name + '-button-id';
      button.appendChild(buttonElement);
    }

    const tooltip = document.createElement('span');
    tooltip.id = name + '-tooltip';
    tooltip.className = 'tooltip ' + name + '-tooltip';
    tooltip.innerHTML = tooltipText;
    button.appendChild(tooltip);

    return button;
  }

  _createToolbar(parentNode) {
    this.toolbar = document.createElement('div');
    this.toolbar.id = 'toolbar';
    this.toolbar.className = 'toolbar';
    parentNode.appendChild(this.toolbar);

    this.toolbarLeft = document.createElement('div');
    this.toolbarLeft.className = 'toolbar-left';
    this.toolbarLeft.id = 'toolbar-left';

    this.toolbarRight = document.createElement('div');
    this.toolbarRight.className = 'toolbar-right';
    this.toolbarRight.id = 'toolbar-right';

    this.toolbar.appendChild(this.toolbarLeft);
    this.toolbar.appendChild(this.toolbarRight);
    if (typeof this._view.mouseEvents !== 'undefined' && this._view.mode !== 'mjpeg') {
      this.toolbar.addEventListener('mouseover', () => this.showToolbar());
      this.toolbar.addEventListener('mouseleave', _ => this._onMouseLeave(_));
      this._view.mouseEvents.hideToolbar = () => this.hideToolbar();
      this._view.mouseEvents.showToolbar = () => this.showToolbar();
    }
  }

  _checkLeftTooltips() {
    const toolbarLeft = document.getElementById('toolbar-left');
    for (let child = toolbarLeft.firstChild; child !== null; child = child.nextSibling) {
      const left = child.lastChild.offsetLeft - child.lastChild.offsetWidth / 2;
      if (left < 0) {
        document.getElementById(child.lastChild.id).style.left = '0';
        document.getElementById(child.lastChild.id).style.transform = 'translateX(0)';
      }
    }
  }

  _onMouseLeave(e) {
    if (e.relatedTarget != null && e.relatedTarget.id !== 'canvas')
      this._view.mouseEvents.onMouseLeave();
  }

  hideToolbar(force) {
    if (this.toolbar === 'undefined')
      return;

    if (force) {
      this.toolbar.style.display = 'none';
      this.forceHidden = true;
      return;
    }

    if (typeof this.robotWindowPane !== 'undefined' && this.robotWindowPane.style.visibility === 'visible')
      return;

    if (typeof this.worldSelectionPane !== 'undefined' && this.worldSelectionPane.style.visibility === 'visible')
      return;

    let canHide;

    let isPlaying = true;
    if (document.getElementById('play-button-id') && document.getElementById('play-button-id').className !== 'icon-pause')
      isPlaying = false;

    let settingsPane = true;
    if (typeof this.settingsPane !== 'undefined' && this.settingsPane.style.visibility !== 'hidden')
      settingsPane = false;

    let gtaoPane = true;
    if (typeof this._gtaoPane !== 'undefined' && this._gtaoPane.style.visibility !== 'hidden')
      gtaoPane = false;

    if (this.type === 'animation') {
      const isSelected = this._timeSlider.selected();
      let speedPane = true;
      if (typeof this._speedPane !== 'undefined' && this._speedPane.style.visibility !== 'hidden')
        speedPane = false;

      canHide = !isSelected && isPlaying && settingsPane && gtaoPane && speedPane;
    } else if (this.type === 'streaming') {
      if (document.getElementById('run-button'))
        isPlaying = isPlaying || document.getElementById('run-button-id').className === 'icon-pause';

      canHide = isPlaying && settingsPane && gtaoPane;
    } else if (this.type === 'scene')
      canHide = settingsPane && gtaoPane;

    if (canHide)
      this.toolbar.style.display = 'none';
  }

  showToolbar(force) {
    if (force)
      this.forceHidden = false;
    if (typeof this.toolbar !== 'undefined' && !this.forceHidden)
      this.toolbar.style.display = 'block';
  }

  _createPlayButton() {
    let action;
    if (this.type === 'animation')
      action = (this._view.animation.gui === 'real-time') ? 'pause' : 'play';
    else if (this.type === 'streaming') {
      action = (this._view.currentState === 'real-time') ? 'pause' : 'play';
      if (action === 'pause')
        this.realTime();
    }

    this.playButton = this._createToolBarButton('play', 'Play (k)', () => this._triggerPlayPauseButton());
    this.playButtonElement = this.playButton.childNodes[0];
    this.playTooltip = this.playButton.childNodes[1];

    if (action === 'pause') {
      this.playButtonElement.className = 'icon-pause';
      this.playTooltip.innerHTML = 'Pause (k)';
    }

    this.toolbarLeft.appendChild(this.playButton);
    document.addEventListener('keydown', this.keydownRefK = _ => this._playKeyboardHandler(_));
    if (!(typeof this.parentNode.showPlay === 'undefined' || this.parentNode.showPlay))
      this.playButton.style.display = 'none';

    this.minWidth += 44;
    if (typeof this._view.stream !== 'undefined')
      this._view.stream.onplay = () => this._triggerPlayPauseButton();
  }

  _triggerPlayPauseButton() {
    const animation = this._view.animation;
    let action;
    if (this.type === 'animation' && typeof animation !== 'undefined') {
      if (animation.gui === 'real-time')
        animation.pause();
      else {
        animation.gui = 'real-time';
        animation.start = new Date().getTime() - animation.data.basicTimeStep * animation.step / animation.speed;
        window.requestAnimationFrame(() => animation.updateAnimation());
      }
      action = (animation.gui === 'real-time') ? 'pause' : 'play';
    } else if (this.type === 'streaming') {
      if (this._view.currentState === 'real-time') {
        this.pause();
        action = 'play';
      } else {
        this.realTime();
        action = 'pause';
      }
    }

    if (typeof this.runButton !== 'undefined') {
      this.runTooltip.innerHTML = 'Run';
      this.runButtonElement.className = 'icon-run';
    }

    this.playTooltip.innerHTML = 'P' + action.substring(1) + ' (k)';
    this.playButtonElement.className = 'icon-' + action;
  }

  _playKeyboardHandler(e) {
    if (e.code === 'KeyK')
      this._triggerPlayPauseButton();
  }

  _createIdeButton() {
    // Do not create IDE button if no IDE is available or screen is too small
    if (this._view.mobileDevice && Math.min(screen.height, screen.width) < 700)
      this._view.ide = false;
    if (!this._view.ide)
      return;

    this.ideButton = this._createToolBarButton('ide', 'Source code editor', undefined);
    this.toolbarRight.appendChild(this.ideButton);
    this._createIde();
    if (!(typeof this.parentNode.showIde === 'undefined' || this.parentNode.showIde))
      this.ideButton.style.display = 'none';
    else
      this.minWidth += 44;
  }

  _createIde() {
    const url = this._view.x3dScene.prefix.slice(0, -1);
    this.ideWindow = new FloatingIde(this.parentNode, 'ide', url);

    const ideWidth = 0.6 * this.parentNode.offsetWidth;
    const ideHeight = 0.75 * this.parentNode.offsetHeight;
    const idePositionX = (this.parentNode.offsetWidth - ideWidth) / 2;
    const idePositionY = (this.parentNode.offsetHeight - ideHeight) / 2;;

    this.ideWindow.floatingWindow.addEventListener('mouseover', () => this.showToolbar());
    this.ideWindow.headerQuit.addEventListener('mouseup', _ => this._changeFloatingWindowVisibility(this.ideWindow.getId()));

    this.ideWindow.setSize(ideWidth, ideHeight);
    this.ideWindow.setPosition(idePositionX, idePositionY);
    this.ideButton.onclick = () => this._changeFloatingWindowVisibility(this.ideWindow.getId());

    this._checkWindowBoundaries();
  }

  _createRobotWindowButton() {
    if (typeof WbWorld.instance !== 'undefined' && WbWorld.instance.readyForUpdates) {
      if (WbWorld.instance.robots.length > 0) {
        this.robotWindowButton = this._createToolBarButton('robot-window', 'Robot windows (w)');
        this.toolbarRight.appendChild(this.robotWindowButton);
        this.robotWindowButton.addEventListener('mouseup', this.mouseupRefWFirst = _ => this._showAllRobotWindows(),
          {once: true});
        document.addEventListener('keydown', this.keydownRefWFirst = _ => this._robotWindowPaneKeyboardHandler(_, true),
          {once: true});
        this.keydownRefW = undefined;
        window.addEventListener('click', _ => this._closeRobotWindowPaneOnClick(_));
        if (!(typeof this.parentNode.showRobotWindow === 'undefined' || this.parentNode.showRobotWindow))
          this.robotWindowButton.style.display = 'none';
        else
          this.minWidth += 44;
      }
    }
  }

  _createRobotWindowPane() {
    this.robotWindowPane = document.createElement('div');
    this.robotWindowPane.id = 'robot-window-pane';
    this.robotWindowPane.className = 'vertical-list-pane';
    this.robotWindowPane.style.visibility = 'hidden';
    this.parentNode.appendChild(this.robotWindowPane);

    const title = document.createElement('div');
    title.className = 'vertical-list-pane-title';
    title.innerHTML = 'Robot Windows';
    this.robotWindowPane.appendChild(title);

    this.robotWindowList = document.createElement('ul');
    this.robotWindowList.id = 'robot-window-list';
    this.robotWindowPane.appendChild(this.robotWindowList);

    const offset = this._scale === 1 ? 0 : screen.width * (1 - this._scale);
    this.robotWindowPane.style.transform = 'translateX(' + offset + 'px)';
  }

  _closeRobotWindowPaneOnClick(event) {
    if (typeof this.robotWindowPane !== 'undefined') {
      if (event.srcElement.id !== 'robot-window-button' && this.robotWindowPane.style.visibility === 'visible') {
        if (!(event.srcElement.id.startsWith('close-') || event.srcElement.id.startsWith('enable-robot-window')))
          this._changeRobotWindowPaneVisibility(event);
      }
    }
  }

  _addRobotWindowToPane(name, mainWindow) {
    const robotWindowLi = document.createElement('li');
    robotWindowLi.className = 'vertical-list-pane-li';
    robotWindowLi.id = 'enable-robot-window-' + name;
    if (mainWindow)
      this.robotWindowList.prepend(robotWindowLi);
    else
      this.robotWindowList.appendChild(robotWindowLi);

    const button = document.createElement('label');
    button.className = 'vertical-list-pane-switch';
    robotWindowLi.appendChild(button);

    let label = document.createElement('input');
    label.id = name + '-button';
    label.type = 'checkbox';
    label.checked = false;
    label.style.display = 'none';
    button.appendChild(label);

    label = document.createElement('span');
    label.className = 'visibility-dot';
    button.appendChild(label);

    label = document.createElement('div');
    label.className = 'vertical-list-pane-spacer';
    robotWindowLi.appendChild(label);

    label = document.createElement('span');
    label.id = 'enable-robot-window-text-' + name;
    label.className = 'robot-window-span';
    label.innerHTML = mainWindow ? 'World Info' : name;
    robotWindowLi.appendChild(label);

    robotWindowLi.onclick = _ => {
      this._changeFloatingWindowVisibility(name);
    };
  }

  _createRobotWindows() {
    const robotWindowUrl = this._view.x3dScene.prefix.slice(0, -1);

    this.robotWindows = [];
    if (typeof WbWorld.instance !== 'undefined' && WbWorld.instance.readyForUpdates) {
      WbWorld.instance.robots.forEach((robot) => {
        if (robot.window !== '<none>') {
          let mainWindow = (WbWorld.instance.window !== '<none>' && robot.window === WbWorld.instance.window) ? true : false;
          let robotWindow = new FloatingRobotWindow(this.parentNode, robot.name, robotWindowUrl, robot.window, mainWindow)
          if (mainWindow)
            this.robotWindows.unshift(robotWindow);
          else
            this.robotWindows.push(robotWindow);
          this._addRobotWindowToPane(robot.name, mainWindow);
        }
      });
      const buttonDisplay = (this.robotWindows.length === 0) ? 'none' : 'auto';
      document.getElementById('robot-window-button').style.display = buttonDisplay;
    }
    this.robotWindowPane.style.right = '150px';

    const viewWidth = this.parentNode.offsetWidth;
    const viewHeight = this.parentNode.offsetHeight;
    const robotWindowWidth = 250;
    const robotWindowHeight = 200;
    const margin = 20;
    let numCol = 0;
    let numRow = 0;
    let layer = 0;

    this.robotWindows.forEach((rw) => {
      rw.floatingWindow.addEventListener('mouseover', () => this.showToolbar());
      rw.headerQuit.addEventListener('mouseup', _ => this._changeFloatingWindowVisibility(rw.getId()));


      if (rw.isMainWindow) {
        rw.setSize(2 * robotWindowWidth, 2 * robotWindowHeight);
        rw.setPosition(margin, margin);
      } else {
        if (margin + (numRow + 1) * (margin + robotWindowHeight) > viewHeight) {
          numCol++;
          if (margin + (numCol + 1) * (margin + robotWindowWidth) > viewWidth) {
            numCol = 0;
            layer++;
          }
          numRow = 0;
        }
  
        rw.setSize(robotWindowWidth, robotWindowHeight);
        rw.setPosition(viewWidth - robotWindowWidth - margin - numCol * (margin + robotWindowWidth) + layer * margin / 3,
          margin + numRow * (margin + robotWindowHeight) + layer * margin / 3);
        numRow++;
      }
    });

    this._checkWindowBoundaries();
    this._initializeWindowLayerChanges();
  }

  _initializeWindowLayerChanges() {
    window.addEventListener('blur', _ => {
      const newElement = document.activeElement;
      if (newElement && newElement.parentNode && newElement.parentNode.parentNode &&
        newElement.parentNode.parentNode.classList.contains('floating-window')) {
        document.querySelectorAll('.floating-window').forEach((fw) => { fw.style.zIndex = '1'; });
        document.getElementById(newElement.parentNode.parentNode.id).style.zIndex = '2';
      }
    })
  }

  _refreshRobotWindowContent() {
    this.robotWindows.forEach((rw) => {
      if (typeof document.getElementById(rw.name + '-robot-window').src !== 'undefined')
        document.getElementById(rw.name + '-robot-window').src = document.getElementById(rw.name + '-robot-window').src;
    });
  }

  loadRobotWindows() {
    this.removeRobotWindows();
    this._createRobotWindowPane();
    this._createRobotWindows();
  }

  removeRobotWindows() {
    if (typeof this.robotWindowPane !== 'undefined')
      this.robotWindowPane.remove();
    if (typeof this.robotWindows !== 'undefined')
      document.querySelectorAll('.floating-window').forEach(fw => fw.remove());
  }

  _changeRobotWindowPaneVisibility(event) {
    if (this.robotWindowPane.style.visibility === 'hidden') {
      this.robotWindowPane.style.visibility = 'visible';
      for (let i of document.getElementsByClassName('tooltip'))
        i.style.visibility = 'hidden';
    } else {
      this.robotWindowPane.style.visibility = 'hidden';
      if (event !== 'undefined' && !(event.srcElement.id.startsWith('settings') ||
        event.srcElement.id.startsWith('world-selection'))) {
        for (let i of document.getElementsByClassName('tooltip'))
          i.style.visibility = '';
      }
    }
  }

  _robotWindowPaneKeyboardHandler(e, isFirst) {
    if (e.code === 'KeyW') {
      if (isFirst)
        this._showAllRobotWindows();
      else
        this._changeRobotWindowPaneVisibility(e);
    }
  }

  _changeFloatingWindowVisibility(name) {
    const floatingWindow = document.getElementById(name);
    const floatingWindowButton = document.getElementById(name);
    if (floatingWindow && floatingWindowButton) {
      if (document.getElementById(name).style.visibility === 'hidden') {
        document.getElementById(name + '-button').checked = true;
        document.getElementById(name).style.visibility = 'visible';
      } else {
        document.getElementById(name + '-button').checked = false;
        document.getElementById(name).style.visibility = 'hidden';
      }
    }
  }

  _showAllRobotWindows() {
    document.removeEventListener('keydown', this.keydownRefWFirst);
    this.keydownRefWFirst = undefined;
    this.robotWindowButton.removeEventListener('mouseup', this.mouseupRefWFirst);
    this.mouseupRefWFirst = undefined;
    this._changeRobotWindowPaneVisibility();
    if (this.robotWindows)
      this.robotWindows.forEach((rw) => this._changeFloatingWindowVisibility(rw.getId()));
    this.robotWindowButton.addEventListener('mouseup', _ => this._changeRobotWindowPaneVisibility(_));
    document.addEventListener('keydown', this.keydownRefW = _ => this._robotWindowPaneKeyboardHandler(_, false));
  }

  _checkWindowBoundaries() {
    const resizeObserver = new ResizeObserver(() => {
      const floatingWindows = document.querySelectorAll('.floating-window');
      floatingWindows.forEach((fw) => {
        const maxLeft = this.parentNode.offsetWidth - parseInt(window.getComputedStyle(fw).minWidth);
        const maxTop = this.parentNode.offsetHeight - parseInt(window.getComputedStyle(fw).minHeight);
        fw.style.left = (fw.offsetLeft > maxLeft ? maxLeft : fw.offsetLeft) + 'px';
        fw.style.top = (fw.offsetTop > maxTop ? maxTop : fw.offsetTop) + 'px';

        const maxWidth = this.parentNode.offsetWidth - fw.offsetLeft;
        const maxHeight = this.parentNode.offsetHeight - fw.offsetTop;
        fw.style.width = (fw.offsetWidth > maxWidth ? maxWidth : fw.offsetWidth) + 'px';
        fw.style.height = (fw.offsetHeight > maxHeight ? maxHeight : fw.offsetHeight) + 'px';
      });
    });
    resizeObserver.observe(document.getElementById('view3d'));
  }

  _createInfoButton() {
    this.infoButton = this._createToolBarButton('info', 'Simulation information', () => this._displayInformationWindow());
    this.toolbarRight.appendChild(this.infoButton);
    this._createInformation();
    window.addEventListener('click', _ => this._closeInfoOnClick(_));
    if (!(typeof this.parentNode.showInfo === 'undefined' || this.parentNode.showInfo))
      this.infoButton.style.display = 'none';
    else
      this.minWidth += 44;
  }

  _createInformation() {
    this.informationPanel = new InformationPanel(this.parentNode);

    if (typeof WbWorld.instance !== 'undefined' && WbWorld.instance.readyForUpdates) {
      this.informationPanel.setTitle(WbWorld.instance.title);
      this.informationPanel.setDescription(WbWorld.instance.description);
    }
  }

  _displayInformationWindow() {
    if (typeof this.informationPanel !== 'undefined') {
      if (this.informationPanel.informationPanel.style.display === 'block')
        this.informationPanel.informationPanel.style.display = 'none';
      else {
        this.informationPanel.informationPanel.style.display = 'block';
        const infoScaleWidth = this.informationPanel.informationPanel.offsetWidth > 0.8 * this.parentNode.offsetWidth
          ? 0.8 * this.parentNode.offsetWidth / this.informationPanel.informationPanel.offsetWidth : 1.0;
        const infoScaleHeight = this.informationPanel.informationPanel.offsetHeight > 0.8 * this.parentNode.offsetHeight
          ? 0.8 * this.parentNode.offsetHeight / this.informationPanel.informationPanel.offsetHeight : 1.0;
        this.informationPanel.informationPanel.style.transform = 'scale(' + Math.min(infoScaleHeight, infoScaleWidth) + ')';
      }
    }
  }

  _closeInfoOnClick(event) {
    if (typeof this.informationPanel !== 'undefined' && !this.informationPanel.informationPanel.contains(event.target) &&
      !this.infoButton.contains(event.target))
      this.informationPanel.informationPanel.style.display = 'none';
  }

  _createSettings() {
    this.toolbarRight.appendChild(this._createToolBarButton('settings', 'Settings'));
    this.minWidth += 44;
    this._createSettingsPane();
  }

  _createSettingsPane() {
    this.settingsPane = document.createElement('div');
    this.settingsPane.className = 'settings-pane';
    this.settingsPane.id = 'settings-pane';
    this.settingsPane.style.visibility = 'hidden';
    document.addEventListener('mouseup', this.settingsRef = _ => this._changeSettingsPaneVisibility(_));
    this.parentNode.appendChild(this.settingsPane);
    this.settingsPane.addEventListener('mouseover', () => this.showToolbar());

    this.settingsList = document.createElement('ul');
    this.settingsList.id = 'settings-list';
    this.settingsPane.appendChild(this.settingsList);

    this._createResetViewpoint();
    this._createChangeShadows();
    this._createChangeGtao();
    if (this.type === 'animation')
      this._createChangeSpeed();
  }

  _changeSettingsPaneVisibility(event) {
    // Avoid to close the settings when modifying the shadows or the other options
    if (event.srcElement.id === 'enable-shadows' || event.srcElement.id === 'gtao-settings')
      return;

    if (typeof this.settingsPane === 'undefined' || typeof this._gtaoPane === 'undefined')
      return;

    let speedPanelHidden = true;
    if (this.type === 'animation') {
      if (event.srcElement.id === 'playback-li' || typeof this._speedPane === 'undefined')
        return;
      else if (this._speedPane.style.visibility === 'visible')
        speedPanelHidden = false;
    }

    if (event.target.id === 'settings-button' && this.settingsPane.style.visibility === 'hidden' &&
      this._gtaoPane.style.visibility === 'hidden' && speedPanelHidden) {
      this.settingsPane.style.visibility = 'visible';
      const tooltips = document.getElementsByClassName('tooltip');
      for (let i of tooltips)
        i.style.visibility = 'hidden';
    } else if (this.settingsPane.style.visibility === 'visible' || this._gtaoPane.style.visibility === 'visible' ||
      !speedPanelHidden) {
      this.settingsPane.style.visibility = 'hidden';
      if (this._gtaoPane.style.visibility === 'hidden' && speedPanelHidden) {
        const tooltips = document.getElementsByClassName('tooltip');
        if ((event !== 'undefined') && !(event.srcElement.id.startsWith('robot-window') ||
          event.srcElement.id.startsWith('world-selection'))) {
          for (let i of tooltips)
            i.style.visibility = '';
        }
      }
    }

    this._gtaoPane.style.visibility = 'hidden';
    if (this.type === 'animation')
      this._speedPane.style.visibility = 'hidden';
  }

  _createResetViewpoint() {
    const resetViewpoint = document.createElement('li');
    resetViewpoint.onclick = () => this._resetViewpoint();
    resetViewpoint.id = 'reset-viewpoint';
    this.settingsList.appendChild(resetViewpoint);

    let label = document.createElement('span');
    label.className = 'setting-span';
    label.innerHTML = 'Reset viewpoint';
    resetViewpoint.appendChild(label);

    label = document.createElement('div');
    label.className = 'spacer';
    resetViewpoint.appendChild(label);
  }

  _createChangeShadows() {
    const shadowLi = document.createElement('li');
    shadowLi.id = 'enable-shadows';
    this.settingsList.appendChild(shadowLi);

    let label = document.createElement('span');
    label.className = 'setting-span';
    label.innerHTML = 'Shadows';
    shadowLi.appendChild(label);

    label = document.createElement('div');
    label.className = 'spacer';
    shadowLi.appendChild(label);

    const button = document.createElement('label');
    button.className = 'switch';
    shadowLi.appendChild(button);

    label = document.createElement('input');
    label.type = 'checkbox';
    label.checked = true;
    button.appendChild(label);

    label = document.createElement('span');
    label.className = 'slider round';
    button.appendChild(label);

    shadowLi.onclick = _ => {
      button.click();
      changeShadows();
      this._view.x3dScene.render();
    };
  }

  _createChangeGtao() {
    const gtaoLi = document.createElement('li');
    gtaoLi.id = 'gtao-settings';
    this.settingsList.appendChild(gtaoLi);
    gtaoLi.onclick = () => this._openGtaoPane();

    let label = document.createElement('span');
    label.className = 'setting-span';
    label.innerHTML = 'Ambient Occlusion';
    gtaoLi.appendChild(label);

    label = document.createElement('div');
    label.className = 'spacer';
    gtaoLi.appendChild(label);

    label = document.createElement('span');
    label.className = 'setting-text';
    label.innerHTML = this.gtaoLevelToText(GtaoLevel);
    label.id = 'gtao-display';
    gtaoLi.appendChild(label);

    label = document.createElement('div');
    label.className = 'arrow-right';
    gtaoLi.appendChild(label);

    this._createGtaoPane();
  }

  _createGtaoPane() {
    this._gtaoPane = document.createElement('div');
    this._gtaoPane.className = 'settings-pane';
    this._gtaoPane.id = 'gtao-pane';
    this._gtaoPane.style.visibility = 'hidden';
    this.parentNode.appendChild(this._gtaoPane);

    this._gtaoPane.addEventListener('mouseover', () => this.showToolbar());

    const gtaoList = document.createElement('ul');
    this._gtaoPane.appendChild(gtaoList);

    let gtaoLevelLi = document.createElement('li');
    gtaoLevelLi.className = 'first-li';

    let label = document.createElement('div');
    label.className = 'arrow-left';
    gtaoLevelLi.appendChild(label);

    label = document.createElement('span');
    label.innerHTML = 'Ambient Occlusion Level';
    label.className = 'setting-span';
    gtaoLevelLi.appendChild(label);

    label = document.createElement('div');
    label.className = 'spacer';
    gtaoLevelLi.appendChild(label);
    gtaoLevelLi.onclick = () => this._closeGtaoPane();
    gtaoList.appendChild(gtaoLevelLi);

    for (let i of ['low', 'normal', 'high', 'ultra']) {
      gtaoLevelLi = document.createElement('li');
      gtaoLevelLi.id = i;
      label = document.createElement('span');
      if (this.gtaoLevelToText(GtaoLevel) === i)
        label.innerHTML = '&check;';
      label.id = 'c-' + i;
      label.className = 'check-gtao';
      gtaoLevelLi.appendChild(label);
      label = document.createElement('span');
      label.innerHTML = i;
      label.className = 'setting-span';
      gtaoLevelLi.appendChild(label);
      label = document.createElement('div');
      label.className = 'spacer';
      gtaoLevelLi.appendChild(label);
      gtaoLevelLi.onclick = _ => this.changeGtao(_);
      gtaoList.appendChild(gtaoLevelLi);
    }
  }

  changeGtao(event) {
    changeGtaoLevel(this._textToGtaoLevel(event.srcElement.id));
    this._gtaoPane.style.visibility = 'hidden';
    const gtaoLabel = document.getElementById('gtao-display');
    if (gtaoLabel)
      gtaoLabel.innerHTML = event.srcElement.id;
    this.settingsPane.style.visibility = 'visible';
    for (let i of document.getElementsByClassName('check-gtao')) {
      if (i.id === 'c-' + event.srcElement.id)
        i.innerHTML = '&check;';
      else
        i.innerHTML = '';
    }
    const animation = this._view.animation;
    if (animation)
      animation.start = new Date().getTime() - animation.data.basicTimeStep * animation.step / animation.speed;
    this._view.x3dScene.render();
  }

  _openGtaoPane() {
    this.settingsPane.style.visibility = 'hidden';
    this._gtaoPane.style.visibility = 'visible';
  }

  _closeGtaoPane() {
    this.settingsPane.style.visibility = 'visible';
    this._gtaoPane.style.visibility = 'hidden';
  }

  gtaoLevelToText(number) {
    const pairs = {
      1: 'low',
      2: 'medium',
      3: 'high',
      4: 'ultra'
    };
    return (number in pairs) ? pairs[number] : '';
  }

  _textToGtaoLevel(text) {
    const pairs = {
      'low': 1,
      'medium': 2,
      'high': 3,
      'ultra': 4
    };
    return (text in pairs) ? pairs[text] : 4;
  }

  _resetViewpoint() {
    WbWorld.instance.viewpoint.resetViewpoint();
    this._view.x3dScene.render(); // render once to visually reset immediatly the viewpoint.
  }

  _createFullscreenButtons() {
    this._fullscreenButton = this._createToolBarButton('fullscreen', 'Full screen (f)', () => requestFullscreen(this._view));
    this.toolbarRight.appendChild(this._fullscreenButton);

    this._exitFullscreenButton = this._createToolBarButton('windowed', 'Exit full screen (f)', () => exitFullscreen());
    this.toolbarRight.appendChild(this._exitFullscreenButton);
    this._exitFullscreenButton.style.display = 'none';

    this.minWidth += 44;

    document.addEventListener('fullscreenchange', this.fullscreenRef = () =>
      onFullscreenChange(this._fullscreenButton, this._exitFullscreenButton));
    document.addEventListener('keydown', this.keydownRefF = _ => this._fullscrenKeyboardHandler(_));
  }

  _parseMillisecondsIntoReadableTime(milliseconds) {
    const hours = (milliseconds + 0.9) / (1000 * 60 * 60);
    const absoluteHours = Math.floor(hours);
    const h = absoluteHours > 9 ? absoluteHours : '0' + absoluteHours;
    const minutes = (hours - absoluteHours) * 60;
    const absoluteMinutes = Math.floor(minutes);
    const m = absoluteMinutes > 9 ? absoluteMinutes : '0' + absoluteMinutes;
    const seconds = (minutes - absoluteMinutes) * 60;
    const absoluteSeconds = Math.floor(seconds);
    const s = absoluteSeconds > 9 ? absoluteSeconds : '0' + absoluteSeconds;
    let ms = Math.floor((seconds - absoluteSeconds) * 1000);
    if (ms < 10)
      ms = '00' + ms;
    else if (ms < 100)
      ms = '0' + ms;
    return h + ':' + m + ':' + s + ':<small>' + ms + '<small>';
  };

  _fullscrenKeyboardHandler(e) {
    if (e.code === 'KeyF')
      this._fullscreenButton.style.display === 'none' ? exitFullscreen() : requestFullscreen(this._view);
  }

  _removeFullscreenAnimation(button) {
    button.style.animation = 'none';
  }

  // Animations functions

  _createSlider() {
    if (!Animation.sliderDefined) {
      window.customElements.define('animation-slider', AnimationSlider);
      Animation.sliderDefined = true;
    }
    this._timeSlider = document.createElement('animation-slider');
    this._timeSlider.id = 'timeSlider';
    document.addEventListener('sliderchange', this.sliderchangeRef = _ => this._updateSlider(_));
    this.toolbar.appendChild(this._timeSlider);
    this._timeSlider.shadowRoot.getElementById('range').addEventListener('mousemove', this.updateFloatingTimeRef = _ =>
      this._updateFloatingTimePosition(_));
    this._timeSlider.shadowRoot.getElementById('range').addEventListener('mouseleave', this.hideFloatingTimeRef = _ =>
      this._hideFloatingTimePosition(_));
  }

  _updateSlider(event) {
    const animation = this._view.animation;
    if (event.mouseup && animation) {
      if (animation.previousState === 'real-time' && animation.gui === 'pause') {
        animation.previousState = undefined;
        this._triggerPlayPauseButton();
      }
      return;
    }

    const value = event.detail;

    if (animation.gui === 'real-time') {
      animation.previousState = 'real-time';
      this._triggerPlayPauseButton();
    }

    const clampedValued = Math.min(value, 99); // set maximum value to get valid step index
    const requestedStep = Math.floor(animation.data.frames.length * clampedValued / 100);
    animation.start = (new Date().getTime()) - Math.floor(animation.data.basicTimeStep * animation.step);
    animation.updateAnimationState(requestedStep);

    this._timeSlider.setTime(this._formatTime(animation.data.frames[requestedStep].time));
  }

  _updateFloatingTimePosition(e) {
    this._timeSlider.shadowRoot.getElementById('floating-time').style.visibility = 'visible';

    const bounds = this._timeSlider.shadowRoot.getElementById('range').getBoundingClientRect();
    let x = (e.clientX - bounds.left) / (bounds.right - bounds.left) * 100;
    if (x > 100)
      x = 100;
    else if (x < 0)
      x = 0;

    const clampedValued = Math.min(x, 99); // set maximum value to get valid step index
    const requestedStep = Math.floor(this._view.animation.data.frames.length * clampedValued / 100);
    this._timeSlider.setTime(this._formatTime(this._view.animation.data.frames[requestedStep].time));

    this._timeSlider.setFloatingTimePosition(e.clientX);
  }

  _hideFloatingTimePosition() {
    this._timeSlider.shadowRoot.getElementById('floating-time').style.visibility = '';
  }

  _createAnimationTimeIndicator() {
    this._currentTime = document.createElement('span');
    this._currentTime.className = 'current-time';
    this._currentTime.id = 'currentTime';
    this._currentTime.innerHTML = this._formatTime(this._view.animation.data.frames[0].time);
    this.toolbarLeft.appendChild(this._currentTime);

    const timeDivider = document.createElement('span');
    timeDivider.innerHTML = '/';
    timeDivider.className = 'time-divider';
    this.toolbarLeft.appendChild(timeDivider);

    const totalTime = document.createElement('span');
    totalTime.className = 'total-time';
    const time = this._formatTime(this._view.animation.data.frames[this._view.animation.data.frames.length - 1].time);
    totalTime.innerHTML = time;
    this.toolbarLeft.appendChild(totalTime);

    let offset;
    switch (time.length) {
      case 20:
        offset = 19;
        break;
      case 22:
        offset = 25;
        break;
      case 23:
        offset = 30;
        break;
      case 25:
        offset = 36;
        break;
      default:
        offset = 0;
    }

    if (typeof this._timeSlider !== 'undefined')
      this._timeSlider.setOffset(offset);

    this.minWidth += 133;
  }

  _formatTime(time) {
    if (typeof this._unusedPrefix === 'undefined') {
      const maxTime = this._view.animation.data.frames[this._view.animation.data.frames.length - 1].time;
      if (maxTime < 60000)
        this._unusedPrefix = 6;
      else if (maxTime < 600000)
        this._unusedPrefix = 4;
      else if (maxTime < 3600000)
        this._unusedPrefix = 3;
      else if (maxTime < 36000000)
        this._unusedPrefix = 1;
    }

    return this._parseMillisecondsIntoReadableTime(time).substring(this._unusedPrefix);
  }

  _createChangeSpeed() {
    const playbackLi = document.createElement('li');
    playbackLi.id = 'playback-li';
    this.settingsList.appendChild(playbackLi);
    playbackLi.onclick = () => this._openSpeedPane();

    let label = document.createElement('span');
    label.innerHTML = 'Playback speed';
    label.className = 'setting-span';
    playbackLi.appendChild(label);

    label = document.createElement('div');
    label.className = 'spacer';
    playbackLi.appendChild(label);

    label = document.createElement('span');
    label.className = 'setting-text';
    label.innerHTML = 'Normal';
    label.id = 'speed-display';
    playbackLi.appendChild(label);

    label = document.createElement('div');
    label.className = 'arrow-right';
    playbackLi.appendChild(label);

    this._createSpeedPane();
  }

  _createSpeedPane() {
    this._speedPane = document.createElement('div');
    this._speedPane.className = 'settings-pane';
    this._speedPane.id = 'speed-pane';
    this._speedPane.style.visibility = 'hidden';

    const speedList = document.createElement('ul');
    this._speedPane.appendChild(speedList);
    this.parentNode.appendChild(this._speedPane);

    let playbackLi = document.createElement('li');
    playbackLi.className = 'first-li';

    let label = document.createElement('div');
    label.className = 'arrow-left';
    playbackLi.appendChild(label);

    label = document.createElement('span');
    label.innerHTML = 'Playback speed';
    label.className = 'setting-span';
    playbackLi.appendChild(label);

    label = document.createElement('div');
    label.className = 'spacer';
    playbackLi.appendChild(label);
    playbackLi.onclick = () => this._closeSpeedPane();
    speedList.appendChild(playbackLi);

    for (let i of ['0.25', '0.5', '0.75', '1', '1.25', '1.5', '1.75', '2']) {
      playbackLi = document.createElement('li');
      playbackLi.id = i;
      label = document.createElement('span');
      if (i === '1')
        label.innerHTML = '&check;';
      label.id = 'c' + i;
      label.className = 'check-speed';
      playbackLi.appendChild(label);
      label = document.createElement('span');
      if (i === '1')
        label.innerHTML = 'Normal';
      else
        label.innerHTML = i;
      label.className = 'setting-span';
      playbackLi.appendChild(label);
      label = document.createElement('div');
      label.className = 'spacer';
      playbackLi.appendChild(label);
      playbackLi.onclick = _ => this._changeSpeed(_);
      speedList.appendChild(playbackLi);
    }
  }

  _changeSpeed(event) {
    const animation = this._view.animation;
    if (animation) {
      animation.speed = event.srcElement.id;
      this._speedPane.style.visibility = 'hidden';
      const speedDisplay = document.getElementById('speed-display');
      if (speedDisplay)
        speedDisplay.innerHTML = animation.speed === '1' ? 'Normal' : animation.speed;
      this.settingsPane.style.visibility = 'visible';
      for (let i of document.getElementsByClassName('check-speed')) {
        if (i.id === 'c' + animation.speed)
          i.innerHTML = '&check;';
        else
          i.innerHTML = '';
      }
      animation.start = new Date().getTime() - animation.data.basicTimeStep * animation.step / animation.speed;
    }
  }

  _openSpeedPane() {
    this.settingsPane.style.visibility = 'hidden';
    this._speedPane.style.visibility = 'visible';
  }

  _closeSpeedPane() {
    this.settingsPane.style.visibility = 'visible';
    this._speedPane.style.visibility = 'hidden';
  }

  removeAnimationToolbar() {
    document.removeEventListener('sliderchange', this.sliderchangeRef);
    this.sliderchangeRef = undefined;

    if (typeof this._timeSlider !== 'undefined') {
      this._timeSlider.shadowRoot.getElementById('range').removeEventListener('mousemove', this.updateFloatingTimeRef);
      this.updateFloatingTimeRef = undefined;
      this._timeSlider.shadowRoot.getElementById('range').removeEventListener('mouseleave', this.hideFloatingTimeRef);
      this.hideFloatingTimeRef = undefined;
      this._timeSlider.removeEventListeners();
      this._timeSlider = undefined;
    }

    if (typeof this._speedPane !== 'undefined') {
      this.parentNode.removeChild(this._speedPane);
      this._speedPane = undefined;
    }

    this.removeStreamingToolbar();
  }

  // Scene functions

  _createRestoreViewpointButton() {
    this.toolbarRight.appendChild(this._createToolBarButton('reset-scene', 'Reset the Scene', () => this._resetViewpoint()));
    this.minWidth += 44;
  }

  // Streaming functions

  _createQuitButton() {
    const quitButton = this._createToolBarButton('quit', 'Quit the simulation', () => this._view.quitSimulation());
    if (!(typeof this.parentNode.showQuit === 'undefined' || this.parentNode.showQuit))
      quitButton.style.display = 'none';
    else
      this.minWidth += 44;

    this.toolbarLeft.appendChild(quitButton);
  }

  _createReloadButton() {
    const reloadButton = this._createToolBarButton('reload', 'Reload the simulation', () => { this.reset(true); });
    if (!this.parentNode.showReload)
      reloadButton.style.display = 'none';
    else
      this.minWidth += 44;
    this.toolbarLeft.appendChild(reloadButton);
  }

  reset(reload = false) {
    if (this._view.broadcast)
      return;
    if (reload)
      this._view.progress.setProgressBar('block', 'Reloading simulation...');
    else
      this._view.progress.setProgressBar('block', 'Restarting simulation...');

    if (typeof this.pauseButton !== 'undefined' && this.playButtonElement.className === 'icon-pause')
      this._view.currentState = 'real-time';
    else if (typeof this.runButton !== 'undefined' && this.runButtonElement.className === 'icon-pause')
      this._view.currentState = 'run';

    const state = this._view.currentState;
    this.pause();
    this._view.currentState = state;

    this.hideToolbar(true);
    let previousOnReady = this._view.onready;
    this._view.onready = () => {
      if (typeof previousOnReady === 'function')
        previousOnReady();

      switch (this._view.currentState) {
        case 'real-time':
          this.realTime();
          break;
        case 'fast':
        case 'run':
          this.run();
          break;
      }
      this.showToolbar(true);
    };
    if (reload)
      this._view.stream.socket.send('reload');
    else
      this._view.stream.socket.send('reset');

    if (this.robotWindows)
      this._refreshRobotWindowContent();
  }

  pause() {
    if (this._view.broadcast)
      return;
    this._view.stream.socket.send('pause');
    this._view.currentState = 'pause';
  }

  realTime(force) {
    if (this._view.broadcast)
      return;
    this._view.stream.socket.send('real-time:' + this._view.timeout);
    this._view.currentState = 'real-time';
  }

  _createStreamingTimeIndicator() {
    const clock = document.createElement('span');
    clock.className = 'webots-streaming-time';

    clock.id = 'webots-clock';
    clock.title = 'Current simulation time';
    clock.innerHTML = this._parseMillisecondsIntoReadableTime(0);
    this.toolbarLeft.appendChild(clock);
    this.minWidth += 125;
  }

  _createResetButton() {
    let resetButton = this._createToolBarButton('reset', 'Reset the simulation', () => this.reset(false));
    this.toolbarLeft.appendChild(resetButton);
    if (!(typeof this.parentNode.showReset === 'undefined' || this.parentNode.showReset))
      resetButton.style.display = 'none';
    else
      this.minWidth += 44;
  }

  _createStepButton() {
    let stepButton = this._createToolBarButton('step', 'Perform one simulation step', () => this.step());
    this.toolbarLeft.appendChild(stepButton);
    if (!(typeof this.parentNode.showStep === 'undefined' || this.parentNode.showStep))
      stepButton.style.display = 'none';
    else
      this.minWidth += 32;
  }

  step() {
    if (this._view.broadcast)
      return;

    if (typeof this.playButton !== 'undefined') {
      this.playTooltip.innerHTML = 'Play (k)';
      this.playButtonElement.className = 'icon-play';
    }

    if (typeof this.runButton !== 'undefined') {
      this.runTooltip.innerHTML = 'Run';
      this.runButtonElement.className = 'icon-run';
    }

    this.pause();

    this._view.stream.socket.send('step');
  }

  _createRunButton() {
    this.runButton = this._createToolBarButton('run', 'Run', () => this._triggerRunPauseButton());
    this.runButtonElement = this.runButton.childNodes[0];
    this.runTooltip = this.runButton.childNodes[1];
    if (!this.parentNode.showRun)
      this.runButton.style.display = 'none';
    else
      this.minWidth += 44;
    if (this._view.currentState === 'run' || this._view.currentState === 'fast') {
      this.runTooltip.innerHTML = 'Pause';
      this.runButtonElement.className = 'icon-pause';
      this.run();
    }
    this.toolbarLeft.appendChild(this.runButton);

    if (typeof this._view.stream !== 'undefined')
      this._view.stream.onrun = () => this._triggerRunPauseButton();
  }

  _triggerRunPauseButton() {
    let action;
    if (this.type === 'streaming') {
      if (this._view.currentState === 'run' || this._view.currentState === 'fast') {
        this.pause();
        action = 'run';
      } else {
        this.run();
        action = 'pause';
      }
    }
    if (typeof this.playButton !== 'undefined') {
      this.playTooltip.innerHTML = 'Play (k)';
      this.playButtonElement.className = 'icon-play';
    }

    this.runTooltip.innerHTML = action.charAt(0).toUpperCase() + action.slice(1);
    this.runButtonElement.className = 'icon-' + action;
  }

  run() {
    if (this._view.broadcast)
      return;
    this._view.stream.socket.send('fast:' + this._view.timeout);
    this._view.currentState = 'fast';
  }

  _createWorldSelectionButton() {
    this.worldSelectionButton = this._createToolBarButton('world-selection', 'Select world');
    this.toolbarLeft.appendChild(this.worldSelectionButton);
    this.createWorldSelectionPane();
    this.worldSelectionButton.addEventListener('mouseup', _ => this._changeWorldSelectionPaneVisibility(_));
    window.addEventListener('click', _ => this._closeWorldSelectionPaneOnClick(_));

    if (!(typeof this.parentNode.showWorldSelection === 'undefined' || this.parentNode.showWorldSelection) ||
      this._view.worlds.length <= 1)
      this.worldSelectionButton.style.display = 'none';
    else
      this.minWidth += 44;
  }

  createWorldSelectionPane() {
    this.worldSelectionPane = document.createElement('div');
    this.worldSelectionPane.id = 'world-selection-pane';
    this.worldSelectionPane.className = 'vertical-list-pane';
    this.parentNode.appendChild(this.worldSelectionPane);
    this.worldSelectionPane.style.visibility = 'hidden';
    this.worldSelectionPane.style.left = '1%';
    this.worldList = document.createElement('ul');
    this.worldList.id = 'world-list';
    this.worldSelectionPane.appendChild(this.worldList);
    const title = document.createElement('li');
    title.className = 'vertical-list-pane-title';
    title.innerHTML = 'Worlds';
    this.worldList.appendChild(title);

    this.worldSelectionPane.addEventListener('mouseover', () => this.showToolbar());

    const worlds = this._view.worlds;
    for (let i in worlds) {
      this._addWorldToPane(worlds[i]);
      if (this._view.currentWorld === worlds[i])
        document.getElementById(worlds[i] + '-button').checked = true;
    }
  }

  _addWorldToPane(name) {
    const worldLi = document.createElement('li');
    worldLi.className = 'vertical-list-pane-li';
    worldLi.id = 'enable-world-' + name;
    this.worldList.appendChild(worldLi);

    const button = document.createElement('label');
    button.className = 'vertical-list-pane-switch';
    worldLi.appendChild(button);

    let label = document.createElement('input');
    label.id = name + '-button';
    label.type = 'checkbox';
    label.checked = false;
    label.style.display = 'none';
    button.appendChild(label);

    label = document.createElement('span');
    label.className = 'visibility-dot';
    button.appendChild(label);

    label = document.createElement('div');
    label.className = 'vertical-list-pane-spacer';
    worldLi.appendChild(label);

    label = document.createElement('span');
    label.id = 'enable-world-text-' + name;
    label.className = 'world-span';
    label.innerHTML = name;
    worldLi.appendChild(label);

    worldLi.onclick = _ => {
      this._changeWorld(name);
    };
  }

  _changeWorld(name) {
    if (this._view.currentWorld === name)
      return;

    this.worldSelectionPane.style.visibility = 'hidden';
    document.getElementById(this._view.currentWorld + '-button').checked = false;
    document.getElementById(name + '-button').checked = true;

    if (this._view.broadcast || typeof name === 'undefined')
      return;
    this._view.progress.setProgressBar('block', 'Loading ' + name + '...');
    this.hideToolbar(true);
    let previousOnready = this._view.onready;
    let stateBeforeChange = this._view.currentState;
    this._view.onready = () => {
      if (previousOnready === 'function')
        previousOnready();
      this.showToolbar(true);
      if (stateBeforeChange === 'real-time')
        this.realTime();
      else if (stateBeforeChange === 'fast' || stateBeforeChange === 'run')
        this.run();
    };
    this._view.stream.socket.send('load:' + name);
  }

  _changeWorldSelectionPaneVisibility(event) {
    if (this.worldSelectionPane.style.visibility === 'hidden') {
      this.worldSelectionPane.style.visibility = 'visible';
      for (let i of document.getElementsByClassName('tooltip'))
        i.style.visibility = 'hidden';
    } else {
      this.worldSelectionPane.style.visibility = 'hidden';
      if (event !== 'undefined' && !(event.srcElement.id.startsWith('settings') ||
        event.srcElement.id.startsWith('robot-window'))) {
        for (let i of document.getElementsByClassName('tooltip'))
          i.style.visibility = '';
      }
    }
  }

  _closeWorldSelectionPaneOnClick(event) {
    if (event.srcElement.id !== 'world-selection-button' && this.worldSelectionPane.style.visibility === 'visible') {
      if (!event.srcElement.id.startsWith('enable-world'))
        this._changeWorldSelectionPaneVisibility(event);
    }
  }

  removeStreamingToolbar() {
    document.removeEventListener('keydown', this.keydownRefK);
    this.keydownRefK = undefined;
    document.removeEventListener('mouseup', this.settingsRef);
    this.settingsRef = undefined;

    if (typeof this._gtaoPane !== 'undefined') {
      this.parentNode.removeChild(this._gtaoPane);
      this._gtaoPane = undefined;
    }

    if (typeof this.settingsPane !== 'undefined') {
      this.parentNode.removeChild(this.settingsPane);
      this.settingsPane = undefined;
    }

    this.removeToolbar();
  }
}
