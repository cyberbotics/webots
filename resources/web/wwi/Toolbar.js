import Animation from './Animation.js';
import AnimationSlider from './AnimationSlider.js';
import {requestFullscreen, exitFullscreen, onFullscreenChange} from './fullscreen_handler.js';
import InformationPanel from './InformationPanel.js';
import FloatingIde from './FloatingIde.js';
import FloatingRobotWindow from './FloatingRobotWindow.js';
import {changeShadows, changeGtaoLevel, GtaoLevel} from './nodes/wb_preferences.js';
import WbWorld from './nodes/WbWorld.js';

export default class Toolbar {
  constructor(view, type, parentNode) {
    this._view = view;
    this.type = type;
    this.parentNode = parentNode;
    this.minWidth = 0;

    this._createToolbar(parentNode);
    if (type === 'animation')
      this.createAnimationToolbar();
    else if (type === 'scene')
      this.createSceneToolbar();
    else if (type === 'streaming')
      this.createStreamingToolbar();

    parentNode.style.minWidth = this.minWidth + 'px';
  }

  createAnimationToolbar() {
    if (this.type !== 'animation' || typeof this._view === 'undefined' || typeof this._view.animation === 'undefined')
      return;

    this._createSlider();

    // Left part
    this._createPlayButton();
    this._createAnimationTimeIndicator();

    // Right part
    this._createInfoButton();
    this._createSettings();
    this._createFullscreenButtons();
  }

  createSceneToolbar() {
    if (this.type !== 'scene' || typeof this._view === 'undefined')
      return;

    this._createInfoButton();
    this._createRestoreViewpointButton();
    this._createFullscreenButtons();
  }

  createStreamingToolbar() {
    this.toolbar.style.backgroundColor = 'rgba(0, 0, 0, 0.4)';

    // Left part
    this._createQuitButton();
    this._createReloadButton();
    this._createStreamingTimeIndicator();
    this._createResetButton();
    this._createStepButton();
    this._createPlayButton();
    this._createRunButton();
    this._createWorldSelection();
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
    this._createFullscreenButtons();
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

    if (typeof this.toolbar !== 'undefined') {
      this.parentNode.removeChild(this.toolbar);
      this.toolbar = undefined;
    }
  }

  _createToolBarButton(name, tooltipText, click) {
    const button = document.createElement('button');
    button.id = name + '-button';
    button.className = 'toolbar-btn icon-' + name;
    if (typeof click === 'function')
      button.onclick = () => click();

    const tooltip = document.createElement('span');
    tooltip.className = 'tooltip ' + name + '-tooltip';
    tooltip.id = name + 'Tooltip';
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

    let canHide;

    let isPlaying = true;
    if (document.getElementById('play-button') && document.getElementById('play-button').className !== 'toolbar-btn icon-pause')
      isPlaying = false;

    let settingsPane = true;
    if (typeof this._settingsPane !== 'undefined' && this._settingsPane.style.visibility !== 'hidden')
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
        isPlaying = isPlaying || document.getElementById('run-button').className === 'toolbar-btn icon-pause';

      canHide = isPlaying && settingsPane && gtaoPane;
    } else if (this.type === 'scene')
      canHide = true;

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
    this.playTooltip = this.playButton.childNodes[0];

    if (action === 'pause') {
      this.playButton.className = 'toolbar-btn icon-pause';
      this.playTooltip.innerHTML = 'Pause (k)';
    }

    this.toolbarLeft.appendChild(this.playButton);
    document.addEventListener('keydown', this.keydownRefK = _ => this._playKeyboardHandler(_));
    if (!(typeof this.parentNode.showPlay === 'undefined' || this.parentNode.showPlay))
      this.playButton.style.display = 'none';
    else
      this.minWidth += 41;
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
      this.runButton.className = 'toolbar-btn icon-run';
    }

    this.playTooltip.innerHTML = 'P' + action.substring(1) + ' (k)';
    this.playButton.className = 'toolbar-btn icon-' + action;
  }

  _playKeyboardHandler(e) {
    if (e.code === 'KeyK')
      this._triggerPlayPauseButton();
  }

  _createIdeButton() {
    this.ideButton = this._createToolBarButton('ide', 'Theia IDE', undefined);
    this.toolbarRight.appendChild(this.ideButton);
    this._createIde();
    if (!this.parentNode.showIde && !this._view.theia)
      this.ideButton.style.display = 'none';
    else
      this.minWidth += 41;
  }
  _createIde() {
    this.floatingIdeContainer = document.createElement('div');
    this.floatingIdeContainer.className = 'floating-window-container';
    this.parentNode.appendChild(this.floatingIdeContainer);

    const url = this._view.x3dScene.prefix.slice(0, -1);

    let ideWindow = new FloatingIde(this.floatingIdeContainer, 'ide', url);

    const robotWindowWidth = 500;
    const robotWindowHeight = 500;
    const margin = 20;

    ideWindow.floatingWindow.addEventListener('mouseover', () => this.showToolbar());
    ideWindow.headerQuit.addEventListener('mouseup', _ => this._changeFloatingWindowVisibility(ideWindow.getID()));

    ideWindow.setSize(robotWindowWidth, robotWindowHeight);
    ideWindow.setPosition(margin, margin);
    this.ideButton.onclick = () => this._changeFloatingWindowVisibility(ideWindow.getID());
  }

  _createRobotWindowButton() {
    if (typeof WbWorld.instance !== 'undefined' && WbWorld.instance.readyForUpdates) {
      if (WbWorld.instance.robots.length > 0) {
        this.robotWindowButton = this._createToolBarButton('robot-window', 'Robot windows (w)');
        this.toolbarRight.appendChild(this.robotWindowButton);
        this.loadRobotWindows();
        this.robotWindowButton.addEventListener('mouseup', this.mouseupRefWFirst = _ => this._showAllRobotWindows(), {once: true});
        document.addEventListener('keydown', this.keydownRefWFirst = _ => this._robotWindowPaneKeyboardHandler(_, true), {once: true});
        this.keydownRefW = undefined;
        window.addEventListener('click', _ => this._closeRobotWindowPaneOnClick(_));
        if (!(typeof this.parentNode.showRobotWindow === 'undefined' || this.parentNode.showRobotWindow))
          this.robotWindowButton.style.display = 'none';
        else
          this.minWidth += 41;
      }
    }
  }

  _createRobotWindowPane() {
    this.robotWindowPane = document.createElement('div');
    this.robotWindowPane.className = 'robot-window-pane';
    this.robotWindowPane.innerHTML = '<h3 class="robot-window-pane-title">Robot Windows</h3>';
    this.robotWindowPane.style.visibility = 'hidden';
    this.parentNode.appendChild(this.robotWindowPane);

    this.robotWindowList = document.createElement('ul');
    this.robotWindowList.id = 'robot-window-list';
    this.robotWindowPane.appendChild(this.robotWindowList);
  }

  _closeRobotWindowPaneOnClick(event) {
    if (event.srcElement.id !== 'robot-window-button' && this.robotWindowPane.style.visibility === 'visible') {
      if (!(event.srcElement.id.startsWith('close-') || event.srcElement.id.startsWith('enable-robot-window')))
        this._changeRobotWindowPaneVisibility(event);
    }
  }

  _addRobotWindowToPane(name) {
    const robotWindowLi = document.createElement('li');
    robotWindowLi.className = 'robot-window-pane-li';
    robotWindowLi.id = 'enable-robot-window-' + name;
    this.robotWindowList.appendChild(robotWindowLi);

    const button = document.createElement('label');
    button.className = 'robot-window-pane-switch';
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
    label.className = 'robot-window-pane-spacer';
    robotWindowLi.appendChild(label);

    label = document.createElement('span');
    label.id = 'enable-robot-window-text-' + name;
    label.className = 'robot-window-span';
    label.innerHTML = name;
    robotWindowLi.appendChild(label);

    robotWindowLi.onclick = _ => {
      this._changeFloatingWindowVisibility(name);
    };
  }

  _createRobotWindows() {
    this.floatingRobotWindowContainer = document.createElement('div');
    this.floatingRobotWindowContainer.className = 'floating-window-container';
    this.parentNode.appendChild(this.floatingRobotWindowContainer);

    const robotWindowUrl = this._view.x3dScene.prefix.slice(0, -1);

    this.robotWindows = [];
    if (typeof WbWorld.instance !== 'undefined' && WbWorld.instance.readyForUpdates) {
      WbWorld.instance.robots.forEach((robot) => this.robotWindows.push(new FloatingRobotWindow(this.floatingRobotWindowContainer, robot.name, robotWindowUrl, robot.window)));
      WbWorld.instance.robots.forEach((robot) => this._addRobotWindowToPane(robot.name));
    }

    const viewWidth = this.parentNode.offsetWidth;
    const viewHeight = this.parentNode.offsetHeight;
    const robotWindowWidth = 250;
    const robotWindowHeight = 200;
    const margin = 20;
    let numCol = 0;
    let numRow = 0;

    this.robotWindows.forEach((rw) => {
      rw.floatingWindow.addEventListener('mouseover', () => this.showToolbar());
      rw.headerQuit.addEventListener('mouseup', _ => this._changeFloatingWindowVisibility(rw.getID()));

      if (margin + (numCol + 1) * (margin + robotWindowWidth) > viewWidth) {
        numRow++;
        if (margin + (numRow + 1) * (margin + robotWindowHeight) > viewHeight)
          numRow = 0;
        numCol = 0;
      }

      rw.setSize(robotWindowWidth, robotWindowHeight);
      rw.setPosition(margin + numCol * (margin + robotWindowWidth), margin + numRow * (margin + robotWindowHeight));
      numCol++;
    });
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
    if (typeof this.floatingRobotWindowContainer !== 'undefined')
      this.floatingRobotWindowContainer.remove();
  }

  _changeRobotWindowPaneVisibility(e) {
    if (this.robotWindowPane.style.visibility === 'hidden') {
      this.robotWindowPane.style.visibility = 'visible';
      for (let i of document.getElementsByClassName('tooltip'))
        i.style.visibility = 'hidden';
    } else {
      this.robotWindowPane.style.visibility = 'hidden';
      if ((e !== 'undefined') && !e.srcElement.id.startsWith('settings')) {
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
      this.robotWindows.forEach((rw) => this._changeFloatingWindowVisibility(rw.getID()));
    this.robotWindowButton.addEventListener('mouseup', _ => this._changeRobotWindowPaneVisibility());
    document.addEventListener('keydown', this.keydownRefW = _ => this._robotWindowPaneKeyboardHandler(_, false));
  }

  _createInfoButton() {
    this.infoButton = this._createToolBarButton('info', 'Simulation information', () => this._displayInformationWindow());
    this.toolbarRight.appendChild(this.infoButton);
    this._createInformation();
    window.addEventListener('click', _ => this._closeInfoOnClick(_));
    this.minWidth += 41;
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
      else
        this.informationPanel.informationPanel.style.display = 'block';
    }
  }

  _closeInfoOnClick(event) {
    if (typeof this.informationPanel !== 'undefined' && !this.informationPanel.informationPanel.contains(event.target) && !this.infoButton.contains(event.target))
      this.informationPanel.informationPanel.style.display = 'none';
  }

  _createSettings() {
    this.toolbarRight.appendChild(this._createToolBarButton('settings', 'Settings'));
    this.minWidth += 41;
    this._createSettingsPane();
  }

  _createSettingsPane() {
    this._settingsPane = document.createElement('div');
    this._settingsPane.className = 'settings-pane';
    this._settingsPane.id = 'settings-pane';
    this._settingsPane.style.visibility = 'hidden';
    document.addEventListener('mouseup', this.settingsRef = _ => this._changeSettingsPaneVisibility(_));
    this.parentNode.appendChild(this._settingsPane);

    this.settingsList = document.createElement('ul');
    this.settingsList.id = 'settings-list';
    this._settingsPane.appendChild(this.settingsList);

    this._createResetViewpoint();
    this._createChangeShadows();
    this._createChangeGtao();
    if (this.type === 'animation')
      this._createChangeSpeed();
  }

  _changeSettingsPaneVisibility(event) {
    if (event.srcElement.id === 'enable-shadows' || event.srcElement.id === 'gtao-settings') // avoid to close the settings when modifying the shadows or the other options
      return;

    if (typeof this._settingsPane === 'undefined' || typeof this._gtaoPane === 'undefined')
      return;

    let speedPanelHidden = true;
    if (this.type === 'animation') {
      if (event.srcElement.id === 'playback-li' || typeof this._speedPane === 'undefined')
        return;
      else if (this._speedPane.style.visibility === 'visible')
        speedPanelHidden = false;
    }

    if (event.target.id === 'settings-button' && this._settingsPane.style.visibility === 'hidden' && this._gtaoPane.style.visibility === 'hidden' && speedPanelHidden) {
      this._settingsPane.style.visibility = 'visible';
      const tooltips = document.getElementsByClassName('tooltip');
      for (let i of tooltips)
        i.style.visibility = 'hidden';
    } else if (this._settingsPane.style.visibility === 'visible' || this._gtaoPane.style.visibility === 'visible' || !speedPanelHidden) {
      this._settingsPane.style.visibility = 'hidden';
      if (this._gtaoPane.style.visibility === 'hidden' && speedPanelHidden) {
        const tooltips = document.getElementsByClassName('tooltip');
        if ((event !== 'undefined') && !event.srcElement.id.startsWith('robot-window')) {
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
    label.innerHTML = this._gtaoLevelToText(GtaoLevel);
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

    for (let i of ['Low', 'Normal', 'High', 'Ultra']) {
      gtaoLevelLi = document.createElement('li');
      gtaoLevelLi.id = i;
      label = document.createElement('span');
      if (this._gtaoLevelToText(GtaoLevel) === i)
        label.innerHTML = '&check;';
      label.id = 'c' + i;
      label.className = 'check-gtao';
      gtaoLevelLi.appendChild(label);
      label = document.createElement('span');
      label.innerHTML = i;
      label.className = 'setting-span';
      gtaoLevelLi.appendChild(label);
      label = document.createElement('div');
      label.className = 'spacer';
      gtaoLevelLi.appendChild(label);
      gtaoLevelLi.onclick = _ => this._changeGtao(_);
      gtaoList.appendChild(gtaoLevelLi);
    }
  }

  _changeGtao(event) {
    changeGtaoLevel(this._textToGtaoLevel(event.srcElement.id));
    this._gtaoPane.style.visibility = 'hidden';
    const gtaoLabel = document.getElementById('gtao-display');
    if (gtaoLabel)
      gtaoLabel.innerHTML = event.srcElement.id;
    this._settingsPane.style.visibility = 'visible';
    for (let i of document.getElementsByClassName('check-gtao')) {
      if (i.id === 'c' + event.srcElement.id)
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
    this._settingsPane.style.visibility = 'hidden';
    this._gtaoPane.style.visibility = 'visible';
  }

  _closeGtaoPane() {
    this._settingsPane.style.visibility = 'visible';
    this._gtaoPane.style.visibility = 'hidden';
  }

  _gtaoLevelToText(number) {
    const pairs = {
      1: 'Low',
      2: 'Medium',
      3: 'High',
      4: 'Ultra'
    };
    return (number in pairs) ? pairs[number] : '';
  }

  _textToGtaoLevel(text) {
    const pairs = {
      'Low': 1,
      'Medium': 2,
      'High': 3,
      'Ultra': 4
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

    this._exitFullscreenButton = this._createToolBarButton('partscreen', 'Exit full screen (f)', () => exitFullscreen());
    this.toolbarRight.appendChild(this._exitFullscreenButton);
    this._exitFullscreenButton.style.display = 'none';

    this.minWidth += 41;

    document.addEventListener('fullscreenchange', this.fullscreenRef = () => onFullscreenChange(this._fullscreenButton, this._exitFullscreenButton));
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
    this._timeSlider.shadowRoot.getElementById('range').addEventListener('mousemove', this.updateFloatingTimeRef = _ => this._updateFloatingTimePosition(_));
    this._timeSlider.shadowRoot.getElementById('range').addEventListener('mouseleave', this.hideFloatingTimeRef = _ => this._hideFloatingTimePosition(_));
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

    this.minWidth += 110;
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
      this._settingsPane.style.visibility = 'visible';
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
    this._settingsPane.style.visibility = 'hidden';
    this._speedPane.style.visibility = 'visible';
  }

  _closeSpeedPane() {
    this._settingsPane.style.visibility = 'visible';
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
    this.minWidth += 41;
  }

  // Streaming functions

  _createQuitButton() {
    const quitButton = this._createToolBarButton('quit', 'Quit the simulation', () => this._view.quitSimulation());
    if (!(typeof this.parentNode.showQuit === 'undefined' || this.parentNode.showQuit))
      quitButton.style.display = 'none';
    else
      this.minWidth += 41;

    this.toolbarLeft.appendChild(quitButton);
  }

  _createReloadButton() {
    const reloadButton = this._createToolBarButton('reload', 'Reload the simulation', () => { this.reset(true); });
    if (!this.parentNode.showReload)
      reloadButton.style.display = 'none';
    else
      this.minWidth += 41;
    this.toolbarLeft.appendChild(reloadButton);
  }

  reset(reload = false) {
    if (this._view.broadcast)
      return;
    if (document.getElementById('webotsProgressMessage')) {
      if (reload)
        document.getElementById('webotsProgressMessage').innerHTML = 'Reloading simulation...';
      else
        document.getElementById('webotsProgressMessage').innerHTML = 'Restarting simulation...';
    }
    if (document.getElementById('webotsProgress'))
      document.getElementById('webotsProgress').style.display = 'block';

    if (typeof this.pauseButton !== 'undefined' && this.playButton.className === 'toolbar-btn icon-pause')
      this._view.currentState = 'real-time';
    else if (typeof this.runButton !== 'undefined' && this.runButton.className === 'toolbar-btn icon-pause')
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

    clock.id = 'webotsClock';
    clock.title = 'Current simulation time';
    clock.innerHTML = this._parseMillisecondsIntoReadableTime(0);
    this.toolbarLeft.appendChild(clock);
    this.minWidth += 105;
  }

  _createResetButton() {
    let resetButton = this._createToolBarButton('reset', 'Reset the simulation', () => this.reset(false));
    this.toolbarLeft.appendChild(resetButton);
    if (!(typeof this.parentNode.showReset === 'undefined' || this.parentNode.showReset))
      resetButton.style.display = 'none';
    else
      this.minWidth += 41;
  }

  _createStepButton() {
    let stepButton = this._createToolBarButton('step', 'Perform one simulation step', () => this.step());
    this.toolbarLeft.appendChild(stepButton);
    if (!(typeof this.parentNode.showStep === 'undefined' || this.parentNode.showStep))
      stepButton.style.display = 'none';
    else
      this.minWidth += 41;
  }

  step() {
    if (this._view.broadcast)
      return;

    if (typeof this.playButton !== 'undefined') {
      this.playTooltip.innerHTML = 'Play (k)';
      this.playButton.className = 'toolbar-btn icon-play';
    }

    if (typeof this.runButton !== 'undefined') {
      this.runTooltip.innerHTML = 'Run';
      this.runButton.className = 'toolbar-btn icon-run';
    }

    this.pause();

    this._view.stream.socket.send('step');
  }

  _createRunButton() {
    this.runButton = this._createToolBarButton('run', 'Run', () => this._triggerRunPauseButton());
    this.runTooltip = this.runButton.childNodes[0];
    if (!this.parentNode.showRun)
      this.runButton.style.display = 'none';
    else
      this.minWidth += 41;
    if (this._view.currentState === 'run' || this._view.currentState === 'fast') {
      this.runTooltip.innerHTML = 'Pause';
      this.runButton.className = 'toolbar-btn icon-pause';
      this.run();
    }
    this.toolbarLeft.appendChild(this.runButton);
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
      this.playButton.className = 'toolbar-btn icon-play';
    }

    this.runTooltip.innerHTML = action.charAt(0).toUpperCase() + action.slice(1);
    this.runButton.className = 'toolbar-btn icon-' + action;
  }

  run() {
    if (this._view.broadcast)
      return;
    this._view.stream.socket.send('fast:' + this._view.timeout);
    this._view.currentState = 'fast';
  }

  _createWorldSelection() {
    this.worldSelectionDiv = document.createElement('div');
    this.worldSelectionDiv.id = 'worldSelectionDiv';
    this.toolbarLeft.appendChild(this.worldSelectionDiv);
    if (this.createWorldSelect() !== -1)
      this.minWidth += 270;
  }

  createWorldSelect() {
    const worlds = this._view.worlds;
    if (typeof worlds === 'undefined' || worlds.length <= 1)
      return -1;
    this.worldSelect = document.createElement('select');
    this.worldSelect.id = 'worldSelection';
    this.worldSelect.classList.add('select-css');
    this.worldSelectionDiv.appendChild(this.worldSelect);

    for (let i in worlds) {
      const option = document.createElement('option');
      option.value = worlds[i];
      option.text = worlds[i];
      this.worldSelect.appendChild(option);
      if (this._view.currentWorld === worlds[i])
        this.worldSelect.selectedIndex = i;
    }

    this.worldSelect.onchange = () => {
      if (this._view.broadcast || typeof this.worldSelect === 'undefined')
        return;
      if (document.getElementById('webotsProgressMessage'))
        document.getElementById('webotsProgressMessage').innerHTML = 'Loading ' + this.worldSelect.value + '...';
      if (document.getElementById('webotsProgress'))
        document.getElementById('webotsProgress').style.display = 'block';
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
      this._view.stream.socket.send('load:' + this.worldSelect.value);
    };
  }

  deleteWorldSelect() {
    if (typeof this.worldSelectionDiv !== 'undefined' && typeof this.worldSelect !== 'undefined') {
      this.worldSelectionDiv.removeChild(this.worldSelect);
      this.worldSelect = undefined;
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

    if (typeof this._settingsPane !== 'undefined') {
      this.parentNode.removeChild(this._settingsPane);
      this._settingsPane = undefined;
    }

    this.removeToolbar();
  }
}
