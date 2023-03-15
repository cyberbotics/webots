import Animation from './Animation.js';
import AnimationSlider from './AnimationSlider.js';
import {requestFullscreen, exitFullscreen, onFullscreenChange, isFullscreen} from './fullscreen_handler.js';
import InformationPanel from './InformationPanel.js';
import FloatingIde from './FloatingIde.js';
import FloatingRobotWindow from './FloatingRobotWindow.js';
import FloatingCustomWindow from './FloatingCustomWindow.js';
import FloatingProtoParameterWindow from './FloatingProtoParameterWindow.js';
import {changeShadows, changeGtaoLevel, GtaoLevel} from './nodes/wb_preferences.js';
import SystemInfo from './system_info.js';
import Terminal from './Terminal.js';
import WbWorld from './nodes/WbWorld.js';

export default class Toolbar {
  #currentTime;
  #exitFullscreenButton;
  #fullscreenButton;
  #gtaoPane;
  #previousScreenOrientation;
  #scale;
  #speedPane;
  #timeSlider;
  #toolbarResizeObserver;
  #unusedPrefix;
  #view;
  constructor(view, type, parentNode) {
    this.#view = view;
    this.type = type;
    this.parentNode = parentNode;
    this.minWidth = 0;
    this.#scale = 1;

    this.#createToolbar(parentNode);
    if (type === 'animation')
      this.#createAnimationToolbar();
    else if (type === 'scene')
      this.#createSceneToolbar();
    else if (type === 'streaming')
      this.#createStreamingToolbar();
    else if (type === 'proto')
      this.#createProtoToolbar();

    this.#resizeToolbar();
    this.toolbar.style.minWidth = this.minWidth + 'px';

    if (SystemInfo.isMobileDevice() && !SystemInfo.isIOS() && !SystemInfo.isSafari()) {
      // Warning: window.orientation is deprecated but screen.orientation is not supported by iOS so we use it in this case.
      if (!screen.orientation || screen.orientation === 'undefined')
        this.#previousScreenOrientation = window.orientation % 180 === 0 ? 'portrait' : 'landscape';
      else
        this.#previousScreenOrientation = screen.orientation.type.includes('portrait') ? 'portrait' : 'landscape';

      screen.orientation.addEventListener('change', this.#mobileOrientationChangeFullscreen.bind(this));

      this.#fullscreenButton.style.animation = 'animation-scale-up-lg 2s infinite forwards';
      this.#fullscreenButton.addEventListener('click', () => this.#removeFullscreenAnimation(this.#fullscreenButton),
        {once: true});
    }
  }

  #createAnimationToolbar() {
    if (this.type !== 'animation' || typeof this.#view === 'undefined' || typeof this.#view.animation === 'undefined')
      return;

    this.#createSlider();

    // Left part
    this.#createPlayButton();
    this.#createAnimationTimeIndicator();

    // Right part
    this.#createCustomWindowButton();
    this.#createInfoButton();
    this.#createSettings();
    if (!SystemInfo.isIOS() && !SystemInfo.isSafari())
      this.#createFullscreenButtons();
  }

  #createSceneToolbar() {
    if ((this.type !== 'scene' && this.type !== 'proto') || typeof this.#view === 'undefined')
      return;

    this.#createInfoButton();
    this.#createSettings();
    if (!SystemInfo.isIOS() && !SystemInfo.isSafari())
      this.#createFullscreenButtons();
  }

  #createProtoToolbar() {
    this.#createProtoParameterButton();
    this.#createSceneToolbar();
  }

  #createStreamingToolbar() {
    // Left part
    this.#createQuitButton();
    this.#createReloadButton();
    this.#createWorldSelectionButton();
    this.#createResetButton();
    this.#createStepButton();
    this.#createPlayButton();
    this.#createRunButton();
    this.#createStreamingTimeIndicator();
    if (this.#view.broadcast) {
      this.toolbarLeft.style.visibility = 'hidden';
      this.minWidth = 0;
    }

    // Right part
    this.#createTerminalButton();
    this.#createIdeButton();
    this.#createRobotWindowButton();
    this.#createInfoButton();
    if (this.#view.mode !== 'mjpeg')
      this.#createSettings();
    if (!SystemInfo.isIOS() && !SystemInfo.isSafari())
      this.#createFullscreenButtons();
  }

  #resizeToolbar() {
    this.#toolbarResizeObserver = new ResizeObserver(() => {
      this.#scale = this.minWidth > this.parentNode.offsetWidth ? this.parentNode.offsetWidth / this.minWidth : 1;
      this.toolbar.style.transformOrigin = 'bottom left';
      this.toolbar.style.transform = 'scale(' + this.#scale + ')';
      if (typeof this.robotWindowPane !== 'undefined') {
        const offset = this.#scale === 1 ? 0 : this.parentNode.offsetWidth * (1 - this.#scale);
        this.robotWindowPane.style.transform = 'translateX(' + offset + 'px)';
      }
    });
    const viewElement = document.getElementsByClassName('webots-view')[0];
    if (viewElement)
      this.#toolbarResizeObserver.observe(viewElement);
  }

  #mobileOrientationChangeFullscreen() {
    let screenOrientation;
    // Warning: window.orientation is deprecated but screen.orientation is not supported by iOS so we use it in this case.
    if (!screen.orientation || screen.orientation === 'undefined')
      screenOrientation = window.orientation % 180 === 0 ? 'portrait' : 'landscape';
    else
      screenOrientation = screen.orientation.type.includes('portrait') ? 'portrait' : 'landscape';
    if (this.#previousScreenOrientation === 'portrait' && screenOrientation === 'landscape' && !isFullscreen())
      requestFullscreen(this.#view);
    else if (this.#previousScreenOrientation === 'landscape' && screenOrientation === 'portrait' && isFullscreen())
      exitFullscreen();

    this.#previousScreenOrientation = screenOrientation;
  }

  removeToolbar() {
    if (typeof this.#view.mouseEvents !== 'undefined' && typeof this.#view.mouseEvents.hideToolbar) {
      this.#view.mouseEvents.hideToolbar = undefined;
      if (typeof this.#view.mouseEvents.showToolbar !== 'undefined') {
        this.#view.mouseEvents.showToolbar();
        this.#view.mouseEvents.showToolbar = undefined;
      }
    }

    document.removeEventListener('fullscreenchange', this.fullscreenRef);
    this.fullscreenRef = undefined;
    document.removeEventListener('keydown', this.keydownRefF);
    this.keydownRefF = undefined;

    window.removeEventListener('click', this.#closeInfoOnClick);

    if (this.#toolbarResizeObserver)
      this.#toolbarResizeObserver.disconnect();

    if (typeof this.toolbar !== 'undefined') {
      this.parentNode.removeChild(this.toolbar);
      this.toolbar = undefined;
    }
  }

  #createToolbarButton(name, tooltipText, click) {
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

    button.title = tooltipText;
    return button;
  }

  #createToolbar(parentNode) {
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
    if (typeof this.#view.mouseEvents !== 'undefined' && this.#view.mode !== 'mjpeg') {
      this.toolbar.addEventListener('mouseover', () => this.showToolbar());
      this.toolbar.addEventListener('mouseleave', _ => this.#onMouseLeave(_));
      this.#view.mouseEvents.hideToolbar = () => this.hideToolbar();
      this.#view.mouseEvents.showToolbar = () => this.showToolbar();
    }
  }

  #onMouseLeave(e) {
    if (e.relatedTarget != null && e.relatedTarget.id !== 'canvas')
      this.#view.mouseEvents.onMouseLeave();
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
    if (typeof this.#gtaoPane !== 'undefined' && this.#gtaoPane.style.visibility !== 'hidden')
      gtaoPane = false;

    if (this.type === 'animation') {
      const isSelected = this.#timeSlider.selected();
      let speedPane = true;
      if (typeof this.#speedPane !== 'undefined' && this.#speedPane.style.visibility !== 'hidden')
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

  #createPlayButton() {
    let action;
    if (this.type === 'animation')
      action = (this.#view.animation.gui === 'real-time') ? 'pause' : 'play';
    else if (this.type === 'streaming') {
      action = (this.#view.currentState === 'real-time') ? 'pause' : 'play';
      if (action === 'pause')
        this.realTime();
    }

    this.playButton = this.#createToolbarButton('play', 'Play (k)', () => this.#triggerPlayPauseButton());
    this.playButtonElement = this.playButton.childNodes[0];
    this.playButtonElement.title = this.playButton.childNodes[1];

    if (action === 'pause') {
      this.playButtonElement.className = 'icon-pause';
      this.playButtonElement.title = 'Pause (k)';
    }

    this.toolbarLeft.appendChild(this.playButton);
    document.addEventListener('keydown', this.keydownRefK = _ => this.#playKeyboardHandler(_));
    if (!(typeof this.parentNode.showPlay === 'undefined' || this.parentNode.showPlay))
      this.playButton.style.display = 'none';

    this.minWidth += 44;
    if (typeof this.#view.stream !== 'undefined')
      this.#view.stream.onplay = () => this.#triggerPlayPauseButton();
  }

  #triggerPlayPauseButton() {
    const animation = this.#view.animation;
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
      if (this.#view.currentState === 'real-time') {
        this.pause();
        action = 'play';
      } else {
        this.realTime();
        action = 'pause';
      }
    }

    if (typeof this.runButton !== 'undefined') {
      this.runButtonElement.title = 'Run';
      this.runButtonElement.className = 'icon-run';
    }

    this.playButtonElement.title = 'P' + action.substring(1) + ' (k)';
    this.playButtonElement.className = 'icon-' + action;
  }

  #playKeyboardHandler(e) {
    if (e.code === 'KeyK')
      this.#triggerPlayPauseButton();
  }

  #createTerminalButton() {
    this.terminalButton = this.#createToolbarButton('terminal', 'Terminal', undefined);
    this.toolbarRight.appendChild(this.terminalButton);
    this.#createTerminal();
    if (!(typeof this.parentNode.showTerminal === 'undefined' || this.parentNode.showTerminal))
      this.terminalButton.style.display = 'none';
    else
      this.minWidth += 44;
  }

  #createTerminal() {
    this.terminal = new Terminal(this.parentNode);

    const terminalWidth = 0.6 * this.parentNode.offsetWidth;
    const terminalHeight = 0.75 * this.parentNode.offsetHeight;
    const terminalPositionX = (this.parentNode.offsetWidth - terminalWidth) / 2;
    const terminalPositionY = (this.parentNode.offsetHeight - terminalHeight) / 2;

    this.terminal.floatingWindow.addEventListener('mouseover', () => this.showToolbar());
    this.terminal.headerQuit.addEventListener('mouseup', _ => this.#changeFloatingWindowVisibility(this.terminal.getId()));

    this.terminal.setSize(terminalWidth, terminalHeight);
    this.terminal.setPosition(terminalPositionX, terminalPositionY);
    this.terminalButton.onclick = () => this.#changeFloatingWindowVisibility(this.terminal.getId());

    this.#checkWindowBoundaries();
  }

  #createIdeButton() {
    // Do not create IDE button if no IDE is available or screen is too small
    if (this.#view.mobileDevice && Math.min(screen.height, screen.width) < 700)
      this.#view.ide = false;
    if (!this.#view.ide)
      return;

    this.ideButton = this.#createToolbarButton('ide', 'Source code editor', undefined);
    this.toolbarRight.appendChild(this.ideButton);
    this.#createIde();
    if (!(typeof this.parentNode.showIde === 'undefined' || this.parentNode.showIde))
      this.ideButton.style.display = 'none';
    else
      this.minWidth += 44;
  }

  #createIde() {
    const url = this.#view.prefix.slice(0, -1);
    this.ideWindow = new FloatingIde(this.parentNode, 'ide', url);

    const ideWidth = 0.6 * this.parentNode.offsetWidth;
    const ideHeight = 0.75 * this.parentNode.offsetHeight;
    const idePositionX = (this.parentNode.offsetWidth - ideWidth) / 2;
    const idePositionY = (this.parentNode.offsetHeight - ideHeight) / 2;

    this.ideWindow.floatingWindow.addEventListener('mouseover', () => this.showToolbar());
    this.ideWindow.headerQuit.addEventListener('mouseup', _ => this.#changeFloatingWindowVisibility(this.ideWindow.getId()));

    this.ideWindow.setSize(ideWidth, ideHeight);
    this.ideWindow.setPosition(idePositionX, idePositionY);
    this.ideButton.onclick = () => this.#changeFloatingWindowVisibility(this.ideWindow.getId());

    this.#checkWindowBoundaries();
  }

  #createRobotWindowButton() {
    if (this.#view.robots.length > 0) {
      this.robotWindowButton = this.#createToolbarButton('robot-window', 'Robot windows (w)');
      this.toolbarRight.appendChild(this.robotWindowButton);
      this.robotWindowButton.addEventListener('mouseup', this.mouseupRefWFirst = _ => this.#showAllRobotWindows(),
        {once: true});
      document.addEventListener('keydown', this.keydownRefWFirst = _ => this.#robotWindowPaneKeyboardHandler(_, true),
        {once: true});
      this.keydownRefW = undefined;
      window.addEventListener('click', _ => this.#closeRobotWindowPaneOnClick(_));
      if (!(typeof this.parentNode.showRobotWindow === 'undefined' || this.parentNode.showRobotWindow))
        this.robotWindowButton.style.display = 'none';
      else
        this.minWidth += 44;
    }
  }

  #createRobotWindowPane() {
    this.robotWindowPane = document.createElement('div');
    this.robotWindowPane.id = 'robot-window-pane';
    this.robotWindowPane.className = 'vertical-list-pane';
    this.robotWindowPane.style.visibility = 'hidden';
    this.parentNode.appendChild(this.robotWindowPane);

    this.worldInfoPaneTitle = document.createElement('div');
    this.worldInfoPaneTitle.className = 'vertical-list-pane-title';
    this.worldInfoPaneTitle.innerHTML = 'World Info';
    this.worldInfoPaneTitle.style.display = 'none';
    this.robotWindowPane.appendChild(this.worldInfoPaneTitle);

    this.worldInfoList = document.createElement('ul');
    this.worldInfoList.id = 'world-info-list';
    this.robotWindowPane.appendChild(this.worldInfoList);

    this.robotWindowPaneTitle = document.createElement('div');
    this.robotWindowPaneTitle.className = 'vertical-list-pane-title';
    this.robotWindowPaneTitle.innerHTML = 'Robot Windows';
    this.robotWindowPaneTitle.style.display = 'none';
    this.robotWindowPane.appendChild(this.robotWindowPaneTitle);

    this.robotWindowList = document.createElement('ul');
    this.robotWindowList.id = 'robot-window-list';
    this.robotWindowPane.appendChild(this.robotWindowList);

    const offset = this.#scale === 1 ? 0 : screen.width * (1 - this.#scale);
    this.robotWindowPane.style.transform = 'translateX(' + offset + 'px)';
  }

  #closeRobotWindowPaneOnClick(event) {
    if (typeof this.robotWindowPane !== 'undefined') {
      if (event.srcElement.id !== 'robot-window-button' && this.robotWindowPane.style.visibility === 'visible') {
        if (!(event.srcElement.id.startsWith('close-') || event.srcElement.id.startsWith('enable-robot-window')))
          this.#changeRobotWindowPaneVisibility(event);
      }
    }
  }

  #addRobotWindowToPane(name, mainWindow) {
    const robotWindowLi = document.createElement('li');
    robotWindowLi.className = 'vertical-list-pane-li';
    robotWindowLi.id = 'enable-robot-window-' + name;
    if (mainWindow) {
      this.worldInfoList.appendChild(robotWindowLi);
      this.worldInfoPaneTitle.style.display = 'block';
      this.robotWindowPaneTitle.style.borderTop = '1px solid #333';
    } else {
      this.robotWindowList.appendChild(robotWindowLi);
      this.robotWindowPaneTitle.style.display = 'block';
    }

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
    label.innerHTML = name;
    robotWindowLi.appendChild(label);

    robotWindowLi.onclick = _ => {
      this.#changeFloatingWindowVisibility(name);
    };
  }

  #createRobotWindows() {
    const robotWindowUrl = this.#view.prefix.slice(0, -1);

    this.robotWindows = [];
    this.#view.robots.forEach((robot) => {
      if (robot.window !== '<none>') {
        let mainWindow = robot.main;
        let robotWindow = new FloatingRobotWindow(this.parentNode, robot.name, robotWindowUrl, robot.window, mainWindow);
        if (robot.visible) // if the robot window is visible by default in Webots, then show it
          robotWindow.changeVisibility();
        if (mainWindow)
          this.robotWindows.unshift(robotWindow);
        else
          this.robotWindows.push(robotWindow);
        this.#addRobotWindowToPane(robot.name, mainWindow);
      }
    });
    const buttonDisplay = (this.robotWindows.length === 0) ? 'none' : 'auto';
    document.getElementById('robot-window-button').style.display = buttonDisplay;

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
      rw.headerQuit.addEventListener('mouseup', _ => this.#changeFloatingWindowVisibility(rw.getId()));

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

    this.#checkWindowBoundaries();
    this.#initializeWindowLayerChanges();
  }

  #initializeWindowLayerChanges() {
    window.addEventListener('blur', _ => {
      const newElement = document.activeElement;
      if (newElement.parentNode.parentNode.classList &&
        newElement.parentNode.parentNode.classList.contains('floating-window')) {
        document.querySelectorAll('.floating-window').forEach((fw) => { fw.style.zIndex = '3'; });
        document.getElementById(newElement.parentNode.parentNode.id).style.zIndex = '4';
      }
    });
  }

  #refreshRobotWindowContent() {
    this.robotWindows.forEach((rw) => {
      if (typeof document.getElementById(rw.name + '-robot-window').src !== 'undefined')
        document.getElementById(rw.name + '-robot-window').src = document.getElementById(rw.name + '-robot-window').src;
    });
  }

  loadRobotWindows() {
    this.removeRobotWindows();
    this.#createRobotWindowPane();
    this.#createRobotWindows();
  }

  removeRobotWindows() {
    this.robotWindowPane?.remove();

    if (typeof this.robotWindows !== 'undefined')
      document.querySelectorAll('.floating-robot-window').forEach(fw => fw.remove());
  }

  #changeRobotWindowPaneVisibility(event) {
    if (this.robotWindowPane.style.visibility === 'hidden')
      this.robotWindowPane.style.visibility = 'visible';
    else
      this.robotWindowPane.style.visibility = 'hidden';
  }

  #robotWindowPaneKeyboardHandler(e, isFirst) {
    if (e.code === 'KeyW') {
      if (isFirst)
        this.#showAllRobotWindows();
      else
        this.#changeRobotWindowPaneVisibility(e);
    }
  }

  #changeFloatingWindowVisibility(name) {
    const floatingWindow = document.getElementById(name);
    const floatingWindowButton = document.getElementById(name + '-button');
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

  #showAllRobotWindows() {
    document.removeEventListener('keydown', this.keydownRefWFirst);
    this.keydownRefWFirst = undefined;
    this.robotWindowButton.removeEventListener('mouseup', this.mouseupRefWFirst);
    this.mouseupRefWFirst = undefined;
    this.#changeRobotWindowPaneVisibility();
    if (this.robotWindows) {
      this.robotWindows.forEach((rw) => {
        if (rw.getVisibility() === 'hidden')
          this.#changeFloatingWindowVisibility(rw.getId());
      });
    }
    this.robotWindowButton.addEventListener('mouseup', _ => this.#changeRobotWindowPaneVisibility(_));
    document.addEventListener('keydown', this.keydownRefW = _ => this.#robotWindowPaneKeyboardHandler(_, false));
  }

  #checkWindowBoundaries() {
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
    resizeObserver.observe(document.getElementsByClassName('webots-view')[0]);
  }

  #createInfoButton() {
    this.infoButton = this.#createToolbarButton('info', 'Simulation information', () => this.#displayInformationWindow());
    this.toolbarRight.appendChild(this.infoButton);
    this.#createInformation();
    window.addEventListener('click', _ => this.#closeInfoOnClick(_));
    if (!(typeof this.parentNode.showInfo === 'undefined' || this.parentNode.showInfo))
      this.infoButton.style.display = 'none';
    else
      this.minWidth += 44;
  }

  #createInformation() {
    this.informationPanel = new InformationPanel(this.parentNode);

    if (typeof WbWorld.instance !== 'undefined' && WbWorld.instance.readyForUpdates) {
      this.informationPanel.setTitle(WbWorld.instance.title);
      this.informationPanel.setDescription(WbWorld.instance.description);
    }
  }

  #displayInformationWindow() {
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

  #closeInfoOnClick(event) {
    if (typeof this.informationPanel !== 'undefined' && !this.informationPanel.informationPanel.contains(event.target) &&
      !this.infoButton.contains(event.target))
      this.informationPanel.informationPanel.style.display = 'none';
  }

  #createSettings() {
    this.toolbarRight.appendChild(this.#createToolbarButton('settings', 'Settings'));
    this.minWidth += 44;
    this.#createSettingsPane();
  }

  #createSettingsPane() {
    this.settingsPane = document.createElement('div');
    this.settingsPane.className = 'settings-pane';
    this.settingsPane.id = 'settings-pane';
    this.settingsPane.style.visibility = 'hidden';
    document.addEventListener('mouseup', this.settingsRef = _ => this.#changeSettingsPaneVisibility(_));
    this.parentNode.appendChild(this.settingsPane);
    this.settingsPane.addEventListener('mouseover', () => this.showToolbar());

    this.settingsList = document.createElement('ul');
    this.settingsList.id = 'settings-list';
    this.settingsPane.appendChild(this.settingsList);

    this.#createResetViewpoint();
    this.#createChangeShadows();
    if (this.type === 'animation' || this.type === 'streaming')
      this.#createFollowObject();
    this.#createChangeGtao();
    if (this.type === 'animation')
      this.#createChangeSpeed();
  }

  #changeSettingsPaneVisibility(event) {
    // Avoid to close the settings when modifying the shadows or the other options
    if (event.srcElement.id === 'enable-shadows' || event.srcElement.id === 'enable-follow' ||
      event.srcElement.id === 'gtao-settings')
      return;

    if (typeof this.settingsPane === 'undefined' || typeof this.#gtaoPane === 'undefined')
      return;

    let speedPanelHidden = true;
    if (this.type === 'animation') {
      if (event.srcElement.id === 'playback-li' || typeof this.#speedPane === 'undefined')
        return;
      else if (this.#speedPane.style.visibility === 'visible')
        speedPanelHidden = false;
    }

    if (event.target.id === 'settings-button' && this.settingsPane.style.visibility === 'hidden' &&
      this.#gtaoPane.style.visibility === 'hidden' && speedPanelHidden)
      this.settingsPane.style.visibility = 'visible';
    else if (this.settingsPane.style.visibility === 'visible' || this.#gtaoPane.style.visibility === 'visible' ||
      !speedPanelHidden)
      this.settingsPane.style.visibility = 'hidden';

    this.#gtaoPane.style.visibility = 'hidden';
    if (this.type === 'animation')
      this.#speedPane.style.visibility = 'hidden';
  }

  #createResetViewpoint() {
    const resetViewpoint = document.createElement('li');
    resetViewpoint.onclick = () => this.#resetViewpoint();
    resetViewpoint.id = 'reset-viewpoint';
    this.settingsList.appendChild(resetViewpoint);

    let label = document.createElement('span');
    label.className = 'setting-span';
    label.innerHTML = 'Reset Viewpoint';
    resetViewpoint.appendChild(label);

    label = document.createElement('div');
    label.className = 'spacer';
    resetViewpoint.appendChild(label);
  }

  #createChangeShadows() {
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
      this.#view.x3dScene.render();
    };
  }

  #createFollowObject() {
    const followLi = document.createElement('li');
    followLi.id = 'enable-follow';
    this.settingsList.appendChild(followLi);

    let label = document.createElement('span');
    label.className = 'setting-span';
    label.innerHTML = 'Follow Object';
    followLi.appendChild(label);

    label = document.createElement('div');
    label.className = 'spacer';
    followLi.appendChild(label);

    const button = document.createElement('label');
    button.className = 'switch';
    followLi.appendChild(button);

    label = document.createElement('input');
    label.type = 'checkbox';
    label.checked = true;
    button.appendChild(label);

    label = document.createElement('span');
    label.className = 'slider round';
    button.appendChild(label);

    followLi.onclick = _ => {
      button.click();
      WbWorld.instance.viewpoint?.enableFollow();
    };
  }

  #createChangeGtao() {
    const gtaoLi = document.createElement('li');
    gtaoLi.id = 'gtao-settings';
    this.settingsList.appendChild(gtaoLi);
    gtaoLi.onclick = () => this.#openGtaoPane();

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

    this.#createGtaoPane();
  }

  #createGtaoPane() {
    this.#gtaoPane = document.createElement('div');
    this.#gtaoPane.className = 'settings-pane';
    this.#gtaoPane.id = 'gtao-pane';
    this.#gtaoPane.style.visibility = 'hidden';
    this.parentNode.appendChild(this.#gtaoPane);

    this.#gtaoPane.addEventListener('mouseover', () => this.showToolbar());

    const gtaoList = document.createElement('ul');
    this.#gtaoPane.appendChild(gtaoList);

    let gtaoLevelLi = document.createElement('li');
    gtaoLevelLi.className = 'first-li';

    let label = document.createElement('div');
    label.className = 'arrow-left';
    gtaoLevelLi.appendChild(label);

    label = document.createElement('span');
    label.innerHTML = 'Ambient Occlusion';
    label.className = 'setting-span';
    gtaoLevelLi.appendChild(label);

    label = document.createElement('div');
    label.className = 'spacer';
    gtaoLevelLi.appendChild(label);
    gtaoLevelLi.onclick = () => this.#closeGtaoPane();
    gtaoList.appendChild(gtaoLevelLi);

    for (let i of ['Low', 'Medium', 'High', 'Ultra']) {
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
    changeGtaoLevel(this.#textToGtaoLevel(event.srcElement.id));
    this.#gtaoPane.style.visibility = 'hidden';
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
    const animation = this.#view.animation;
    if (animation)
      animation.start = new Date().getTime() - animation.data.basicTimeStep * animation.step / animation.speed;
    this.#view.x3dScene.render();
  }

  #openGtaoPane() {
    this.settingsPane.style.visibility = 'hidden';
    this.#gtaoPane.style.visibility = 'visible';
  }

  #closeGtaoPane() {
    this.settingsPane.style.visibility = 'visible';
    this.#gtaoPane.style.visibility = 'hidden';
  }

  gtaoLevelToText(number) {
    const pairs = {
      1: 'Low',
      2: 'Medium',
      3: 'High',
      4: 'Ultra'
    };
    return (number in pairs) ? pairs[number] : '';
  }

  #textToGtaoLevel(text) {
    const pairs = {
      'Low': 1,
      'Medium': 2,
      'High': 3,
      'Ultra': 4
    };
    return (text in pairs) ? pairs[text] : 4;
  }

  #resetViewpoint() {
    WbWorld.instance.viewpoint.resetViewpoint();
    this.#view.x3dScene.render(); // render once to visually reset immediatly the viewpoint.
  }

  #createFullscreenButtons() {
    this.#fullscreenButton = this.#createToolbarButton('fullscreen', 'Full screen (f)', () => requestFullscreen(this.#view));
    this.toolbarRight.appendChild(this.#fullscreenButton);

    this.#exitFullscreenButton = this.#createToolbarButton('windowed', 'Exit full screen (f)', () => exitFullscreen());
    this.toolbarRight.appendChild(this.#exitFullscreenButton);
    this.#exitFullscreenButton.style.display = 'none';

    this.minWidth += 44;

    document.addEventListener('fullscreenchange', this.fullscreenRef = () =>
      onFullscreenChange(this.#fullscreenButton, this.#exitFullscreenButton));
    document.addEventListener('keydown', this.keydownRefF = _ => this.#fullscrenKeyboardHandler(_));
  }

  #parseMillisecondsIntoReadableTime(milliseconds) {
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

  #fullscrenKeyboardHandler(e) {
    if (e.code === 'KeyF' && e.target.tagName !== 'INPUT')
      this.#fullscreenButton.style.display === 'none' ? exitFullscreen() : requestFullscreen(this.#view);
  }

  #removeFullscreenAnimation(button) {
    button.style.animation = 'none';
  }

  // Animations functions

  #createSlider() {
    if (!Animation.sliderDefined) {
      window.customElements.define('animation-slider', AnimationSlider);
      Animation.sliderDefined = true;
    }
    this.#timeSlider = document.createElement('animation-slider');
    this.#timeSlider.id = 'timeSlider';
    document.addEventListener('sliderchange', this.sliderchangeRef = _ => this.#updateSlider(_));
    this.toolbar.appendChild(this.#timeSlider);
    this.#timeSlider.shadowRoot.getElementById('range').addEventListener('mousemove', this.updateFloatingTimeRef = _ =>
      this.#updateFloatingTimePosition(_));
    this.#timeSlider.shadowRoot.getElementById('range').addEventListener('mouseleave', this.hideFloatingTimeRef = _ =>
      this.#hideFloatingTimePosition(_));
  }

  #updateSlider(event) {
    const animation = this.#view.animation;
    if (event.mouseup && animation) {
      if (animation.previousState === 'real-time' && animation.gui === 'pause') {
        animation.previousState = undefined;
        this.#triggerPlayPauseButton();
      }
      return;
    }

    const value = event.detail;

    if (animation.gui === 'real-time') {
      animation.previousState = 'real-time';
      this.#triggerPlayPauseButton();
    }

    const clampedValued = Math.min(value, 99); // set maximum value to get valid step index
    const requestedStep = Math.floor(animation.data.frames.length * clampedValued / 100);
    animation.start = (new Date().getTime()) - Math.floor(animation.data.basicTimeStep * animation.step);
    animation.updateAnimationState(requestedStep);

    this.#timeSlider.setTime(this.#formatTime(animation.data.frames[requestedStep].time));
  }

  #updateFloatingTimePosition(e) {
    this.#timeSlider.shadowRoot.getElementById('floating-time').style.visibility = 'visible';

    const bounds = this.#timeSlider.shadowRoot.getElementById('range').getBoundingClientRect();
    let x = (e.clientX - bounds.left) / (bounds.right - bounds.left) * 100;
    if (x > 100)
      x = 100;
    else if (x < 0)
      x = 0;

    const clampedValued = Math.min(x, 99); // set maximum value to get valid step index
    const requestedStep = Math.floor(this.#view.animation.data.frames.length * clampedValued / 100);
    this.#timeSlider.setTime(this.#formatTime(this.#view.animation.data.frames[requestedStep].time));

    this.#timeSlider.setFloatingTimePosition(e.clientX);
  }

  #hideFloatingTimePosition() {
    this.#timeSlider.shadowRoot.getElementById('floating-time').style.visibility = '';
  }

  #createAnimationTimeIndicator() {
    this.#currentTime = document.createElement('span');
    this.#currentTime.className = 'current-time';
    this.#currentTime.id = 'currentTime';
    this.#currentTime.innerHTML = this.#formatTime(this.#view.animation.data.frames[0].time);
    this.toolbarLeft.appendChild(this.#currentTime);

    const timeDivider = document.createElement('span');
    timeDivider.innerHTML = '/';
    timeDivider.className = 'time-divider';
    this.toolbarLeft.appendChild(timeDivider);

    const totalTime = document.createElement('span');
    totalTime.className = 'total-time';
    const time = this.#formatTime(this.#view.animation.data.frames[this.#view.animation.data.frames.length - 1].time);
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

    this.#timeSlider?.setOffset(offset);

    this.minWidth += 133;
  }

  #createCustomWindowButton() {
    this.customWindowButton = this.#createToolbarButton('custom-window', 'Custom window', undefined);
    this.toolbarRight.appendChild(this.customWindowButton);
    this.#createCustomWindow();
    if (!this.parentNode.showCustomWindow)
      this.customWindowButton.style.display = 'none';
    else
      this.minWidth += 44;
  }

  #createCustomWindow() {
    this.customWindow = new FloatingCustomWindow(this.parentNode);

    const customWindowWidth = 0.6 * this.parentNode.offsetWidth;
    const customWindowHeight = 0.75 * this.parentNode.offsetHeight;
    const customWindowPositionX = (this.parentNode.offsetWidth - customWindowWidth) / 2;
    const customWindowPositionY = (this.parentNode.offsetHeight - customWindowHeight) / 2;

    this.customWindow.floatingWindow.addEventListener('mouseover', () => this.showToolbar());
    this.customWindow.headerQuit.addEventListener('mouseup',
      _ => this.#changeFloatingWindowVisibility(this.customWindow.getId()));

    this.customWindow.setSize(customWindowWidth, customWindowHeight);
    this.customWindow.setPosition(customWindowPositionX, customWindowPositionY);
    this.customWindowButton.onclick = () => this.#changeFloatingWindowVisibility(this.customWindow.getId());

    this.#checkWindowBoundaries();
  }

  #formatTime(time) {
    if (typeof this.#unusedPrefix === 'undefined') {
      const maxTime = this.#view.animation.data.frames[this.#view.animation.data.frames.length - 1].time;
      if (maxTime < 60000)
        this.#unusedPrefix = 6;
      else if (maxTime < 600000)
        this.#unusedPrefix = 4;
      else if (maxTime < 3600000)
        this.#unusedPrefix = 3;
      else if (maxTime < 36000000)
        this.#unusedPrefix = 1;
    }

    return this.#parseMillisecondsIntoReadableTime(time).substring(this.#unusedPrefix);
  }

  #createChangeSpeed() {
    const playbackLi = document.createElement('li');
    playbackLi.id = 'playback-li';
    this.settingsList.appendChild(playbackLi);
    playbackLi.onclick = () => this.#openSpeedPane();

    let label = document.createElement('span');
    label.innerHTML = 'Playback Speed';
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

    this.#createSpeedPane();
  }

  #createSpeedPane() {
    this.#speedPane = document.createElement('div');
    this.#speedPane.className = 'settings-pane';
    this.#speedPane.id = 'speed-pane';
    this.#speedPane.style.visibility = 'hidden';

    const speedList = document.createElement('ul');
    this.#speedPane.appendChild(speedList);
    this.parentNode.appendChild(this.#speedPane);

    let playbackLi = document.createElement('li');
    playbackLi.className = 'first-li';

    let label = document.createElement('div');
    label.className = 'arrow-left';
    playbackLi.appendChild(label);

    label = document.createElement('span');
    label.innerHTML = 'Playback Speed';
    label.className = 'setting-span';
    playbackLi.appendChild(label);

    label = document.createElement('div');
    label.className = 'spacer';
    playbackLi.appendChild(label);
    playbackLi.onclick = () => this.#closeSpeedPane();
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
      playbackLi.onclick = _ => this.#changeSpeed(_);
      speedList.appendChild(playbackLi);
    }
  }

  #changeSpeed(event) {
    const animation = this.#view.animation;
    if (animation) {
      animation.speed = event.srcElement.id;
      this.#speedPane.style.visibility = 'hidden';
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

  #openSpeedPane() {
    this.settingsPane.style.visibility = 'hidden';
    this.#speedPane.style.visibility = 'visible';
  }

  #closeSpeedPane() {
    this.settingsPane.style.visibility = 'visible';
    this.#speedPane.style.visibility = 'hidden';
  }

  removeAnimationToolbar() {
    document.removeEventListener('sliderchange', this.sliderchangeRef);
    this.sliderchangeRef = undefined;

    if (typeof this.#timeSlider !== 'undefined') {
      this.#timeSlider.shadowRoot.getElementById('range').removeEventListener('mousemove', this.updateFloatingTimeRef);
      this.updateFloatingTimeRef = undefined;
      this.#timeSlider.shadowRoot.getElementById('range').removeEventListener('mouseleave', this.hideFloatingTimeRef);
      this.hideFloatingTimeRef = undefined;
      this.#timeSlider.removeEventListeners();
      this.#timeSlider = undefined;
    }

    if (typeof this.#speedPane !== 'undefined') {
      this.parentNode.removeChild(this.#speedPane);
      this.#speedPane = undefined;
    }

    this.removeStreamingToolbar();
  }

  // Scene functions

  _createRestoreViewpointButton() {
    this.toolbarRight.appendChild(this.#createToolbarButton('reset-scene', 'Reset the Scene', () => this.#resetViewpoint()));
    this.minWidth += 44;
  }

  // Streaming functions

  #createQuitButton() {
    const quitButton = this.#createToolbarButton('quit', 'Quit the simulation', () => this.#view.quitSimulation());
    if (!(typeof this.parentNode.showQuit === 'undefined' || this.parentNode.showQuit))
      quitButton.style.display = 'none';
    else
      this.minWidth += 44;

    this.toolbarLeft.appendChild(quitButton);
  }

  #createReloadButton() {
    const reloadButton = this.#createToolbarButton('reload', 'Reload the simulation', () => { this.reset(true); });
    if (!this.parentNode.showReload)
      reloadButton.style.display = 'none';
    else
      this.minWidth += 44;
    this.toolbarLeft.appendChild(reloadButton);
  }

  reset(reload = false) {
    if (this.#view.broadcast)
      return;
    if (reload)
      this.#view.progress.setProgressBar('block', 'Reloading simulation...');
    else
      this.#view.progress.setProgressBar('block', 'Restarting simulation...');

    if (typeof this.pauseButton !== 'undefined' && this.playButtonElement.className === 'icon-pause')
      this.#view.currentState = 'real-time';
    else if (typeof this.runButton !== 'undefined' && this.runButtonElement.className === 'icon-pause')
      this.#view.currentState = 'run';

    const state = this.#view.currentState;
    this.pause();
    this.#view.currentState = state;

    this.hideToolbar(true);
    let previousOnReady = this.#view.onready;
    this.#view.onready = () => {
      if (typeof previousOnReady === 'function')
        previousOnReady();

      switch (this.#view.currentState) {
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
      this.#view.stream.socket.send('reload');
    else
      this.#view.stream.socket.send('reset');

    if (this.robotWindows)
      this.#refreshRobotWindowContent();
  }

  pause() {
    if (this.#view.broadcast)
      return;
    this.#view.stream.socket.send('pause');
    this.#view.currentState = 'pause';
  }

  realTime(force) {
    if (this.#view.broadcast)
      return;
    this.#view.stream.socket.send('real-time:' + this.#view.timeout);
    this.#view.currentState = 'real-time';
  }

  #createStreamingTimeIndicator() {
    const clock = document.createElement('span');
    clock.className = 'webots-streaming-time';

    clock.id = 'webots-clock';
    clock.title = 'Current simulation time';
    clock.innerHTML = this.#parseMillisecondsIntoReadableTime(0);
    this.toolbarLeft.appendChild(clock);
    this.minWidth += 125;
  }

  #createResetButton() {
    let resetButton = this.#createToolbarButton('reset', 'Reset the simulation', () => this.reset(false));
    this.toolbarLeft.appendChild(resetButton);
    if (!(typeof this.parentNode.showReset === 'undefined' || this.parentNode.showReset))
      resetButton.style.display = 'none';
    else
      this.minWidth += 44;
  }

  #createStepButton() {
    let stepButton = this.#createToolbarButton('step', 'Perform one simulation step', () => this.step());
    this.toolbarLeft.appendChild(stepButton);
    if (!(typeof this.parentNode.showStep === 'undefined' || this.parentNode.showStep))
      stepButton.style.display = 'none';
    else
      this.minWidth += 32;
  }

  step() {
    if (this.#view.broadcast)
      return;

    if (typeof this.playButton !== 'undefined') {
      this.playButtonElement.title = 'Play (k)';
      this.playButtonElement.className = 'icon-play';
    }

    if (typeof this.runButton !== 'undefined') {
      this.runButtonElement.title = 'Run';
      this.runButtonElement.className = 'icon-run';
    }

    this.pause();

    this.#view.stream.socket.send('step');
  }

  #createRunButton() {
    this.runButton = this.#createToolbarButton('run', 'Run', () => this.#triggerRunPauseButton());
    this.runButtonElement = this.runButton.childNodes[0];
    this.runButtonElement.title = this.runButton.childNodes[1];
    if (!this.parentNode.showRun)
      this.runButton.style.display = 'none';
    else
      this.minWidth += 44;
    if (this.#view.currentState === 'run' || this.#view.currentState === 'fast') {
      this.runButtonElement.title = 'Pause';
      this.runButtonElement.className = 'icon-pause';
      this.run();
    }
    this.toolbarLeft.appendChild(this.runButton);

    if (typeof this.#view.stream !== 'undefined')
      this.#view.stream.onrun = () => this.#triggerRunPauseButton();
  }

  #triggerRunPauseButton() {
    let action;
    if (this.type === 'streaming') {
      if (this.#view.currentState === 'run' || this.#view.currentState === 'fast') {
        this.pause();
        action = 'run';
      } else {
        this.run();
        action = 'pause';
      }
    }
    if (typeof this.playButton !== 'undefined') {
      this.playButtonElement.title = 'Play (k)';
      this.playButtonElement.className = 'icon-play';
    }

    this.runButtonElement.title = action.charAt(0).toUpperCase() + action.slice(1);
    this.runButtonElement.className = 'icon-' + action;
  }

  run() {
    if (this.#view.broadcast)
      return;
    this.#view.stream.socket.send('fast:' + this.#view.timeout);
    this.#view.currentState = 'fast';
  }

  #createWorldSelectionButton() {
    this.worldSelectionButton = this.#createToolbarButton('world-selection', 'Select world');
    this.toolbarLeft.appendChild(this.worldSelectionButton);
    this.createWorldSelectionPane();
    this.worldSelectionButton.addEventListener('mouseup', _ => this.#changeWorldSelectionPaneVisibility(_));
    window.addEventListener('click', _ => this.#closeWorldSelectionPaneOnClick(_));

    if (!(typeof this.parentNode.showWorldSelection === 'undefined' || this.parentNode.showWorldSelection) ||
      this.#view.worlds.length <= 1)
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

    const worlds = this.#view.worlds;
    for (let i in worlds) {
      this.#addWorldToPane(worlds[i]);
      if (this.#view.currentWorld === worlds[i])
        document.getElementById(worlds[i] + '-button').checked = true;
    }
  }

  #addWorldToPane(name) {
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
      this.#changeWorld(name);
    };
  }

  #changeWorld(name) {
    if (this.#view.currentWorld === name)
      return;

    this.worldSelectionPane.style.visibility = 'hidden';
    document.getElementById(this.#view.currentWorld + '-button').checked = false;
    document.getElementById(name + '-button').checked = true;

    if (this.#view.broadcast || typeof name === 'undefined')
      return;
    this.#view.progress.setProgressBar('block', 'Loading ' + name + '...');
    this.hideToolbar(true);
    let previousOnready = this.#view.onready;
    let stateBeforeChange = this.#view.currentState;
    this.#view.onready = () => {
      if (previousOnready === 'function')
        previousOnready();
      this.showToolbar(true);
      if (stateBeforeChange === 'real-time')
        this.realTime();
      else if (stateBeforeChange === 'fast' || stateBeforeChange === 'run')
        this.run();
    };
    this.#view.stream.socket.send('load:' + name);
  }

  #changeWorldSelectionPaneVisibility(event) {
    if (this.worldSelectionPane.style.visibility === 'hidden')
      this.worldSelectionPane.style.visibility = 'visible';
    else
      this.worldSelectionPane.style.visibility = 'hidden';
  }

  #closeWorldSelectionPaneOnClick(event) {
    if (event.srcElement.id !== 'world-selection-button' && this.worldSelectionPane.style.visibility === 'visible') {
      if (!event.srcElement.id.startsWith('enable-world'))
        this.#changeWorldSelectionPaneVisibility(event);
    }
  }

  removeStreamingToolbar() {
    document.removeEventListener('keydown', this.keydownRefK);
    this.keydownRefK = undefined;
    document.removeEventListener('mouseup', this.settingsRef);
    this.settingsRef = undefined;

    if (typeof this.#gtaoPane !== 'undefined') {
      this.parentNode.removeChild(this.#gtaoPane);
      this.#gtaoPane = undefined;
    }

    if (typeof this.settingsPane !== 'undefined') {
      this.parentNode.removeChild(this.settingsPane);
      this.settingsPane = undefined;
    }

    this.removeToolbar();
  }

  // Proto function

  #createProtoParameterButton() {
    this.protoParameterButton = this.#createToolbarButton('proto-parameter', 'Proto parameter window', undefined);
    this.toolbarRight.appendChild(this.protoParameterButton);
    this.#createProtoParameterWindow();
    this.minWidth += 44;
  }

  #createProtoParameterWindow() {
    this.protoParameterWindow = new FloatingProtoParameterWindow(this.parentNode, this.parentNode.protoManager, this.#view);

    this.protoParameterWindow.floatingWindow.addEventListener('mouseover', () => this.showToolbar());
    this.protoParameterWindow.headerQuit.addEventListener('mouseup',
      _ => this.#changeFloatingWindowVisibility(this.protoParameterWindow.getId()));

    this.protoParameterButton.onclick = () => this.#changeFloatingWindowVisibility(this.protoParameterWindow.getId());
    this.protoParameterWindow.populateProtoParameterWindow();

    this.protoParameterWindow.updateDevicesTabs();

    this.protoParameterWindowInitializeSizeAndPosition();
    this.#checkWindowBoundaries();
    this.protoParameterWindow.setVisibility('visible');
  }

  protoParameterWindowInitializeSizeAndPosition() {
    const protoParameterWindowWidth = 440;
    const protoParameterWindowHeight = this.parentNode.offsetHeight - 40;
    const protoParameterWindowPositionX = (this.parentNode.offsetWidth - protoParameterWindowWidth);
    const protoParameterWindowPositionY = (this.parentNode.offsetHeight - 40 - protoParameterWindowHeight) / 2;
    this.protoParameterWindow.setPosition(protoParameterWindowPositionX, protoParameterWindowPositionY);

    this.protoParameterWindow.setSize(protoParameterWindowWidth, protoParameterWindowHeight);
  }
}
