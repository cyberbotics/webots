/*
 * Injects a Webots 3D view inside a HTML tag.
 * @class
 * @classdesc
 *   The Webots view object displays a 3D view on a web page.
 *   This view represents a Webots simulation world that may be
 *   connected to a webots instance running on a remote server.
 * @example
 *   // Example: Initialize from a Webots streaming server
 *   var view = new webots.View(document.getElementById("myDiv"));
 *   view.open("ws://localhost:80/simple/worlds/simple.wbt");
 *   // or view.open("ws://localhost:80");
 *   // or view.open("file.x3d");
 *   view.onready = () => {
 *       // the initialization is done
 *   }
 *   view.onclose = () => {
 *       view = null;
 *   }
 */

/* global webots */
/* global Animation, Console, ContextMenu, Editor, MouseEvents, DefaultUrl, RobotWindow, TextureLoader */
/* global Server, Stream, SystemInfo, Toolbar, MultimediaClient, X3dScene */
/* global MathJax: false */
/* eslint no-eval: "off" */

/* The following member variables should be set by the application:

webots.User1Id             // ID of the main user (integer value > 0). If 0 or unset, the user is not logged in.
webots.User1Name           // user name of the main user.
webots.User1Authentication // password hash or authentication for the main user (empty or unset if user not authenticated).
webots.User2Id             // ID of the secondary user (in case of a soccer match between two different users). 0 or unset if not used.
webots.User2Name           // user name of the secondary user.
webots.CustomData          // application specific data to be passed to the simulation server
webots.showRevert          // defines whether the revert button should be displayed
webots.showQuit            // defines whether the quit button should be displayed
webots.showRun             // defines whether the run button should be displayed
*/

webots.View = class View {
  constructor(view3D, mobile) {
    webots.currentView = this;
    this.onerror = (text) => {
      console.log('%c' + text, 'color:black');
      this.onrobotwindowsdestroy();
    };
    this.onstdout = (text) => {
      console.log('%c' + text, 'color:blue');
    };
    this.onstderr = (text) => {
      console.log('%c' + text, 'color:red');
    };
    this.onrobotmessage = (robot, message) => {
      if (typeof this.robotWindowNames[robot] === 'undefined') {
        console.log("Robot '" + robot + "' has no associated robot window");
        return;
      }
      this.robotWindows[this.robotWindowNames[robot]].receive(message, robot);
    };
    this.onrobotwindowsdestroy = () => {
      this.robotWindowsGeometries = {};
      for (let win in this.robotWindows) {
        this.robotWindowsGeometries[win] = this.robotWindows[win].geometry();
        this.robotWindows[win].destroy();
      }
      this.infoWindow = undefined;
      this.robotWindows = {}; // delete robot windows
      this.robotWindowNames = {};
    };
    this.onquit = () => { // You can change this behavior by overriding this onquit() method
      window.history.back(); // go back to the previous page in the navigation history
    };
    this.onresize = () => {
      if (typeof this.x3dScene !== 'undefined') {
        // Sometimes the page is not fully loaded by that point and the field of view is not yet available.
        // In that case we add a callback at the end of the queue to try again when all other callbacks are finished.
        if (this.x3dScene.root === null) {
          setTimeout(this.onresize, 0);
          return;
        }
        this.x3dScene.resize();
      } else if (typeof this.multimediaClient !== 'undefined')
        this.multimediaClient.requestNewSize();
    };
    this.ondialogwindow = (opening) => {
      // Pause the simulation if needed when a pop-up dialog window is open
      // and restart running the simulation when it is closed.
      if (opening && typeof this.isAutomaticallyPaused === 'undefined') {
        this.isAutomaticallyPaused = this.toolBar && this.toolBar.pauseButton && this.toolBar.pauseButton.style.display ===
          'inline';
        this.toolBar.pauseButton.click();
      } else if (!opening && this.isAutomaticallyPaused) {
        this.toolBar.real_timeButton.click();
        this.isAutomaticallyPaused = undefined;
      }
    };
    window.onresize = this.onresize;

    // Map robot name to robot window name used as key in robotWindows lists.
    this.robotWindowNames = {};
    this.robotWindows = {};

    this.view3D = view3D;
    this.view3D.className = view3D.className + ' webotsView';

    if (typeof mobile === 'undefined')
      this.mobileDevice = SystemInfo.isMobileDevice();
    else
      this.mobileDevice = mobile;

    this.fullscreenEnabled = !SystemInfo.isIOS();
    if (!this.fullscreenEnabled)
      // Add tag needed to run standalone web page in fullscreen on iOS.
      $('head').append('<meta name="apple-mobile-web-app-capable" content="yes">');

    // Prevent the backspace key to quit the simulation page.
    var rx = /INPUT|SELECT|TEXTAREA/i;
    $(document).bind('keydown keypress', (e) => {
      if (e.which === 8) { // backspace key
        if (!rx.test(e.target.tagName) || e.target.disabled || e.target.readOnly)
          e.preventDefault();
      }
    });

    this.debug = false;
    this.timeout = 60 * 1000; // default to one minute
    this.time = undefined;
    this.deadline = this.timeout;
    this.runOnLoad = false;
    this.quitting = false;
  }

  setTimeout(timeout) { // expressed in seconds
    if (timeout < 0) {
      this.timeout = timeout;
      this.deadline = 0;
      return;
    }

    this.timeout = timeout * 1000; // convert to millisecons
    this.deadline = this.timeout;
    if (typeof this.time !== 'undefined')
      this.deadline += this.time;
  }

  setWebotsDocUrl(url) {
    webots.webotsDocUrl = url;
  }

  setAnimation(url, gui, loop) {
    if (typeof gui === 'undefined')
      gui = 'play';
    if (typeof loop === 'undefined')
      loop = true;
    this.animation = new Animation(url, this.x3dScene, this, gui, loop);
  }

  open(url, mode, texturePathPrefix = '') {
    this.url = url;
    if (typeof mode === 'undefined')
      mode = 'x3d';
    this.mode = mode;

    var initWorld = () => {
      function findGetParameter(parameterName) {
        let tmp = [];
        let items = window.location.search.substr(1).split('&');
        for (let index = 0; index < items.length; index++) {
          tmp = items[index].split('=');
          if (tmp[0] === parameterName)
            return decodeURIComponent(tmp[1]);
        }
        return null;
      }
      if (this.isWebSocketProtocol) {
        this.progress = document.createElement('div');
        this.progress.id = 'webotsProgress';
        this.progress.innerHTML = "<div><img src='" + DefaultUrl.wwiImagesUrl() + "load_animation.gif'>" +
          "</div><div id='webotsProgressMessage'>Initializing...</div>" +
          "</div><div id='webotsProgressPercent'></div>";
        this.view3D.appendChild(this.progress);

        if (typeof this.toolBar === 'undefined')
          this.toolBar = new Toolbar(this.view3D, this);

        const url = findGetParameter('url');
        if (url || this.url.endsWith('.wbt')) { // url expected form: "wss://localhost:1999/simple/worlds/simple.wbt" or
          // "wss://localhost/1999/?url=webots://github.com/cyberbotics/webots/branch/master/projects/languages/python"
          this.server = new Server(this.url, this, finalizeWorld);
          this.server.connect();
        } else { // url expected form: "ws://cyberbotics1.epfl.ch:80"
          const httpServerUrl = 'http' + this.url.slice(2); // replace 'ws'/'wss' with 'http'/'https'
          this.stream = new Stream(this.url, this, finalizeWorld);
          TextureLoader.setTexturePathPrefix(httpServerUrl + '/');
          this.stream.connect();
        }
      } else // assuming it's an URL to a .x3d file
        this.x3dScene.loadWorldFile(this.url, finalizeWorld);
    };

    var finalizeWorld = () => {
      $('#webotsProgressMessage').html('Loading HTML and JavaScript files...');
      if (typeof this.x3dScene !== 'undefined') {
        if (this.x3dScene.viewpoint.followedObjectId == null || this.broadcast)
          this.x3dScene.viewpoint.initFollowParameters();
        else
          // Reset follow parameters.
          this.x3dScene.viewpoint.follow(this.x3dScene.viewpoint.followedObjectId);

        if (!this.isWebSocketProtocol) { // skip robot windows initialization
          if (this.animation != null)
            this.animation.init(loadFinalize);
          else
            loadFinalize();
          this.onresize();
          return;
        }
      }

      var loadRobotWindow = (windowName, nodeName) => {
        this.robotWindowNames[nodeName] = windowName;
        let win = new RobotWindow(this.view3D, this.mobileDevice, windowName);
        this.robotWindows[windowName] = win;
        // Initialize robot windows dialogs.
        function closeInfoWindow() {
          $('#infoButton').removeClass('toolBarButtonActive');
        }
        if (infoWindowName && windowName === infoWindowName) {
          var user;
          if (typeof webots.User1Id !== 'undefined' && webots.User1Id !== '') {
            user = ' [' + webots.User1Name;
            if (typeof webots.User2Id !== 'undefined' && webots.User2Id !== '')
              user += '/' + webots.User2Name;
            user += ']';
          } else
            user = '';
          let worldInfoTitle;
          if (typeof this.x3dScene !== 'undefined')
            worldInfoTitle = this.x3dScene.worldInfo.title;
          else
            worldInfoTitle = this.multimediaClient.worldInfo.title;
          win.setProperties({
            title: worldInfoTitle + user,
            close: closeInfoWindow
          });
          this.infoWindow = win;
        } else {
          win.setProperties({
            title: 'Robot: ' + nodeName
          });
        }
        pendingRequestsCount++;
        const baseUrl = (this.server ? this.server.httpServerUrl : 'http' + this.url.slice(2)) + '/';
        const url = baseUrl + 'robot_windows/' + windowName + '/' + windowName + '.html ';
        $.get(url, (data) => {
          function fixSrc(collection, serverUrl) {
            for (let i = 0; i < collection.length; i++) {
              if (collection[i].src) {
                const src = collection[i].getAttribute('src');
                let url, newSrc;
                try {
                  url = new URL(src);
                  newSrc = serverUrl.href + 'robot_windows/' + windowName + url.pathname;
                } catch (_) {
                  url = null;
                  newSrc = serverUrl.href + 'robot_windows/' + windowName + '/' + src;
                }
                if (url === null || url.origin === serverUrl.origin) {
                  if (collection[i].tagName === 'SCRIPT') {
                    collection[i].removeAttribute('src');
                    collection[i].setAttribute('disabled-src', newSrc);
                  } else // IMG
                    collection[i].setAttribute('src', newSrc);
                } else {
                  collection[i].setAttribute('disabled-src', src);
                  collection[i].removeAttribute('src');
                }
              }
            }
          }
          let parser = new DOMParser();
          let doc = parser.parseFromString(data, 'text/html');
          let serverUrl = new URL(baseUrl);
          fixSrc(doc.getElementsByTagName('script'), serverUrl);
          fixSrc(doc.getElementsByTagName('img'), serverUrl);
          let links = doc.getElementsByTagName('link');
          for (let i = 0; i < links.length; i++) {
            if (links[i].rel === 'stylesheet' && links[i].type === 'text/css') {
              let link = document.createElement('link');
              for (let j = links[i].attributes.length - 1; j >= 0; j--) {
                const name = links[i].attributes[j].name;
                let value = links[i].attributes[j].value;
                if (name === 'href' && !value.startsWith('https://') && !value.startsWith('http://')) // local css file
                  value = baseUrl + 'robot_windows/' + windowName + '/' + value;
                link.setAttribute(name, value);
              }
              document.getElementsByTagName('head')[0].appendChild(link);
            }
          }
          const body = doc.getElementsByTagName('body')[0];
          win.setContent(body.innerHTML);
          MathJax.Hub.Queue(['Typeset', MathJax.Hub, win[0]]);

          function nodeScriptClone(node) { // this forces the execution of the Javascript code
            let script = document.createElement('script');
            script.text = node.innerHTML;
            for (let i = node.attributes.length - 1; i >= 0; i--) {
              const name = node.attributes[i].name;
              const value = node.attributes[i].value;
              if (name === 'disabled-src') {
                script.setAttribute('src', value);
                continue;
              }
              script.setAttribute(name, value);
            }
            return script;
          }

          function nodeScriptReplace(node) {
            if (node.tagName === 'SCRIPT')
              node.parentNode.replaceChild(nodeScriptClone(node), node);
            else {
              let i = 0;
              let children = node.childNodes;
              while (i < children.length)
                nodeScriptReplace(children[i++]);
            }
          }
          nodeScriptReplace(win.panel); // execute Javascript code if any script tag
          pendingRequestsCount--;
          if (pendingRequestsCount === 0)
            loadFinalize();
        }).fail(() => {
          if (windowName === infoWindowName)
            this.infoWindow = undefined;
          pendingRequestsCount--;
          if (pendingRequestsCount === 0)
            loadFinalize();
        });
      };

      let pendingRequestsCount = 1; // start from 1 so that it can be 0 only after the loop is completed and all the nodes are checked
      let windowsDict = [];
      let infoWindowName;
      if (typeof this.x3dScene !== 'undefined') {
        windowsDict = this.x3dScene.getRobotWindows();
        infoWindowName = this.x3dScene.worldInfo.window;
      } else if (this.multimediaClient) {
        windowsDict = this.multimediaClient.robotWindows;
        infoWindowName = this.multimediaClient.worldInfo.infoWindow;
      } else {
        loadFinalize();
        return;
      }

      windowsDict.forEach((window) => {
        // window: [robot name, window name]
        loadRobotWindow(window[1], window[0]);
        if (window[0] === 'worldInfoWindow')
          infoWindowName = window[0];
      });
      pendingRequestsCount--; // notify that loop is completed
      if (pendingRequestsCount === 0)
        // If no pending requests execute loadFinalize
        // otherwise it will be executed when the last request will be handled.
        loadFinalize();
    };

    let loadFinalize = () => {
      $('#webotsProgress').hide();
      if (typeof this.multimediaClient !== 'undefined')
        // finalize multimedia client and set toolbar buttons status
        this.multimediaClient.finalize();
      else if (this.toolBar)
        this.toolBar.enableToolBarButtons(true);

      if (typeof this.onready === 'function')
        this.onready();

      // Restore robot windows.
      if (this.robotWindowsGeometries) { // on reset
        for (let win in this.robotWindows) {
          if (win in this.robotWindowsGeometries) {
            this.robotWindows[win].restoreGeometry(this.robotWindowsGeometries[win]);
            if (this.robotWindowsGeometries[win].open) {
              if (this.robotWindows[win] === this.infoWindow)
                this.toolBar.toggleInfo();
              else
                this.robotWindows[win].open();
            }
          }
        }
      } else if (this.infoWindow && !this.broadcast) // at first load
        this.toolBar.toggleInfo();

      if (this.runOnLoad && this.toolBar)
        this.toolBar.realTime();

      if (typeof this.x3dScene !== 'undefined')
        // Force a rendering after 1 second.
        // This should make sure that all the texture transforms are applied (for example in the Highway Driving benchmark).
        setTimeout(() => this.x3dScene.render(), 1000);
    };

    if (this.broadcast)
      this.setTimeout(-1);
    this.isWebSocketProtocol = this.url.startsWith('ws://') || this.url.startsWith('wss://');

    if (typeof this.contextMenu === 'undefined' && this.isWebSocketProtocol) {
      let authenticatedUser = !this.broadcast;
      if (authenticatedUser && typeof webots.User1Id !== 'undefined' && webots.User1Id !== '')
        authenticatedUser = Boolean(webots.User1Authentication);
      this.contextMenu = new ContextMenu(authenticatedUser, this.view3D);
      this.contextMenu.onEditController = (controller) => {
        this.editController(controller);
      };
      this.contextMenu.onOpenRobotWindow = (robotName) => {
        this.openRobotWindow(robotName);
      };
      this.contextMenu.isRobotWindowValid = (robotName, setResult) => {
        setResult(this.robotWindows[this.robotWindowNames[robotName]]);
      };
    }

    if (mode === 'mjpeg') {
      this.url = url;
      this.multimediaClient = new MultimediaClient(this, this.view3D, this.contextMenu);
      this.contextMenu.onFollowObject = (id, mode) => {
        this.multimediaClient.setFollowed(id, mode);
      };
    } else if (typeof this.x3dScene === 'undefined') {
      this.x3dDiv = document.createElement('div');
      this.x3dDiv.className = 'webots3DView';
      this.view3D.appendChild(this.x3dDiv);
      this.x3dScene = new X3dScene(this.x3dDiv);
      this.x3dScene.init(texturePathPrefix);
      let param = document.createElement('param');
      param.name = 'showProgress';
      param.value = false;
      this.x3dScene.domElement.appendChild(param);
    }

    if (typeof this.x3dScene !== 'undefined' && typeof this.mouseEvents === 'undefined')
      this.mouseEvents = new MouseEvents(this.x3dScene, this.contextMenu, this.x3dDiv, this.mobileDevice);

    if (typeof this.console === 'undefined')
      this.console = new Console(this.view3D, this.mobileDevice);

    if (typeof this.editor === 'undefined')
      this.editor = new Editor(this.view3D, this.mobileDevice, this);

    initWorld();
  }

  close() {
    if (this.multimediaClient)
      this.multimediaClient.disconnect();
    if (this.server && this.server.socket)
      this.server.socket.close();
    if (this.stream)
      this.stream.close();
  }

  sendRobotMessage(message, robot) {
    this.stream.socket.send('robot:' + robot + ':' + message);
    if (this.toolBar.isPaused()) // if paused, make a simulation step
      webots.currentView.stream.socket.send('step'); // so that the robot controller handles the message
    // FIXME: there seems to be a bug here: after that step, the current time is not incremented in the web interface,
    // this is because the next 'application/json:' is not received, probably because it gets overwritten by the
    // answer to the robot message...
  }

  getControllerUrl(name) {
    if (!this.server)
      return;
    var port = 0;
    for (let i in this.server.controllers) {
      if (this.server.controllers[i].name === name) {
        port = this.server.controllers[i].port;
        break;
      }
    }
    if (port === 0)
      return;
    return this.url.substring(0, this.url.indexOf(':', 6) + 1) + port;
  }

  // Functions for internal use.

  updateWorldList(currentWorld, worlds) {
    if (!this.toolBar || this.broadcast)
      // Do not show world list if no toolbar exists or in broadcast mode,
      // where multiple users can connect to the same Webots instance.
      return;

    if (typeof this.toolBar.worldSelect !== 'undefined')
      this.toolBar.deleteWorldSelect();
    if (worlds.length <= 1)
      return;
    this.toolBar.createWorldSelect();
    for (let i in worlds) {
      var option = document.createElement('option');
      option.value = worlds[i];
      option.text = worlds[i];
      this.toolBar.worldSelect.appendChild(option);
      if (currentWorld === worlds[i])
        this.toolBar.worldSelect.selectedIndex = i;
    }
    this.toolBar.worldSelect.onchange = () => {
      if (this.broadcast || typeof this.toolBar.worldSelect === 'undefined')
        return;
      if (this.toolBar)
        this.toolBar.enableToolBarButtons(false);
      if (typeof this.x3dScene !== 'undefined')
        this.x3dScene.viewpoint.resetFollow();
      this.onrobotwindowsdestroy();
      $('#webotsProgressMessage').html('Loading ' + this.toolBar.worldSelect.value + '...');
      $('#webotsProgress').show();
      this.stream.socket.send('load:' + this.toolBar.worldSelect.value);
    };
  }

  setLabel(properties) {
    var labelElement = document.getElementById('label' + properties.id);
    if (labelElement == null) {
      labelElement = document.createElement('div');
      labelElement.id = 'label' + properties.id;
      labelElement.className = 'webotsLabel';
      this.x3dDiv.appendChild(labelElement);
    }
    labelElement.style.fontFamily = properties.font;
    labelElement.style.color = properties.color;
    labelElement.style.fontSize = $(this.x3dDiv).height() * properties.size / 2.25 + 'px'; // 2.25 is an empirical value to match with Webots appearance
    labelElement.style.left = $(this.x3dDiv).width() * properties.x + 'px';
    labelElement.style.top = $(this.x3dDiv).height() * properties.y + 'px';
    labelElement.innerHTML = properties.text;
  }

  removeLabels() {
    var labels = document.getElementsByClassName('webotsLabel');
    for (let i = labels.length - 1; i >= 0; i--) {
      var element = labels.item(i);
      element.parentNode.removeChild(element);
    }
  }

  resetSimulation() {
    this.removeLabels();
    $('#webotsClock').html(webots.parseMillisecondsIntoReadableTime(0));
    this.deadline = this.timeout;
    if (this.deadline >= 0)
      $('#webotsTimeout').html(webots.parseMillisecondsIntoReadableTime(this.deadline));
    else
      $('#webotsTimeout').html(webots.parseMillisecondsIntoReadableTime(0));
    if (typeof this.x3dScene !== 'undefined')
      this.x3dScene.viewpoint.reset(this.time);
  }

  quitSimulation() {
    if (this.broadcast)
      return;
    this.close();
    $('#webotsProgressMessage').html('Bye bye...');
    $('#webotsProgress').show();
    this.quitting = true;
    this.onquit();
  }

  destroyWorld() {
    if (typeof this.x3dScene !== 'undefined')
      this.x3dScene.destroyWorld();
    this.removeLabels();
    this.onrobotwindowsdestroy();
  }

  editController(controller) {
    if (this.editor.dirname !== controller) {
      this.editor.closeAllTabs();
      this.editor.dirname = controller;
      this.stream.socket.send('get controller:' + controller);
    }
  }

  openRobotWindow(robotName) {
    var win = this.robotWindows[this.robotWindowNames[robotName]];
    if (win) {
      if (win === this.infoWindow) {
        if (!this.infoWindow.isOpen())
          this.toolBar.toggleInfo();
      } else
        win.open();
    } else
      console.log('No valid robot window for robot: ' + robotName);
  }
};

webots.window = (name) => {
  var win = webots.currentView.robotWindows[name];
  if (!win)
    console.log("Robot window '" + name + "' not found.");
  return win;
};
