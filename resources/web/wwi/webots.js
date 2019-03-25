/*
 * Injects a Webots 3D view inside a HTML tag.
 * @class
 * @classdesc
 *   The Webots view object displays a 3D view on a web page.
 *   This view represents a Webots simulation world that may be
 *   connected to a webots instance running on a remote server.
 *   This library depends on the x3dom-full.js library // TODO
 * @example
 *   // Example: Initialize from a Webots streaming server
 *   var view = new webots.View(document.getElementById("myDiv"));
 *   view.open("ws://localhost:80/simple/worlds/simple.wbt");
 *   // or view.open("ws://localhost:80");
 *   // or view.open("file.x3d");
 *   view.onready = function() {
 *       // the initialization is done
 *   }
 *   view.onclose = function() {
 *       view = null;
 *   }
 */

/* global webots THREE, Animation, X3dSceneManager, MouseEvents, Server, Stream, ContextMenu, VideoManager, Console, Editor, RobotWindow, Toolbar, ResourceManager */
/* global MathJax: false */
/* eslint no-eval: "off" */

/* The following member variables should be set by the application:

webots.User1Id             // ID of the main user (integer value > 0). If 0 or unset, the user is not logged in.
webots.User1Name           // user name of the main user.
webots.User1Authentication // password or authentication for the main user (empty or unset if user not authenticated).
webots.User2Id             // ID of the secondary user (in case of a soccer match between two different users). 0 or unset if not used.
webots.User2Name           // user name of the secondary user.
webots.CustomData          // application specific data to be passed to the simulation server
webots.showRevert          // defines whether the revert button should be displayed
webots.showQuit            // defines whether the quit button should be displayed

*/

webots.View = function(view3D, mobile) {
  webots.currentView = this;
  var that = this;
  this.onerror = function(text) {
    console.log('%c' + text, 'color:black');
    that.onrobotwindowsdestroy();
  };
  this.onstdout = function(text) {
    console.log('%c' + text, 'color:blue');
  };
  this.onstderr = function(text) {
    console.log('%c' + text, 'color:red');
  };
  this.onrobotmessage = function(robot, message) {
    if (that.robotWindowNames[robot] === undefined) {
      console.log("Robot '" + robot + "' has no associated robot window");
      return;
    }
    that.robotWindows[that.robotWindowNames[robot]].receive(message, robot);
  };
  this.onrobotwindowsdestroy = function() {
    that.robotWindowsGeometries = {};
    for (var win in that.robotWindows) {
      that.robotWindowsGeometries[win] = that.robotWindows[win].geometry();
      that.robotWindows[win].destroy();
    }
    that.infoWindow = null;
    that.robotWindows = {}; // delete robot windows
    that.robotWindowNames = {};
  };
  this.onquit = function() {
    // If the simulation page URL is this https://mydomain.com/mydir/mysimulation.html, the quit action redirects to the
    // folder level, e.g., https://mydomain.com/mydir/
    // If the simulation page is https://mydomain.com/mydir/mysimulation/, the quit action redirects to the upper level:
    // https://mydomain.com/mydir/
    // You can change this behavior by overriding this onquit() method
    var currentLocation = window.location.href;
    // remove filename or last directory name from url and keep the final slash
    var quitDestination = currentLocation.substring(0, currentLocation.lastIndexOf('/', currentLocation.length - 2) + 1);
    window.location = quitDestination;
  };
  this.onresize = function() {
    // Sometimes the page is not fully loaded by that point and the field of view is not yet available.
    // In that case we add a callback at the end of the queue to try again when all other callbacks are finished.
    if (that.x3dSceneManager.root === null) {
      setTimeout(that.onresize, 0);
      return;
    }

    var viewHeight = parseFloat($(that.x3dDiv).css('height').slice(0, -2));
    var viewWidth = parseFloat($(that.x3dDiv).css('width').slice(0, -2));
    that.x3dSceneManager.viewpoint.resetFov(viewWidth, viewHeight);
  };
  this.ondialogwindow = function(opening) {
    // Pause the simulation if needed when a pop-up dialog window is open
    // and restart running the simulation when it is closed.
    if (opening && that.isAutomaticallyPaused === undefined) {
      that.isAutomaticallyPaused = that.toolBar && that.toolBar.pauseButton && that.toolBar.pauseButton.style.display === 'inline';
      that.toolBar.pauseButton.click();
    } else if (!opening && that.isAutomaticallyPaused) {
      that.toolBar.real_timeButton.click();
      that.isAutomaticallyPaused = undefined;
    }
  };
  window.onresize = this.onresize;

  this.onmousedown = null;
  this.onworldloaded = null;

  this.robotWindowNames = {}; // map robot name to robot window name used as key in robotWindows lists
  this.robotWindows = {};

  this.view3D = view3D;
  this.view3D.className = view3D.className + ' webotsView';

  if (mobile === undefined)
    this.mobileDevice = /Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent);
  else
    this.mobileDevice = mobile;

  this.fullscreenEnabled = !/iPhone|iPad|iPop/i.test(navigator.userAgent);
  if (!this.fullscreenEnabled)
    // add tag needed to run standalone web page in fullscreen on iOS
    $('head').append('<meta name="apple-mobile-web-app-capable" content="yes">');

  // prevent the backspace key to quit the simulation page
  var rx = /INPUT|SELECT|TEXTAREA/i;
  $(document).bind('keydown keypress', function(e) {
    if (e.which === 8) { // backspace key
      if (!rx.test(e.target.tagName) || e.target.disabled || e.target.readOnly)
        e.preventDefault();
    }
  });

  this.x3dSceneManager = null;
  this.debug = false;
  this.timeout = 60 * 1000; // default to one minute
  this.time = undefined;
  this.deadline = this.timeout;
  this.runOnLoad = false;
  this.quitting = false;
};

webots.View.prototype.setTimeout = function(timeout) { // expressed in seconds
  if (timeout < 0) {
    this.timeout = timeout;
    this.deadline = 0;
    return;
  }

  this.timeout = timeout * 1000; // convert to millisecons
  this.deadline = this.timeout;
  if (this.time !== undefined)
    this.deadline += this.time;
};

webots.View.prototype.setWebotsDocUrl = function(url) {
  this.webotsDocUrl = url;
};

webots.View.prototype.updateWorldList = function(currentWorld, worlds) {
  if (!this.toolBar)
    return;

  var that = this;
  if (typeof this.worldSelect !== 'undefined')
    this.toolBar.worldSelectionDiv.removeChild(this.worldSelect);
  if (worlds.length <= 1)
    return;
  this.worldSelect = document.createElement('select');
  this.worldSelect.id = 'worldSelection';
  this.toolBar.worldSelectionDiv.appendChild(this.worldSelect);
  for (var i = 0; i < worlds.length; i++) {
    var option = document.createElement('option');
    option.value = worlds[i];
    option.text = worlds[i];
    this.worldSelect.appendChild(option);
    if (currentWorld === worlds[i])
      this.worldSelect.selectedIndex = i;
  }
  this.worldSelect.onchange = loadWorld;
  function loadWorld() {
    if (that.broadcast || typeof that.worldSelect === 'undefined')
      return;
    if (that.toolBar)
      that.toolBar.enableToolBarButtons(false);
    that.x3dSceneManager.viewpoint.resetFollow();
    that.onrobotwindowsdestroy();
    $('#webotsProgressMessage').html('Loading ' + that.worldSelect.value + '...');
    $('#webotsProgress').show();
    that.stream.socket.send('load:' + that.worldSelect.value);
  }
};

webots.View.prototype.open = function(url, mode) {
  var that = this;
  this.url = url;
  if (mode === undefined)
    mode = 'x3dom';
  this.mode = mode;

  if (mode === 'video') {
    this.url = url;
    this.video = new VideoManager(this.view3D, this.mouseEvents);
    initWorld();
    return;
  }
  if (mode !== 'x3dom') {
    console.log('Error: webots.View.open: wrong mode argument: ' + mode);
    return;
  }

  if (this.broadcast)
    this.setTimeout(-1);
  this.isWebSocketProtocol = this.url.startsWith('ws://') || this.url.startsWith('wss://');

  if (!this.x3dSceneManager) {
    this.x3dDiv = document.createElement('div');
    this.x3dDiv.className = 'webots3DView';
    this.view3D.appendChild(this.x3dDiv);
    this.x3dSceneManager = new X3dSceneManager(this.x3dDiv);
    that.x3dSceneManager.init();
    var param = document.createElement('param');
    param.name = 'showProgress';
    param.value = false;
    this.x3dSceneManager.domElement.appendChild(param);
  }

  if (!that.contextMenu) {
    this.contextMenu = new ContextMenu(webots.User1Id && !webots.User1Authentication, this.view3D);
    this.contextMenu.onEditController = function(controller) { that.editController(controller); };
    this.contextMenu.onFollowObject = function(id) { that.x3dSceneManager.viewpoint.follow(id); };
    this.contextMenu.isFollowedObject = function(object3d, setResult) { setResult(that.x3dSceneManager.viewpoint.isFollowedObject(object3d)); };
    this.contextMenu.onOpenRobotWindow = function(robotName) { that.openRobotWindow(robotName); };
    this.contextMenu.isRobotWindowValid = function(robotName, setResult) { setResult(that.robotWindows[that.robotWindowNames[robotName]]); };
  }

  if (!this.mouseEvents)
    this.mouseEvents = new MouseEvents(this.x3dSceneManager, this.contextMenu, this.x3dDiv);

  if (!this.console)
    this.console = new Console(this.view3D, this.mobileDevice);

  if (!this.editor)
    this.editor = new Editor(this.view3D, this.mobileDevice, this);

  // TODO if THREE js library is already loaded the if is useless
  /* if (this.url === undefined)
    //loadTHREEjs(); TODO
  else
    initWorld();
  */
  initWorld();

  function initWorld() {
    if (that.isWebSocketProtocol) {
      var resourceManager = new ResourceManager();
      that.progress = document.createElement('div');
      that.progress.id = 'webotsProgress';
      that.progress.innerHTML = "<div><img src='" + resourceManager.wwiUrl + "images/load_animation.gif'>" +
                                "</div><div id='webotsProgressMessage'>Initializing...</div>" +
                                "</div><div id='webotsProgressPercent'></div>";
      that.view3D.appendChild(that.progress);

      if (!that.toolBar)
        that.toolBar = new Toolbar(that.view3D, that);

      if (that.url.endsWith('.wbt')) { // url expected form: "ws://localhost:80/simple/worlds/simple.wbt"
        var callback;
        if (that.mode === 'video')
          callback = that.video.finalize;
        else
          callback = finalizeWorld;
        that.server = new Server(that.url, that, callback);
        that.server.connect();
      } else { // url expected form: "ws://cyberbotics2.cyberbotics.com:80"
        that.stream = new Stream(that.url, that, finalizeWorld);
        that.stream.connect();
      }
    } else { // assuming it's an URL to a .x3d file
      that.x3dSceneManager.loadWorldFile(that.url);
      finalizeWorld();
    }
  }

  function finalizeWorld() {
    $('#webotsProgressMessage').html('Loading HTML and Javascript files...');
    if (that.x3dSceneManager.viewpoint.followedObjectId == null || that.broadcast)
      that.x3dSceneManager.viewpoint.initFollowParameters();
    else
      // reset follow parameters
      that.x3dSceneManager.viewpoint.follow(that.x3dSceneManager.viewpoint.followedObjectId);

    if (!that.isWebSocketProtocol) { // skip robot windows initialization
      if (that.animation != null)
        that.animation.init(loadFinalize);
      else
        loadFinalize();
      that.onresize();
      return;
    }

    function loadRobotWindow(windowName, nodeName) {
      that.robotWindowNames[nodeName] = windowName;
      var win = new RobotWindow(that.view3D, that.mobileDevice, windowName);
      that.robotWindows[windowName] = win;
      // init robot windows dialogs
      function closeInfoWindow() {
        $('#infoButton').removeClass('toolBarButtonActive');
      }
      if (windowName === infoWindowName) {
        var user;
        if (webots.User1Id) {
          user = ' [' + webots.User1Name;
          if (webots.User2Id)
            user += '/' + webots.User2Name;
          user += ']';
        } else
          user = '';
        win.setProperties({title: that.x3dSceneManager.worldInfo.title + user, close: closeInfoWindow});
        that.infoWindow = win;
      } else
        win.setProperties({title: 'Robot: ' + nodeName});
      pendingRequestsCount++;
      $.get('window/' + windowName + '/' + windowName + '.html', function(data) {
        // we need to fix the img src relative URLs
        var d = data.replace(/ src='/g, ' src=\'window/' + windowName + '/').replace(/ src="/g, ' src="window/' + windowName + '/');
        win.setContent(d);
        MathJax.Hub.Queue(['Typeset', MathJax.Hub, win[0]]);
        $.get('window/' + windowName + '/' + windowName + '.js', function(data) {
          eval(data);
          pendingRequestsCount--;
          if (pendingRequestsCount === 0)
            loadFinalize();
        }).fail(function() {
          pendingRequestsCount--;
          if (pendingRequestsCount === 0)
            loadFinalize();
        });
      }).fail(function() {
        if (windowName === infoWindowName)
          that.infoWindow = null;
        pendingRequestsCount--;
        if (pendingRequestsCount === 0)
          loadFinalize();
      });
    }

    var infoWindowName = that.x3dSceneManager.worldInfo.window;
    var pendingRequestsCount = 1; // start from 1 so that it can be 0 only after the loop is completed and all the nodes are checked
    var nodes = that.x3dSceneManager.root ? that.x3dSceneManager.root.children : [];
    for (var i = 0; i < nodes.length; i++) {
      if (nodes[i] instanceof THREE.Object3D && nodes[i].userData && nodes[i].userData.window && nodes[i].userData.name)
        loadRobotWindow(nodes[i].userData.window, nodes[i].userData.name);
    }
    pendingRequestsCount--; // notify that loop is completed
    if (pendingRequestsCount === 0)
      // if no pending requests execute loadFinalize
      // otherwise it will be executed when the last request will be handled
      loadFinalize();
  }

  function loadFinalize() {
    $('#webotsProgress').hide();
    if (that.toolBar && !that.broadcast)
      that.toolBar.enableToolBarButtons(true);

    if (that.onready)
      that.onready();

    // restore robot windows
    if (that.robotWindowsGeometries) { // on reset
      for (var win in that.robotWindows) {
        if (win in that.robotWindowsGeometries) {
          that.robotWindows[win].restoreGeometry(that.robotWindowsGeometries[win]);
          if (that.robotWindowsGeometries[win].open) {
            if (that.robotWindows[win] === that.infoWindow)
              that.toolBar.toggleInfo();
            else
              that.robotWindows[win].open();
          }
        }
      }
    } else if (that.infoWindow && !that.broadcast) // at first load
      that.toolBar.toggleInfo();

    if (that.runOnLoad && that.toolBar)
      that.toolBar.realTime();
  }
};

webots.View.prototype.close = function() {
  if (this.server)
    this.server.socket.close();
  if (this.stream)
    this.stream.close();
};

webots.View.prototype.sendRobotMessage = function(robot, message) {
  this.stream.socket.send('robot:' + robot + ':' + message);
};

webots.View.prototype.resize = function(width, height) {
  if (this.video)
    this.video.resize(width, height);
};

webots.View.prototype.getControllerUrl = function(name) {
  if (!this.server)
    return;
  var port = 0;
  for (var i = 0; i < this.server.controllers.length; i++) {
    if (this.server.controllers[i].name === name) {
      port = this.server.controllers[i].port;
      break;
    }
  }
  if (port === 0)
    return;
  return this.url.substring(0, this.url.indexOf(':', 6) + 1) + port;
};

webots.View.prototype.setAnimation = function(url, gui, loop) {
  if (gui === undefined)
    gui = 'play';
  if (loop === undefined)
    loop = true;
  var that = this;
  this.animation = new Animation(url, this.x3dSceneManager, this, gui, loop);
  this.animation.init(function() {
    $('#webotsProgress').hide();
    if (that.onready)
      that.onready();
  });
};

webots.View.prototype.setLabel = function(properties) {
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
};

webots.View.prototype.removeLabels = function() {
  // remove labels
  var labels = document.getElementsByClassName('webotsLabel');
  for (var i = labels.length - 1; i >= 0; i--) {
    var element = labels.item(i);
    element.parentNode.removeChild(element);
  }
};

webots.View.prototype.resetSimulation = function() {
  this.removeLabels();
  $('#webotsClock').html(webots.parseMillisecondsIntoReadableTime(0));
  this.deadline = this.timeout;
  if (this.deadline >= 0)
    $('#webotsTimeout').html(webots.parseMillisecondsIntoReadableTime(this.deadline));
  else
    $('#webotsTimeout').html(webots.parseMillisecondsIntoReadableTime(0));
  this.x3dSceneManager.viewpoint.reset(this.time);
};

webots.View.prototype.quitSimulation = function() {
  if (this.broadcast)
    return;
  $('#webotsProgressMessage').html('Bye bye...');
  $('#webotsProgress').show();
  this.quitting = true;
  this.onquit();
};

webots.View.prototype.destroyWorld = function() {
  if (this.x3dSceneManager)
    this.x3dSceneManager.destroyWorld();

  this.removeLabels();
};

webots.View.prototype.editController = function(controller) {
  if (this.editor.dirname !== controller) {
    this.editor.closeAllTabs();
    this.editor.dirname = controller;
    this.stream.socket.send('get controller:' + controller);
  }
};

webots.View.prototype.openRobotWindow = function(robotName) {
  var win = this.robotWindows[this.robotWindowNames[robotName]];
  if (win) {
    if (win === this.infoWindow) {
      if (!this.infoWindow.isOpen())
        this.toolBar.toggleInfo();
    } else
      win.open();
  } else
    console.log('No valid robot window for robot: ' + robotName);
};

webots.window = function(name) {
  var win = webots.currentView.robotWindows[name];
  if (!win)
    console.log("Robot window '" + name + "' not found.");
  return win;
};
