/*
 * Injects a Webots 3D view inside a HTML tag.
 * @class
 * @classdesc
 *   The Webots view object displays a 3D view on a web page.
 *   This view represents a Webots simulation world that may be
 *   connected to a webots instance running on a remote server.
 *   This library depends on the x3dom-full.js library
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

/* global x3dom: false */
/* global ace: false */
/* global MathJax: false */
/* eslint no-extend-native: ["error", { "exceptions": ["String"] }] */
/* eslint no-eval: "off" */

/* The following member variables should be set by the application:

webots.User1Id             // ID of the main user (integer value > 0). If 0 or unset, the user is not logged in.
webots.User1Name           // user name of the main user.
webots.User1Authentication // password or authentication for the main user (empty or unset if user not authenticated).
webots.User2Id             // ID of the secondary user (in case of a soccer match between two different users). 0 or unset if not used.
webots.User2Name           // user name of the secondary user.
webots.CustomData          // application specific data to be passed to the simulation server

*/

var webots = window.webots || {};

var scripts = document.getElementsByTagName('script');
webots.WwiUrl = scripts[scripts.length - 1].src;
webots.WwiUrl = webots.WwiUrl.substr(0, webots.WwiUrl.lastIndexOf('/') + 1); // remove "webots.js"

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
    var viewpoint = that.x3dScene.getElementsByTagName('Viewpoint')[0];
    var viewHeight = parseFloat($(that.x3dNode).css('height').slice(0, -2));
    var viewWidth = parseFloat($(that.x3dNode).css('width').slice(0, -2));
    if (that.viewpointFieldOfView == null) {
      var fieldOfView = viewpoint.getAttribute('fieldOfView');
      // Sometimes the page is not fully loaded by that point and the field of view is not yet available.
      // In that case we add a callback at the end of the queue to try again when all other callbacks are finished.
      if (fieldOfView == null) {
        setTimeout(that.onresize, 0);
        return;
      }
      that.viewpointFieldOfView = fieldOfView;
    }

    var fieldOfViewY = that.viewpointFieldOfView;
    if (viewWidth > viewHeight) {
      var tanHalfFieldOfViewY = Math.tan(0.5 * that.viewpointFieldOfView) * viewHeight / viewWidth;
      fieldOfViewY = 2.0 * Math.atan(tanHalfFieldOfViewY);
    }

    viewpoint.setAttribute('fieldOfView', fieldOfViewY);
  };
  this.ondialogwindow = function(opening) {
    // Pause the simulation if needed when a pop-up dialog window is open
    // and restart running the simulation when it is closed
    if (opening && that.isAutomaticallyPaused === undefined) {
      that.isAutomaticallyPaused = webots.currentView.pauseButton.style.display === 'inline';
      that.pauseButton.click();
    } else if (!opening && that.isAutomaticallyPaused) {
      that.real_timeButton.click();
      that.isAutomaticallyPaused = undefined;
    }
  };
  window.onresize = this.onresize;
  this.robotWindowNames = {}; // map robot name to robot window name used as key in robotWindows lists
  this.robotWindows = {};
  this.followedObject = null; // after initialization contains the id of the followed node or -1 if no object is followed
  // If the followed object has moved since the last time we updated the viewpoint position, this field will contain a
  // vector with the translation applied to the object.
  this.followedObjectDeltaPosition = null;
  this.viewpointMass = 1.0; // Mass of the viewpoint used during the object following algorithm.
  this.viewpointFriction = 0.05; // Friction applied to the viewpoint whenever it is going faster than the followed object.
  this.viewpointForce = null; // Vector with the force that will be applied to the viewpoint for the next delta T.
  this.viewpointVelocity = null; // Current velocity of the viewpoint.
  this.viewpointLastUpdate = undefined; // Last time we updated the position of the viewpoint.
  this.onmousedown = null;
  this.onworldloaded = null;
  this.view3D = view3D;
  this.viewpointFieldOfView = null;
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
  this.view3D.className = view3D.className + ' webotsView';
  $(this.view3D).append(
    "<ul id='contextMenu'>" +
    "<li class='ui-widget-header'><div id='contextMenuTitle'>Object</div></li>" +
    "<li id='contextMenuFollow'><div>Follow</div></li>" +
    "<li id='contextMenuUnfollow'><div>Unfollow</div></li>" +
    "<li><div class='ui-state-disabled'>Zoom</div></li>" +
    "<li id='contextMenuRobotWindow'><div id='contextMenuRobotWindowDiv'>Robot window</div></li>" +
    "<li id='contextMenuEditController'><div id='contextMenuEditControllerDiv'>Edit controller</div></li>" +
    "<li><div class='ui-state-disabled'>Delete</div></li>" +
    "<li><div class='ui-state-disabled'>Properties</div></li>" +
    '</ul>');
  $('#contextMenu').menu({items: '> :not(.ui-widget-header)'});
  $('#contextMenu').css('position', 'absolute');
  $('#contextMenu').css('z-index', 1);
  $('#contextMenu').css('display', 'none');
  $('#contextMenu').on('menuselect', function(event, ui) {
    if (ui.item.children().hasClass('ui-state-disabled'))
      return;
    var id = ui.item.attr('id');
    if (id === 'contextMenuFollow')
      that.follow(that.selection.id);
    else if (id === 'contextMenuUnfollow')
      that.follow('none');
    else if (id === 'contextMenuEditController') {
      var controller = that.selection.getAttribute('controller');
      $('#webotsEditor').dialog('open');
      $('#webotsEditor').dialog('option', 'title', 'Controller: ' + controller);
      if (that.editor.dirname !== controller) {
        that.editor.closeAllTabs();
        that.editor.dirname = controller;
        that.stream.socket.send('get controller:' + controller);
      }
    } else if (id === 'contextMenuRobotWindow') {
      var robotName = that.selection.getAttribute('name');
      var win = that.robotWindows[that.robotWindowNames[robotName]];
      if (win) {
        if (win === that.infoWindow) {
          if (!that.infoWindow.isOpen())
            that.toggleInfo();
        } else
          win.open();
      } else
        console.log('No valid robot window for robot: ' + that.selection.getAttribute('name'));
    } else
      console.log('Unknown menu item: ' + id);
    $('#contextMenu').css('display', 'none');
  });
  this.console = new webots.Console(view3D, this.mobileDevice);
  this.editor = new webots.Editor(view3D, this);
  this.infoWindow = null;
  this.selection = null;
  this.x3dScene = null;
  this.x3dNode = null;
  this.initialViewpointPosition = null;
  this.initialViewpointOrientation = null;
  this.mouseState = {
    'initialized': false,
    'mouseDown': 0,
    'moved': false,
    'pickPosition': null,
    'wheelFocus': false,
    'wheelTimeout': null
  };
  this.animation = null;
  this.enableNavigation = true;
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

webots.View.prototype.open = function(url, mode) {
  if (mode === undefined)
    mode = 'x3dom';
  var that = this;
  this.mode = mode;
  this.videoStream = null;
  if (mode === 'video') {
    this.url = url;
    this.video = document.createElement('video');
    this.video.style.background = 'grey';
    this.video.id = 'remoteVideo';
    this.video.class = 'rounded centered';
    this.video.autoplay = 'true';
    this.video.width = 800;
    this.video.height = 600;
    this.view3D.appendChild(this.video);
    initWorld();
    return;
  }
  if (mode !== 'x3dom') {
    console.log('Error: webots.View.open: wrong mode argument: ' + mode);
    return;
  }
  if (this.broadcast)
    this.setTimeout(-1);
  if (!this.x3dScene) {
    this.x3dNode = document.createElement('x3d');
    this.x3dNode.className = 'webots3DView';
    this.view3D.appendChild(this.x3dNode);
    var param = document.createElement('param');
    param.name = 'showProgress';
    param.value = false;
    this.x3dNode.appendChild(param);
    this.x3dScene = document.createElement('Scene');
    this.x3dNode.appendChild(this.x3dScene);
  }
  if (this.url === undefined) {
    this.url = url;
    initX3Dom();
  } else {
    this.url = url;
    initWorld();
  }
  this.isWebSocketProtocol = that.url.startsWith('ws://') || that.url.startsWith('wss://');
  function requestQuit() {
    if (that.unloggedFileModified || that.editor.hasUnsavedChanges()) {
      var text;
      if (that.unloggedFileModified || !webots.User1Id)
        text = 'Your changes to the robot controller will be lost because you are not logged in.';
      else
        text = 'Your unsaved changes to the robot controller will be lost.';
      var quitDialog = document.getElementById('quitDialog');
      if (!quitDialog) {
        quitDialog = document.createElement('div');
        quitDialog.id = 'quitDialog';
        $(quitDialog).html(text);
        that.view3D.appendChild(quitDialog);
        $(quitDialog).dialog({
          title: 'Quit the simulation?',
          modal: true,
          resizable: false,
          appendTo: that.view3D,
          open: webotsOpenDialog,
          buttons: {
            'Cancel': function() {
              $(this).dialog('close');
            },
            'Quit': function() {
              $(this).dialog('close');
              quit();
            }
          }
        });
      } else
        $(quitDialog).dialog('open');
      return;
    }
    quit();
  }
  function quit() {
    if (that.broadcast)
      return;
    $('#webotsProgressMessage').html('Bye bye...');
    $('#webotsProgress').show();
    that.quitting = true;
    that.onquit();
  }
  function reset() {
    if (that.broadcast)
      return;
    that.time = 0; // reset time to correctly compute the initial deadline
    $('#webotsProgressMessage').html('Restarting simulation...');
    $('#webotsProgress').show();
    that.runOnLoad = that.pauseButton.style.display === 'inline';
    pause();
    for (var i = 0; i < that.editor.filenames.length; i++) {
      that.editor.save(i);
      if (that.editor.needToUploadFiles[i])
        that.editor.upload(i);
    }
    that.onrobotwindowsdestroy();
    if (that.timeout >= 0) {
      that.deadline = that.timeout;
      $('#webotsTimeout').html(webots.parseMillisecondsIntoReadableTime(that.deadline));
    }
    enableToolBarButtons(false);
    that.stream.socket.send('reset');
  }
  function pause() {
    if (that.broadcast)
      return;
    $('#contextMenu').css('display', 'none');
    that.stream.socket.send('pause');
  }
  function realTime() {
    if (that.broadcast)
      return;
    $('#contextMenu').css('display', 'none');
    that.stream.socket.send('real-time:' + that.timeout);
    that.pauseButton.style.display = 'inline';
    that.real_timeButton.style.display = 'none';
  }
  function step() {
    if (that.broadcast)
      return;
    $('#contextMenu').css('display', 'none');
    that.pauseButton.style.display = 'none';
    that.real_timeButton.style.display = 'inline';
    that.stream.socket.send('step');
  }
  function requestFullscreen() {
    $('#contextMenu').css('display', 'none');
    var elem = that.view3D;
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
    $('#contextMenu').css('display', 'none');
    if (document.exitFullscreen)
      document.exitFullscreen();
    else if (document.msExitFullscreen)
      document.msExitFullscreen();
    else if (document.mozCancelFullScreen)
      document.mozCancelFullScreen();
    else if (document.webkitExitFullscreen)
      document.webkitExitFullscreen();
  }
  function fullscreenchange(event) {
    var element = document.fullScreenElement || document.mozFullScreenElement || document.webkitFullScreenElement || document.msFullScreenElement || document.webkitCurrentFullScreenElement;
    if (element != null) {
      that.fullscreenButton.style.display = 'none';
      that.exit_fullscreenButton.style.display = 'inline';
    } else {
      that.fullscreenButton.style.display = 'inline';
      that.exit_fullscreenButton.style.display = 'none';
    }
  }
  function toolBarButton(name, tooltip) {
    var buttonName = name + 'Button';
    that[buttonName] = document.createElement('button');
    that[buttonName].id = buttonName;
    that[buttonName].className = 'toolBarButton';
    that[buttonName].title = tooltip;
    that[buttonName].style.backgroundImage = 'url(' + webots.WwiUrl + 'images/' + name + '.png)';
    return that[buttonName];
  }
  function toggleConsole() {
    $('#contextMenu').css('display', 'none');
    if ($('#webotsConsole').is(':visible')) {
      $('#webotsConsole').dialog('close');
      that.consoleButton.classList.remove('toolBarButtonActive');
    } else {
      $('#webotsConsole').dialog('open');
      that.consoleButton.classList.add('toolBarButtonActive');
    }
  }
  function toggleHelp() {
    $('#contextMenu').css('display', 'none');
    if (!that.helpWindow) {
      if (!that.broadcast && that.webotsDocUrl)
        var webotsDocUrl = that.webotsDocUrl;
      that.helpWindow = new webots.HelpWindow(that.view3D, webotsDocUrl, that.mobileDevice);
      that.helpButton.classList.add('toolBarButtonActive');
    } else if ($('#webotsHelp').is(':visible')) {
      $('#webotsHelp').dialog('close');
      that.helpButton.classList.remove('toolBarButtonActive');
    } else {
      $('#webotsHelp').dialog('open');
      that.helpButton.classList.add('toolBarButtonActive');
    }
  }
  function enableToolBarButtons(enabled) {
    var buttons = [that.infoButton, that.resetButton, that.stepButton, that.real_timeButton, that.pauseButton, that.consoleButton];
    for (var i in buttons) {
      if (buttons[i]) {
        if ((!that.broadcast || buttons[i] === that.consoleButton) && enabled) {
          buttons[i].disabled = false;
          buttons[i].classList.remove('toolBarButtonDisabled');
        } else {
          buttons[i].disabled = true;
          buttons[i].classList.add('toolBarButtonDisabled');
        }
      }
    }
  }
  function initWorld() {
    // override the original x3dom function to workaround a bug with USE/DEF nodes
    x3dom.Texture.prototype.update = function() {
      if (x3dom.isa(this.node, x3dom.nodeTypes.Text))
        this.updateText();
      else
        this.updateTexture();
      // x3dom bug: do not call validateGLObject because it somehow prevents USE Apperance update (bug #5117)
      // this.node.validateGLObject();
    };

    // redirect the X3Dom log entirely to the JS console
    if (that.mode === 'x3dom') {
      if (this.debug) {
        x3dom.debug.doLog = function(msg, type) {
          console.log(type + ': ' + msg);
        };
      }
      x3dom.runtime.ready = addX3domMouseNavigation;
    }
    if (that.isWebSocketProtocol) {
      that.progress = document.createElement('div');
      that.progress.id = 'webotsProgress';
      that.progress.innerHTML = "<div><img src='" + webots.WwiUrl + "images/load_animation.gif'>" +
                                "</div><div id='webotsProgressMessage'>Initializing...</div>";
      that.view3D.appendChild(that.progress);
      that.toolBar = document.createElement('div');
      that.toolBar.id = 'toolBar';
      that.toolBar.left = document.createElement('div');
      that.toolBar.left.className = 'toolBarLeft';
      that.toolBar.left.appendChild(toolBarButton('quit', 'Quit the simulation'));
      that.quitButton.onclick = requestQuit;
      that.toolBar.left.appendChild(toolBarButton('info', 'Open the information window'));
      that.infoButton.onclick = toggleInfo;
      that.toolBar.left.appendChild(toolBarButton('reset', 'Save controllers and reset the simulation'));
      that.resetButton.onclick = reset;
      that.toolBar.left.appendChild(toolBarButton('step', 'Perform one simulation step'));
      that.stepButton.onclick = step;
      that.toolBar.left.appendChild(toolBarButton('real_time', 'Run the simulation in real time'));
      that.real_timeButton.onclick = realTime;
      that.toolBar.left.appendChild(toolBarButton('pause', 'Pause the simulation'));
      that.pauseButton.onclick = pause;
      that.pauseButton.style.display = 'none';
      var div = document.createElement('div');
      div.className = 'webotsTime';
      var clock = document.createElement('span');
      clock.id = 'webotsClock';
      clock.title = 'Current simulation time';
      clock.innerHTML = webots.parseMillisecondsIntoReadableTime(0);
      var timeout = document.createElement('span');
      timeout.id = 'webotsTimeout';
      timeout.title = 'Simulation time out';
      timeout.innerHTML = webots.parseMillisecondsIntoReadableTime(that.deadline);
      div.appendChild(clock);
      div.appendChild(document.createElement('br'));
      div.appendChild(timeout);
      that.toolBar.left.appendChild(div);
      that.toolBar.left.appendChild(toolBarButton('console', 'Open the console window'));
      that.consoleButton.onclick = toggleConsole;
      that.toolBar.right = document.createElement('div');
      that.toolBar.right.className = 'toolBarRight';
      that.toolBar.right.appendChild(toolBarButton('help', 'Get help on the simulator'));
      that.helpButton.onclick = toggleHelp;
      if (that.fullscreenEnabled) {
        that.toolBar.right.appendChild(toolBarButton('exit_fullscreen', 'Exit fullscreen'));
        that.exit_fullscreenButton.onclick = exitFullscreen;
        that.exit_fullscreenButton.style.display = 'none';
        that.toolBar.right.appendChild(toolBarButton('fullscreen', 'Enter fullscreen'));
        that.fullscreenButton.onclick = requestFullscreen;
      }
      that.toolBar.appendChild(that.toolBar.left);
      that.toolBar.appendChild(that.toolBar.right);
      that.view3D.appendChild(that.toolBar);
      enableToolBarButtons(false);
      if (that.broadcast) {
        that.quitButton.disabled = true;
        that.quitButton.classList.add('toolBarButtonDisabled');
        $('#contextMenuRobotWindowDiv').addClass('ui-state-disabled');
        $('#contextMenuEditControllerDiv').addClass('ui-state-disabled');
      }
      document.addEventListener('fullscreenchange', fullscreenchange);
      document.addEventListener('webkitfullscreenchange', fullscreenchange);
      document.addEventListener('mozfullscreenchange', fullscreenchange);
      document.addEventListener('MSFullscreenChange', fullscreenchange);
      if (that.url.endsWith('.wbt')) { // url expected form: "ws://localhost:80/simple/worlds/simple.wbt"
        var callback;
        if (that.mode === 'video')
          callback = videoFinalize;
        else
          callback = x3domFinalize;
        that.server = new webots.Server(that.url, that, callback);
      } else // url expected form: "ws://cyberbotics2.cyberbotics.com:80"
        that.stream = new webots.Stream(that.url, that, x3domFinalize);
    } else // assuming it's an URL to a .x3d file
      initX3dFile();
  }

  function toggleInfo() {
    that.toggleInfo();
  }

  function initX3Dom() { // load x3dom.css, x3dom-full.js and calls initWorld
    var head = document.getElementsByTagName('head')[0];
    var link = document.createElement('link');
    link.rel = 'stylesheet';
    link.type = 'text/css';
    link.href = 'https://www.cyberbotics.com/x3dom/dev_13062018/x3dom.css';
    link.media = 'all';
    head.appendChild(link);
    // source http://stackoverflow.com/questions/950087/include-a-javascript-file-in-another-javascript-file
    var script = document.createElement('script');
    script.src = 'https://www.cyberbotics.com/x3dom/dev_13062018/x3dom-full.js';
    script.onload = initWorld;
    script.onerror = function() {
      that.onerror('Error when loading the X3DOM library');
    };
    head.appendChild(script); // fire the loading
  }

  function initX3dFile() {
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.open('GET', that.url, true);
    xmlhttp.overrideMimeType('text/xml');
    xmlhttp.onreadystatechange = function() {
      if (xmlhttp.readyState === 4 && xmlhttp.status === 200) {
        var scene = xmlhttp.responseText.substring(xmlhttp.responseText.indexOf('<Scene>') + 8, xmlhttp.responseText.lastIndexOf('</Scene>'));
        $(that.x3dScene).append(scene);
        x3domFinalize();
      }
    };
    xmlhttp.send();
  }

  function x3domFinalize() {
    $('#webotsProgressMessage').html('Loading HTML and Javascript files...');
    if (that.followedObject == null || that.broadcast) {
      var viewpoint = that.x3dScene.getElementsByTagName('Viewpoint')[0];
      that.initialViewpointPosition = viewpoint.getAttribute('position');
      that.initialViewpointOrientation = viewpoint.getAttribute('orientation');
      var viewpointFollowSmoothness = viewpoint.getAttribute('followSmoothness');
      if (viewpointFollowSmoothness !== null)
        that.setViewpointMass(viewpointFollowSmoothness);
      var viewpointFollowedId = viewpoint.getAttribute('followedId');
      if (viewpointFollowedId != null) {
        that.followedObject = viewpointFollowedId;
        that.follow(viewpointFollowedId);
      } else
        that.followedObject = 'none';
    } else
      // reset follow parameters
      that.follow(that.followedObject);

    if (!that.isWebSocketProtocol) { // skip robot windows initialization
      if (that.animation != null)
        that.animation.init(loadFinalize);
      else
        loadFinalize();
      that.onresize();
      return;
    }

    function loadRobotWindow(node) {
      var windowName = node.getAttribute('window');
      that.robotWindowNames[node.getAttribute('name')] = windowName;
      var win = new webots.RobotWindow(that.view3D, windowName, that.mobileDevice);
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
        win.setProperties({title: worldInfo.getAttribute('title') + user, close: closeInfoWindow});
        that.infoWindow = win;
      } else
        win.setProperties({title: 'Robot: ' + node.getAttribute('name')});
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
        pendingRequestsCount--;
        if (pendingRequestsCount === 0)
          loadFinalize();
      });
    }

    var worldInfo = that.x3dScene.getElementsByTagName('WorldInfo')[0];
    var infoWindowName = worldInfo.getAttribute('window');
    var pendingRequestsCount = 1; // start from 1 so that it can be 0 only after the loop is completed and all the nodes are checked
    var nodes = that.x3dScene.childNodes;
    for (var i = 0; i < nodes.length; i++) {
      if (nodes[i].nodeType !== 1 || nodes[i].nodeName.toUpperCase() !== 'TRANSFORM' || !nodes[i].hasAttribute('window') || !nodes[i].hasAttribute('name'))
        continue;
      loadRobotWindow(nodes[i]);
    }
    pendingRequestsCount--; // notify that loop is completed
    if (pendingRequestsCount === 0)
      // if no pending requests execute loadFinalize
      // otherwise it will be executed when the last request will be handled
      loadFinalize();
  }

  function loadFinalize() {
    $('#webotsProgress').hide();
    enableToolBarButtons(true);

    if (that.onready)
      that.onready();

    // restore robot windows
    if (that.robotWindowsGeometries) { // on reset
      for (var win in that.robotWindows) {
        if (win in that.robotWindowsGeometries) {
          that.robotWindows[win].restoreGeometry(that.robotWindowsGeometries[win]);
          if (that.robotWindowsGeometries[win].open) {
            if (that.robotWindows[win] === that.infoWindow)
              that.toggleInfo();
            else
              that.robotWindows[win].open();
          }
        }
      }
    } else if (that.infoWindow && !that.broadcast) // at first load
      that.toggleInfo();
    that.viewpointLastUpdate = undefined;

    if (that.runOnLoad)
      realTime();
  }

  function rotateViewpoint(viewpoint, params) {
    var halfYawAngle = -0.005 * params.dx;
    var halfPitchAngle = -0.005 * params.dy;
    if (that.mouseState.pickPosition == null) {
      halfYawAngle /= -8;
      halfPitchAngle /= -8;
    }
    var sinusYaw = Math.sin(halfYawAngle);
    var sinusPitch = Math.sin(halfPitchAngle);
    var tx = (1 - params.c) * params.vo.x;
    var pitch = new x3dom.fields.SFVec3f(tx * params.vo.x + params.c, tx * params.vo.y + params.s * params.vo.z, tx * params.vo.z - params.s * params.vo.y);
    var pitchRotation = new x3dom.fields.Quaternion(sinusPitch * pitch.x, sinusPitch * pitch.y, sinusPitch * pitch.z, Math.cos(halfPitchAngle));
    var worldUp = new x3dom.fields.SFVec3f(0, 1, 0);
    var yawRotation = new x3dom.fields.Quaternion(sinusYaw * worldUp.x, sinusYaw * worldUp.y, sinusYaw * worldUp.z, Math.cos(halfYawAngle));
    var deltaRotation = yawRotation.multiply(pitchRotation);
    if (that.mouseState.pickPosition) {
      var currentPosition = deltaRotation.toMatrix().multMatrixVec(params.vp.subtract(that.mouseState.pickPosition)).add(that.mouseState.pickPosition);
      viewpoint.setAttribute('position', currentPosition.toString());
    }
    var voq = x3dom.fields.Quaternion.axisAngle(new x3dom.fields.SFVec3f(params.vo.x, params.vo.y, params.vo.z), params.vo.w);
    var currentOrientation = deltaRotation.multiply(voq);
    var aa = currentOrientation.toAxisAngle();
    viewpoint.setAttribute('orientation', aa[0].toString() + ' ' + aa[1]);
  }

  function translateViewpoint(viewpoint, params) {
    var targetRight = -params.distanceToPickPosition * params.scaleFactor * params.dx;
    var targetUp = params.distanceToPickPosition * params.scaleFactor * params.dy;
    var tx = (1 - params.c) * params.vo.x;
    var pitch = new x3dom.fields.SFVec3f(tx * params.vo.x + params.c, tx * params.vo.y + params.s * params.vo.z, tx * params.vo.z - params.s * params.vo.y);
    var ty = (1 - params.c) * params.vo.y;
    var yaw = new x3dom.fields.SFVec3f(ty * params.vo.x - params.s * params.vo.z, ty * params.vo.y + params.c, ty * params.vo.z + params.s * params.vo.x);
    var target = params.vp.add(pitch.multiply(targetRight).add(yaw.multiply(targetUp)));
    viewpoint.setAttribute('position', target.toString());
  }

  function zoomAndTiltViewpoint(viewpoint, params) {
    var tz = (1 - params.c) * params.vo.z;
    var roll = new x3dom.fields.SFVec3f(tz * params.vo.x + params.s * params.vo.y, tz * params.vo.y - params.s * params.vo.x, tz * params.vo.z + params.c);
    var target = params.vp.add(roll.multiply(params.zoomScale));
    viewpoint.setAttribute('position', target.toString());
    var zRotation = x3dom.fields.Quaternion.axisAngle(roll, params.tiltAngle);
    var voq = x3dom.fields.Quaternion.axisAngle(new x3dom.fields.SFVec3f(params.vo.x, params.vo.y, params.vo.z), params.vo.w);
    var aa = zRotation.multiply(voq).toAxisAngle();
    viewpoint.setAttribute('orientation', aa[0].toString() + ' ' + aa[1]);
  }

  function initMouseMove(event) {
    that.mouseState.x = event.clientX;
    that.mouseState.y = event.clientY;
    that.mouseState.initialX = null;
    that.mouseState.initialY = null;
    that.mouseState.moved = false;
    that.mouseState.initialTimeStamp = Date.now();
    that.mouseState.longClick = false;
    var mousePosition = that.x3dNode.runtime.mousePosition(event);
    var shootRay = that.x3dNode.runtime.shootRay(mousePosition[0], mousePosition[1]);
    that.mouseState.pickPosition = shootRay.pickPosition;
    if ($('#contextMenu').css('display') === 'block') {
      $('#contextMenu').css('display', 'none');
      that.contextMenu = true;
    } else
      that.contextMenu = false;
  }

  function clearMouseMove() {
    if (that.mouseState.mobileDevice)
      that.mouseState.longClick = Date.now() - that.mouseState.initialTimeStamp >= 100;
    else
      that.mouseState.longClick = Date.now() - that.mouseState.initialTimeStamp >= 1000;
    if (that.mouseState.moved === false) {
      that.previousSelection = that.selection;
      unselect();
    } else
      that.previousSelection = null;
    that.mouseState.previousMouseDown = that.mouseState.mouseDown;
    that.mouseState.mouseDown = 0;
    that.mouseState.initialTimeStamp = null;
    that.mouseState.initialX = null;
    that.mouseState.initialY = null;
  }

  function addX3domMouseNavigation() {
    if (that.mobileDevice) {
      that.x3dNode.addEventListener('touchmove', function(event) {
        if (!that.enableNavigation || event.targetTouches.length === 0 || event.targetTouches.length > 2)
          return;
        if (that.mouseState.initialTimeStamp === null)
          // prevent applying mouse move action before drag initialization in mousedrag event
          return;
        if ((that.mouseState.mouseDown !== 2) !== (event.targetTouches.length > 1))
          // gesture single/multi touch changed after initialization
          return;

        var touch = event.targetTouches['0'];

        var params = {};
        var viewpoint = that.x3dScene.getElementsByTagName('Viewpoint')[0];
        params.vp = x3dom.fields.SFVec3f.parse(viewpoint.getAttribute('position'));
        params.vo = x3dom.fields.SFVec4f.parse(viewpoint.getAttribute('orientation'));
        params.c = Math.cos(params.vo.w);
        params.s = Math.sin(params.vo.w);
        params.scaleFactor = 1.90 * Math.tan(that.viewpointFieldOfView / 2);
        var viewHeight = parseFloat($(that.x3dNode).css('height').slice(0, -2));
        var viewWidth = parseFloat($(that.x3dNode).css('width').slice(0, -2));
        params.scaleFactor /= Math.max(viewHeight, viewWidth);

        if (that.mouseState.pickPosition == null)
          params.distanceToPickPosition = params.vp.length();
        else
          params.distanceToPickPosition = params.vp.subtract(that.mouseState.pickPosition).length() - 0.05; // FIXME this is different from webots.
        if (params.distanceToPickPosition < 0.001) // 1 mm
          params.distanceToPickPosition = 0.001;
        var x = Math.round(touch.clientX); // discard decimal values returned on android
        var y = Math.round(touch.clientY);

        if (that.mouseState.mouseDown === 2) { // translation
          params.dx = x - that.mouseState.x;
          params.dy = y - that.mouseState.y;
          translateViewpoint(viewpoint, params);

          // on small phone screens (Android) this is needed to correctly detect clicks and longClicks
          if (that.mouseState.initialX == null && that.mouseState.initialY == null) {
            that.mouseState.initialX = Math.round(that.mouseState.x);
            that.mouseState.initialY = Math.round(that.mouseState.y);
          }
          if (Math.abs(params.dx) < 2 && Math.abs(params.dy) < 2 &&
              Math.abs(that.mouseState.initialX - x) < 5 && Math.abs(that.mouseState.initialY - y) < 5)
            that.mouseState.moved = false;
          else
            that.mouseState.moved = true;
        } else {
          var touch1 = event.targetTouches['1'];
          var x1 = Math.round(touch1.clientX);
          var y1 = Math.round(touch1.clientY);
          var distanceX = x - x1;
          var distanceY = y - y1;
          var newTouchDistance = distanceX * distanceX + distanceY * distanceY;
          var pinchSize = that.mouseState.touchDistance - newTouchDistance;

          var moveX1 = x - that.mouseState.x;
          var moveX2 = x1 - that.mouseState.x1;
          var moveY1 = y - that.mouseState.y;
          var moveY2 = y1 - that.mouseState.y1;
          var ratio = window.devicePixelRatio || 1;

          if (Math.abs(pinchSize) > 500 * ratio) { // zoom and tilt
            var d;
            if (Math.abs(moveX2) < Math.abs(moveX1))
              d = moveX1;
            else
              d = moveX2;
            params.tiltAngle = 0.0004 * d;
            params.zoomScale = params.scaleFactor * 0.015 * pinchSize;
            zoomAndTiltViewpoint(viewpoint, params);
          } else if (Math.abs(moveY2 - moveY1) < 3 * ratio && Math.abs(moveX2 - moveX1) < 3 * ratio) { // rotation (pitch and yaw)
            params.dx = moveX1 * 0.8;
            params.dy = moveY1 * 0.5;
            rotateViewpoint(viewpoint, params);
          }

          that.mouseState.touchDistance = newTouchDistance;
          that.mouseState.moved = true;
        }

        that.mouseState.x = x;
        that.mouseState.y = y;
        that.mouseState.x1 = x1;
        that.mouseState.y1 = y1;
        if (that.ontouchmove)
          that.ontouchmove(event);
      }, true);
      that.x3dNode.addEventListener('touchstart', function(event) {
        initMouseMove(event.targetTouches['0']);
        if (event.targetTouches.length === 2) {
          var touch1 = event.targetTouches['1'];
          that.mouseState.x1 = touch1.clientX;
          that.mouseState.y1 = touch1.clientY;
          var distanceX = that.mouseState.x - that.mouseState.x1;
          var distanceY = that.mouseState.y - that.mouseState.y1;
          that.mouseState.touchDistance = distanceX * distanceX + distanceY * distanceY;
          that.mouseState.touchOrientation = Math.atan2(that.mouseState.y1 - that.mouseState.y, that.mouseState.x1 - that.mouseState.x);
          that.mouseState.mouseDown = 3; // two fingers: rotation, tilt, zoom
        } else
          that.mouseState.mouseDown = 2; // 1 finger: translation or single click
      }, true);
      that.x3dNode.addEventListener('touchend', function(event) {
        clearMouseMove();
        if (that.ontouchend)
          that.ontouchend(event);
      }, true);
    } else {
      that.x3dNode.addEventListener('wheel', function(event) {
        var viewpoint = that.x3dScene.getElementsByTagName('Viewpoint')[0];
        var vp = x3dom.fields.SFVec3f.parse(viewpoint.getAttribute('position'));
        var vo = x3dom.fields.SFVec4f.parse(viewpoint.getAttribute('orientation'));
        var mousePosition = that.x3dNode.runtime.mousePosition(event);
        var shootRay = that.x3dNode.runtime.shootRay(mousePosition[0], mousePosition[1]);
        var distanceToPickPosition;
        that.mouseState.pickPosition = shootRay.pickPosition;
        if (that.mouseState.pickPosition == null)
          distanceToPickPosition = vp.length();
        else
          distanceToPickPosition = vp.subtract(that.mouseState.pickPosition).length();
        if (distanceToPickPosition < 0.001) // 1 mm
          distanceToPickPosition = 0.001;
        if (!that.enableNavigation || that.mouseState.wheelFocus === false) {
          var offset = event.deltaY;
          if (event.deltaMode === 1)
            offset *= 40; // standard line height in pixel
          window.scroll(0, window.pageYOffset + offset);
          if (that.mouseState.wheelTimeout) { // you have to rest at least 1.5 seconds over the x3d canvas
            clearTimeout(that.mouseState.wheelTimeout); // so that the wheel focus will get enabled and
            that.mouseState.wheelTimeout = setTimeout(wheelTimeoutCallback, 1500); // allow you to zoom in/out.
          }
          return;
        }
        var scaleFactor = 0.02 * distanceToPickPosition * ((event.deltaY < 0) ? -1 : 1);
        var c = Math.cos(vo.w);
        var s = Math.sin(vo.w);
        var tz = (1 - c) * vo.z;
        var roll = new x3dom.fields.SFVec3f(tz * vo.x + s * vo.y, tz * vo.y - s * vo.x, tz * vo.z + c);
        var target = vp.add(roll.multiply(scaleFactor));
        viewpoint.setAttribute('position', target.toString());
        if (that.onmousewheel)
          that.onmousewheel(event);
      }, true);
      that.x3dNode.addEventListener('mousemove', function(event) {
        if (!that.enableNavigation && event.button === 0)
          return;
        if (that.mouseState.x === undefined)
          // mousedown event has not been called yet
          // this could happen for example when another application has focus while loading the scene
          return;
        if ('buttons' in event)
          that.mouseState.mouseDown = event.buttons;
        else if ('which' in event) { // Safari only
          switch (event.which) {
            case 0: that.mouseState.mouseDown = 0; break;
            case 1: that.mouseState.mouseDown = 1; break;
            case 2: that.mouseState.pressedButton = 4; break;
            case 3: that.mouseState.pressedButton = 2; break;
            default: that.mouseState.pressedButton = 0; break;
          }
        }
        if (that.mouseState.mouseDown === 0) {
          if (that.animation && that.animation.playSlider && that.animation.sliding) {
            var w = event.target.clientWidth - 66; // size of the borders of the slider
            var x = event.clientX - event.target.getBoundingClientRect().left - 48; // size of the left border (including play button) of the slider
            var value = 100 * x / w;
            if (value < 0)
              value = 0;
            else if (value >= 100)
              value = 99.999;
            that.animation.playSlider.slider('value', value);
            // setting the value should trigger the change event, unfortunately, doesn't seem to work reliably,
            // therefore, we need to trigger this event manually:
            var ui = {};
            ui.value = value;
            that.animation.playSlider.slider('option', 'change').call(that.animation.playSlider, event, ui);
          }
          return;
        }
        if (that.mouseState.initialTimeStamp === null)
          // prevent applying mouse move action before drag initialization in mousedrag event
          return;

        var viewpoint = that.x3dScene.getElementsByTagName('Viewpoint')[0];
        var params = {};
        params.dx = event.clientX - that.mouseState.x;
        params.dy = event.clientY - that.mouseState.y;
        params.vp = x3dom.fields.SFVec3f.parse(viewpoint.getAttribute('position'));
        params.vo = x3dom.fields.SFVec4f.parse(viewpoint.getAttribute('orientation'));
        params.c = Math.cos(params.vo.w);
        params.s = Math.sin(params.vo.w);

        if (that.mouseState.pickPosition == null)
          params.distanceToPickPosition = params.vp.length();
        else
          params.distanceToPickPosition = params.vp.subtract(that.mouseState.pickPosition).length() - 0.05; // FIXME this is different from webots.
        if (params.distanceToPickPosition < 0.001) // 1 mm
          params.distanceToPickPosition = 0.001;

        // FIXME this is different from webots. We need to understand why the same formula doesn't work.
        params.scaleFactor = 1.90 * Math.tan(that.viewpointFieldOfView / 2);
        var viewHeight = parseFloat($(that.x3dNode).css('height').slice(0, -2));
        var viewWidth = parseFloat($(that.x3dNode).css('width').slice(0, -2));
        params.scaleFactor /= Math.max(viewHeight, viewWidth);

        if (that.mouseState.mouseDown === 1) { // left mouse button to rotate viewpoint
          params.distanceToPickPosition = 0;
          rotateViewpoint(viewpoint, params);
        } else if (that.mouseState.mouseDown === 2) // right mouse button to translate viewpoint {}
          translateViewpoint(viewpoint, params);
        else if (that.mouseState.mouseDown === 3 || that.mouseState.mouseDown === 4) { // both left and right button or middle button to zoom
          params.tiltAngle = 0.01 * params.dx;
          params.zoomScale = params.distanceToPickPosition * params.scaleFactor * 10 * params.dy; // FIXME this is different from webots.
          zoomAndTiltViewpoint(viewpoint, params, true);
        }
        that.mouseState.moved = event.clientX !== that.mouseState.x || event.clientY !== that.mouseState.y;
        that.mouseState.x = event.clientX;
        that.mouseState.y = event.clientY;
        if (that.onmousedrag)
          that.onmousedrag(event);
      }, true);
      that.x3dNode.addEventListener('mousedown', function(event) {
        that.mouseState.wheelFocus = true;
        if (event.button === 0)
          that.mouseState.mouseDown |= 1;
        else if (event.button === 1)
          that.mouseState.mouseDown |= 4;
        else if (event.button === 2)
          that.mouseState.mouseDown |= 2;
        initMouseMove(event);
      }, true);
      that.x3dNode.addEventListener('mouseup', clearMouseMove, true);
      that.x3dNode.addEventListener('mouseover', function(event) {
        that.mouseState.wheelTimeout = setTimeout(wheelTimeoutCallback, 1500);
      }, true);
      that.x3dNode.addEventListener('mouseleave', function(event) {
        if (that.mouseState.wheelTimeout != null) {
          clearTimeout(that.mouseState.wheelTimeout);
          that.mouseState.wheelTimeout = null;
        }
        that.mouseState.wheelFocus = false;
      }, true);
    }

    that.x3dScene.addEventListener('mouseup', function(event) {
      if (that.mouseState.moved === false && (!that.mouseState.longClick || that.mobileDevice)) {
        var s = getTopX3dElement(event.target);
        if (that.previousSelection == null || that.previousSelection.id !== s.id || (that.mouseState.previousMouseDown === 2 && (!that.mobileDevice || that.mouseState.longClick)))
          select(s);
        if (((that.mobileDevice && that.mouseState.longClick) || (!that.mobileDevice && that.mouseState.previousMouseDown === 2)) &&
            that.contextMenu === false && that.isWebSocketProtocol) {
          // right click: show popup menu
          $(function() {
            var title = that.selection.getAttribute('name');
            if (title == null || title === '') {
              title = that.selection.getAttribute('DEF');
              if (title == null || title === '')
                title = 'Object';
            }
            $('#contextMenuTitle').html(title);
            var controller = that.selection.getAttribute('controller');
            if (controller) { // the current selection is a robot
              $('#contextMenuEditController').css('display', 'inline');
              if (controller === 'void' || controller.length === 0 || (webots.User1Id && !webots.User1Authentication))
                $('#contextMenuEditController').children().addClass('ui-state-disabled');
              var robotName = that.selection.getAttribute('name');
              if (that.robotWindows[that.robotWindowNames[robotName]])
                $('#contextMenuRobotWindow').css('display', 'inline');
              else
                $('#contextMenuRobotWindow').css('display', 'none');
            } else {
              $('#contextMenuEditController').css('display', 'none');
              $('#contextMenuRobotWindow').css('display', 'none');
            }
            if (that.followedObject != null && (that.selection.id === that.followedObject || that.selection.getAttribute('DEF') === that.followedObject)) {
              $('#contextMenuFollow').css('display', 'none');
              $('#contextMenuUnfollow').css('display', 'inline');
            } else {
              $('#contextMenuFollow').css('display', 'inline');
              $('#contextMenuUnfollow').css('display', 'none');
            }
            // ensure that the context menu is completely visible
            var w = $('#contextMenu').width();
            var h = $('#contextMenu').height();
            var maxWidth = $('#playerDiv').width();
            var maxHeight = $('#playerDiv').height();
            var left;
            var top;
            if (maxWidth != null && (w + that.mouseState.x) > maxWidth)
              left = maxWidth - w;
            else
              left = that.mouseState.x;
            if (maxHeight != null && (h + that.mouseState.y) > maxHeight)
              top = maxHeight - h - $('#toolBar').height();
            else
              top = that.mouseState.y;
            $('#contextMenu').css('left', left + 'px');
            $('#contextMenu').css('top', top + 'px');
            $('#contextMenu').css('display', 'block');
          });
        }
      }
      if (that.onmouseup)
        that.onmouseup(event);
    }, false);
  }
  function wheelTimeoutCallback(event) {
    that.mouseState.wheelTimeout = null;
    that.mouseState.wheelFocus = true;
  }
  function getTopX3dElement(el) {
    // If it exists, return the upmost Solid, otherwise the top node
    var upmostSolid = null;
    while (el) {
      if (el.getAttribute('solid'))
        upmostSolid = el;
      if (el.parentNode === that.x3dScene)
        break;
      el = el.parentNode;
    }
    if (upmostSolid)
      return upmostSolid;
    return el;
  }

  function unselect() {
    if (that.selection) {
      var selectors = that.selection.getElementsByClassName('selector');
      for (var i = 0; i < selectors.length; i++) {
        var selector = selectors[i];
        selector.setAttribute('whichChoice', '-1');
      }
      that.selection = null;
    }
  }

  function select(el) {
    var selectors = el.getElementsByClassName('selector');
    for (var i = 0; i < selectors.length; i++) {
      var selector = selectors[i];
      selector.setAttribute('whichChoice', '0');
    }
    that.selection = el;
  }

  function videoFinalize() {
    console.log('video finalize');
    addVideoMouseNavigation();
    if (that.onready)
      that.onready();
  }

  function sendVideoMouseEvent(type, event, wheel) {
    var socket = that.stream.socket;
    if (!socket || socket.readyState !== 1)
      return;
    var modifier = (event.shiftKey ? 1 : 0) + (event.ctrlKey ? 2 : 0) + (event.altKey ? 4 : 0);
    socket.send('mouse ' + type + ' ' + event.button + ' ' + that.mouseState.mouseDown + ' ' +
                event.offsetX + ' ' + event.offsetY + ' ' + modifier + ' ' + wheel);
  }

  function onVideoMouseDown(event) {
    event.target.addEventListener('mousemove', onVideoMouseMove, false);
    sendVideoMouseEvent(-1, event, 0);
    event.preventDefault();
    return false;
  }

  function onVideoMouseMove(event) {
    if (that.mouseState.mouseDown === 0) {
      event.target.removeEventListener('mousemove', onVideoMouseMove, false);
      return false;
    }
    sendVideoMouseEvent(0, event, 0);
    return false;
  }

  function onVideoMouseUp(event) {
    event.target.removeEventListener('mousemove', onVideoMouseMove, false);
    sendVideoMouseEvent(1, event, 0);
    event.preventDefault();
    return false;
  }

  function onVideoWheel(event) {
    sendVideoMouseEvent(2, event, Math.sign(event.deltaY));
    return false;
  }

  function onVideoContextMenu(event) {
    event.preventDefault();
    return false;
  }

  function addVideoMouseNavigation() {
    that.video.addEventListener('mousedown', onVideoMouseDown, false);
    that.video.addEventListener('mouseup', onVideoMouseUp, false);
    that.video.addEventListener('wheel', onVideoWheel, false);
    that.video.addEventListener('contextmenu', onVideoContextMenu, false);
  }
};

webots.View.prototype.toggleInfo = function() {
  $('#contextMenu').css('display', 'none');
  if (!this.infoWindow)
    return;
  if (this.infoWindow.isOpen()) {
    this.infoWindow.close();
    this.infoButton.classList.remove('toolBarButtonActive');
  } else {
    this.infoWindow.open();
    this.infoButton.classList.add('toolBarButtonActive');
  }
};

webots.View.prototype.follow = function(id) {
  this.followedObject = id;
  this.viewpointForce = new x3dom.fields.SFVec3f(0.0, 0.0, 0.0);
  this.viewpointVelocity = new x3dom.fields.SFVec3f(0.0, 0.0, 0.0);
};

webots.View.prototype.setViewpointMass = function(mass) {
  this.viewpointMass = mass;
  if (this.viewpointMass <= 0.05)
    this.viewpointMass = 0.0;
  else {
    if (this.viewpointMass > 1.0)
      this.viewpointMass = 1.0;
    this.friction = 0.05 / this.viewpointMass;
  }
};

webots.View.prototype.updateViewpointPosition = function(forcePosition) {
  if (this.time === undefined)
    return;
  if (this.viewpointLastUpdate === undefined)
    this.viewpointLastUpdate = this.time;

  var timeInterval = Math.abs(this.time - this.viewpointLastUpdate) / 1000;
  var viewpoints = this.x3dScene.getElementsByTagName('Viewpoint');

  if (timeInterval > 0 && viewpoints[0]) {
    this.viewpointLastUpdate = this.time;
    var viewpointPosition = x3dom.fields.SFVec3f.parse(viewpoints[0].getAttribute('position'));
    var viewpointDeltaPosition;
    if (this.followedObjectDeltaPosition != null)
      this.viewpointForce = this.viewpointForce.add(this.followedObjectDeltaPosition);

    // Special case: if the mass is 0 we simply move the viewpoint to its equilibrium position.
    // If timeInterval is too large (longer than 1/10 of a second), the progression won't be smooth either way,
    // so in this case we simply move the viewpoint to the equilibrium position as well.
    if (forcePosition || this.viewpointMass === 0 || (timeInterval > 0.1 && this.animation == null)) {
      viewpointDeltaPosition = this.viewpointForce;
      this.viewpointVelocity = new x3dom.fields.SFVec3f(0.0, 0.0, 0.0);
    } else {
      var acceleration = this.viewpointForce.divide(this.viewpointMass);
      this.viewpointVelocity = this.viewpointVelocity.add(acceleration.multiply(timeInterval));
      var scalarVelocity = this.viewpointVelocity.length();

      // Velocity of the object projected onto the velocity of the viewpoint.
      var scalarObjectVelocityProjection;
      if (this.followedObjectDeltaPosition != null) {
        var objectVelocity = this.followedObjectDeltaPosition.divide(timeInterval);
        scalarObjectVelocityProjection = objectVelocity.dot(this.viewpointVelocity) / scalarVelocity;
      } else
        scalarObjectVelocityProjection = 0;

      // The viewpoint is going "faster" than the object, to prevent oscillations we apply a slowing force.
      if (this.viewpointFriction > 0 && scalarVelocity > scalarObjectVelocityProjection) {
        // We apply a friction based on the extra velocity.
        var velocityFactor = (scalarVelocity - (scalarVelocity - scalarObjectVelocityProjection) * this.viewpointFriction) / scalarVelocity;
        this.viewpointVelocity = this.viewpointVelocity.multiply(velocityFactor);
      }
      viewpointDeltaPosition = this.viewpointVelocity.multiply(timeInterval);
    }
    var viewpointNewPosition = viewpointPosition.add(viewpointDeltaPosition);
    this.viewpointForce = this.viewpointForce.subtract(viewpointDeltaPosition);
    viewpoints[0].setAttribute('position', viewpointNewPosition.toString());
    this.followedObjectDeltaPosition = null;
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
  if (this.mode !== 'video')
    return;
  this.video.width = width;
  this.video.height = height;
  this.stream.socket.send('resize: ' + width + 'x' + height);
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
  this.animation = new webots.Animation(url, this, gui, loop);
};

webots.View.prototype.applyPose = function(pose) {
  var id = pose.id;
  var el = document.getElementById('n' + id);
  if (el && !el.getAttribute('blockWebotsUpdate')) {
    for (var key in pose) {
      if (key !== 'id') {
        var value = pose[key];
        if (key === 'translation' && this.followedObject &&
            (id === this.followedObject || // animation case
             el.id === this.followedObject || // streaming case
             el.getAttribute('DEF') === this.followedObject)) {
          var objectPosition = x3dom.fields.SFVec3f.parse(el.getAttribute('translation'));
          el.setAttribute(key, value);
          // If this is the followed object, we save a vector with the translation applied
          // to the object to compute the new position of the viewpoint.
          var objectNewPosition = x3dom.fields.SFVec3f.parse(value);
          this.followedObjectDeltaPosition = objectNewPosition.subtract(objectPosition);
        } else
          el.setAttribute(key, value);
      }
    }
  }
};

webots.Animation = function(url, view, gui, loop) { // gui may be either "play" or "pause"
  this.url = url;
  this.view = view;
  this.gui = gui;
  this.loop = loop;
  this.sliding = false;
  this.onready = null;
};

webots.Animation.prototype.init = function(onready) {
  var that = this;
  this.onready = onready;
  var xmlhttp = new XMLHttpRequest();
  xmlhttp.open('GET', this.url, true);
  xmlhttp.overrideMimeType('application/json');
  xmlhttp.onreadystatechange = function() {
    if (xmlhttp.readyState === 4 && xmlhttp.status === 200)
      setup(JSON.parse(xmlhttp.responseText));
  };
  xmlhttp.send();
  function setup(data) {
    that.data = data;
    var div = document.createElement('div');
    div.id = 'playBar';
    that.view.view3D.appendChild(div);
    that.button = document.createElement('button');
    that.button.id = 'playPauseButton';
    var action = (that.gui === 'play') ? 'pause' : 'play';
    that.button.style.backgroundImage = 'url(' + webots.WwiUrl + action + '.png)';
    that.button.style.padding = '0';
    that.button.onclick = triggerPlayPauseButton;
    div.appendChild(that.button);
    var slider = document.createElement('div');
    slider.id = 'playSlider';
    div.appendChild(slider);
    that.playSlider = $('#playSlider').slider({
      change: function(e, ui) { updateSlider(ui.value); },
      slide: function(e, ui) { updateSlider(ui.value); },
      start: function(e, ui) { that.sliding = true; },
      stop: function(e, ui) { that.sliding = false; }
    });
    that.start = new Date().getTime();
    that.step = 0;
    that.previousStep = 0;
    updateAnimation();
    if (that.onready)
      that.onready();
  }

  function elapsedTime() {
    var end = new Date().getTime();
    return end - that.start;
  }

  function triggerPlayPauseButton() {
    that.button.style.backgroundImage = 'url(' + webots.WwiUrl + that.gui + '.png)';
    if (that.gui === 'play') {
      that.gui = 'pause';
      if (that.step < 0 || that.step >= that.data.frames.length) {
        that.start = new Date().getTime();
        updateAnimationState(true);
      } else
        that.start = new Date().getTime() - that.data.basicTimeStep * that.step;
    } else {
      that.gui = 'play';
      that.start = new Date().getTime() - that.data.basicTimeStep * that.step;
      requestAnimationFrame(updateAnimation);
    }
  }

  function connectSliderEvents() {
    that.playSlider = that.playSlider.slider({
      change: function(e, ui) { updateSlider(ui.value); },
      slide: function(e, ui) { updateSlider(ui.value); },
      start: function(e, ui) { that.sliding = true; },
      stop: function(e, ui) { that.sliding = false; }
    });
  }

  function disconnectSliderEvents() {
    that.playSlider.slider({change: null, slide: null});
  }

  function updateSlider(value) {
    that.step = Math.floor(that.data.frames.length * value / 100);
    that.start = (new Date().getTime()) - Math.floor(that.data.basicTimeStep * that.step);
    updateAnimationState(false);
  }

  function updateAnimationState(moveSlider) {
    if (moveSlider) {
      that.step = Math.floor(elapsedTime() / that.data.basicTimeStep);
      if (that.step < 0 || that.step >= that.data.frames.length) {
        if (that.loop) {
          if (that.step > that.data.frames.length) {
            that.step = 0;
            that.previousStep = 0;
            that.start = new Date().getTime();
          } else
            return;
        } else if (that.gui === 'play') {
          triggerPlayPauseButton();
          return;
        } else
          return;
      }
    }
    var p;
    var appliedIds = [];
    if (that.data.frames[that.step].hasOwnProperty('poses')) {
      var poses = that.data.frames[that.step].poses;
      for (p = 0; p < poses.length; p++) {
        that.view.applyPose(poses[p]);
        appliedIds[appliedIds.length] = poses[p].id;
      }
    }
    // lookback mechanism: search in history
    if (that.step !== that.previousStep + 1) {
      var previousPoseStep;
      if (that.step > that.previousStep)
        // in forward animation check only the changes since last pose
        previousPoseStep = that.previousStep;
      else
        previousPoseStep = 0;
      var allIds = that.data.ids.split(';');
      for (var i = 0; i < allIds.length; i++) {
        var id = parseInt(allIds[i]);
        if (appliedIds.indexOf(id) === -1) {
          outer:
          for (var f = that.step - 1; f >= previousPoseStep; f--) {
            if (that.data.frames[f].poses) {
              for (p = 0; p < that.data.frames[f].poses.length; p++) {
                if (that.data.frames[f].poses[p].id === id) {
                  that.view.applyPose(that.data.frames[f].poses[p]);
                  break outer;
                }
              }
            }
          }
        }
      }
    }
    if (moveSlider) {
      disconnectSliderEvents();
      that.playSlider.slider('option', 'value', 100 * that.step / that.data.frames.length);
      connectSliderEvents();
    }
    that.previousStep = that.step;
    that.view.time = that.data.frames[that.step].time;
    if (that.view.followedObject != null && that.view.followedObject !== 'none')
      that.view.updateViewpointPosition(!moveSlider | that.step === 0);
  }

  function updateAnimation() {
    if (that.gui === 'play') {
      updateAnimationState(true);
      requestAnimationFrame(updateAnimation);
    }
  }
};

webots.Server = function(url, view, onready) {
  var that = this;
  this.view = view;
  this.onready = onready;
  // url has the following form: "ws(s)://cyberbotics2.cyberbotics.com:80/simple/worlds/simple.wbt"
  var n = url.indexOf('/', 6);
  var m = url.lastIndexOf('/');
  this.url = 'http' + url.substring(2, n); // e.g., "http(s)://cyberbotics2.cyberbotics.com:80"
  this.project = url.substring(n + 1, m - 7); // e.g., "simple"
  this.worldFile = url.substring(m + 1); // e.g., "simple.wbt"
  this.controllers = [];
  var xhr = new XMLHttpRequest();
  xhr.open('GET', this.url + '/session', true);
  $('#webotsProgressMessage').html('Connecting to session server...');
  xhr.onreadystatechange = function(e) {
    if (xhr.readyState !== 4)
      return;
    if (xhr.status !== 200)
      return;
    var data = xhr.responseText;
    if (data.startsWith('Error:')) {
      $('#webotsProgress').hide();
      var errorMessage = data.substring(6).trim();
      errorMessage = errorMessage.charAt(0).toUpperCase() + errorMessage.substring(1);
      webots.alert('Session server error', errorMessage);
      return;
    }
    that.socket = new WebSocket(data + '/client');
    that.socket.onopen = function(event) {
      var host = location.protocol + '//' + location.host.replace(/^www./, ''); // remove 'www' prefix
      if (typeof webots.User1Id === 'undefined')
        webots.User1Id = '';
      if (typeof webots.User1Name === 'undefined')
        webots.User1Name = '';
      if (typeof webots.User1Authentication === 'undefined')
        webots.User1Authentication = '';
      if (typeof webots.User2Id === 'undefined')
        webots.User2Id = '';
      if (typeof webots.User2Name === 'undefined')
        webots.User2Name = '';
      if (typeof webots.CustomData === 'undefined')
        webots.CustomData = '';
      this.send('{ "init" : [ "' + host + '", "' + that.project + '", "' + that.worldFile + '", "' +
                webots.User1Id + '", "' + webots.User1Name + '", "' + webots.User1Authentication + '", "' +
                webots.User2Id + '", "' + webots.User2Name + '", "' + webots.CustomData + '" ] }');
      $('#webotsProgressMessage').html('Starting simulation...');
    };
    that.socket.onclose = function(event) {
      view.console.info('Disconnected to the Webots server.');
    };
    that.socket.onmessage = function(event) {
      var message = event.data;
      if (message.indexOf('webots:ws://') === 0 || message.indexOf('webots:wss://') === 0)
        view.stream = new webots.Stream(message.substring(7), view, that.onready);
      else if (message.indexOf('controller:') === 0) {
        var n = message.indexOf(':', 11);
        var controller = {};
        controller.name = message.substring(11, n);
        controller.port = message.substring(n + 1);
        view.console.info('Using controller ' + controller.name + ' on port ' + controller.port);
        that.controllers.push(controller);
      } else if (message.indexOf('queue:') === 0)
        view.console.error('The server is saturated. Queue to wait: ' + message.substring(6) + ' client(s).');
      else if (message === '.') { // received every 5 seconds when Webots is running
        // nothing to do
      } else if (message.indexOf('reset controller:') === 0)
        view.stream.socket.send('sync controller:' + message.substring(18).trim());
      else
        console.log('Received an unknown message from the Webots server socket: "' + message + '"');
    };
    that.socket.onerror = function(event) {
      view.console.error('Cannot connect to the simulation server');
    };
  };
  xhr.send();
};

webots.Server.prototype.resetController = function(filename) {
  this.socket.send('{ "reset controller" : "' + filename + '" }');
};

webots.Stream = function(url, view, onready) {
  var that = this;
  this.view = view;
  this.onready = onready;

  this.socket = new WebSocket(url);
  $('#webotsProgressMessage').html('Connecting to Webots instance...');
  this.socket.onopen = function() {
    var mode = that.view.mode;
    if (mode === 'video')
      mode += ': ' + that.view.video.width + 'x' + that.view.video.height;
    else if (that.view.broadcast)
      mode += ';broadcast';
    that.socket.send(mode);
  };
  this.socket.onclose = function(event) {
    view.onerror('Disconnected from ' + url + ' (' + event.code + ')');
    if ((event.code > 1001 && event.code < 1016) || (event.code === 1001 && view.quitting === false)) { // https://tools.ietf.org/html/rfc6455#section-7.4.1
      webots.alert('Streaming server error',
        'Connection closed abnormally.<br>(Error code: ' + event.code + ')<br><br>' +
        'Please reset the simulation by clicking ' +
        '<a href="' + window.location.href + '">here</a>.');
    }
    destroyWorld();
    if (view.onclose)
      view.onclose();
  };
  this.socket.onmessage = function(event) {
    var lines, i;
    var data = event.data;
    if (data.startsWith('robot:') ||
        data.startsWith('stdout:') ||
        data.startsWith('stderr:')) {
      lines = data.split('\n'); // in that case, we support one message per line
      for (i = 0; i < lines.length; i++) {
        var line = lines[i];
        if (line === '') // FIXME: should not happen
          continue;
        if (line.startsWith('stdout:'))
          view.console.stdout(line.substring(7));
        else if (line.startsWith('stderr:'))
          view.console.stderr(line.substring(7));
        else if (line.startsWith('robot:')) {
          var secondColonIndex = line.indexOf(':', 6);
          var robot = line.substring(6, secondColonIndex);
          var message = line.substring(secondColonIndex + 1);
          that.view.onrobotmessage(robot, message);
        }
      }
    } else if (data.startsWith('application/json:')) {
      if (that.view.time !== undefined) { // otherwise ignore late updates until the scene loading is completed
        data = data.substring(data.indexOf(':') + 1);
        var frame = JSON.parse(data);
        that.view.time = frame.time;
        $('#webotsClock').html(webots.parseMillisecondsIntoReadableTime(frame.time));
        if (frame.hasOwnProperty('poses')) {
          for (i = 0; i < frame.poses.length; i++)
            that.view.applyPose(frame.poses[i]);
        }
        if (that.view.followedObject != null && that.view.followedObject !== 'none')
          that.view.updateViewpointPosition();
      }
    } else if (data.startsWith('node:')) {
      data = data.substring(data.indexOf(':') + 1);
      var parentId = data.split(':')[0];
      data = data.substring(data.indexOf(':') + 1);
      var parser = new DOMParser();
      var x3d = parser.parseFromString(data, 'text/xml').children[0];
      if (parentId === '0')
        that.view.x3dScene.appendChild(x3d);
      else
        document.getElementById('n' + parentId).appendChild(x3d);
    } else if (data.startsWith('delete:')) {
      data = data.substring(data.indexOf(':') + 1).trim();
      var itemToDelete = document.getElementById('n' + data);
      if (itemToDelete) {
        if (that.selection === itemToDelete)
          that.selection = null;
        itemToDelete.parentElement.removeChild(itemToDelete);
      }
    } else if (data.startsWith('model:')) {
      $('#webotsProgressMessage').html('Loading 3D scene...');
      destroyWorld();
      data = data.substring(data.indexOf(':') + 1).trim();
      if (!data) // received an empty model case: just destroy the view
        return;
      var scene = data.substring(data.indexOf('<Scene>') + 8, data.lastIndexOf('</Scene>'));
      $(that.view.x3dScene).append(scene);
      that.view.onresize();
    } else if (data.startsWith('image')) {
      // extract texture url: the url should only contains escaped ']' characters
      var urlPattern = /[^\\]\]/g; // first occurrence of non-escaped ']'
      var match = urlPattern.exec(data);
      var textureUrlEndIndex = match.index + 1;
      var textureUrl = data.substring(data.indexOf('[') + 1, textureUrlEndIndex).replace(/\\]/g, ']');
      data = data.substring(data.indexOf(':', textureUrlEndIndex) + 1);
      // replace in ImageTexture nodes
      var textures = that.view.x3dScene.getElementsByTagName('ImageTexture');
      for (i = 0; i < textures.length; i++) {
        var texture = textures[i];
        // getFieldValue is not defined for USE nodes
        if (typeof texture.getFieldValue === 'function' && that.compareTextureUrl(texture.getFieldValue('url'), textureUrl))
          texture.setFieldValue('url', [data]);
      }
      // replace in Background nodes
      var backgrounds = that.view.x3dScene.getElementsByTagName('Background');
      var backgroundUrlFieldNames = ['frontUrl', 'backUrl', 'leftUrl', 'rightUrl', 'topUrl', 'bottomUrl'];
      for (i = 0; i < backgrounds.length; i++) {
        var background = backgrounds[i];
        for (var j = 0; j < backgroundUrlFieldNames.length; j++) {
          var backgroundUrlFieldName = backgroundUrlFieldNames[j];
          if (typeof background.getFieldValue === 'function' && that.compareTextureUrl(background.getFieldValue(backgroundUrlFieldName), textureUrl))
            background.setFieldValue(backgroundUrlFieldName, [data]);
        }
      }
    } else if (data.startsWith('video: ')) {
      console.log('Received data = ' + data);
      var list = data.split(' ');
      var url = list[1];
      var streamId = list[2];
      console.log('Received video message on ' + url + ' stream = ' + streamId);
      that.VideoStream = new webots.VideoStream(url, view.video, document.getElementById('BitrateViewer'), streamId);
      if (that.onready)
        that.onready();
    } else if (data.startsWith('set controller:')) {
      var slash = data.indexOf('/', 15);
      var dirname = data.substring(15, slash);
      var filename = data.substring(slash + 1, data.indexOf(':', slash + 1));
      if (that.view.editor.dirname === dirname)
        that.view.editor.addFile(filename, data.substring(data.indexOf('\n') + 1)); // remove the first line
      else
        console.log('Warning: ' + filename + ' not in controller directory: ' + dirname + ' != ' + that.view.editor.dirname);
    } else if (data === 'pause') {
      that.view.pauseButton.style.display = 'none';
      that.view.real_timeButton.style.display = 'inline';
      if (that.view.timeout > 0 && !that.view.isAutomaticallyPaused) {
        that.view.deadline = that.view.timeout;
        if (that.view.time !== undefined)
          that.view.deadline += that.view.time;
        $('#webotsTimeout').html(webots.parseMillisecondsIntoReadableTime(that.view.deadline));
      }
    } else if (data === 'real-time' || data === 'run' || data === 'fast') {
      that.view.pauseButton.style.display = 'inline';
      that.view.real_timeButton.style.display = 'none';
      if (that.view.timeout >= 0)
        that.view.stream.socket.send('timeout:' + that.view.timeout);
    } else if (data === 'scene load completed') {
      that.view.time = 0;
      $('#webotsClock').html(webots.parseMillisecondsIntoReadableTime(0));
      if (that.onready)
        that.onready();
    } else if (data === 'reset finished') {
      // remove labels
      var labels = document.getElementsByClassName('webotsLabel');
      for (i = labels.length - 1; i >= 0; i--) {
        var element = labels.item(i);
        element.parentNode.removeChild(element);
      }
      $('#webotsClock').html(webots.parseMillisecondsIntoReadableTime(0));
      if (that.onready)
        that.onready();
      that.view.deadline = that.view.timeout;
      $('#webotsTimeout').html(webots.parseMillisecondsIntoReadableTime(that.view.deadline));
      // restore viewpoint
      var viewpoint = that.view.x3dScene.getElementsByTagName('Viewpoint')[0];
      viewpoint.setAttribute('position', that.view.initialViewpointPosition);
      viewpoint.setAttribute('orientation', that.view.initialViewpointOrientation);
      that.view.updateViewpointPosition(true);
    } else if (data.startsWith('label')) {
      var semiColon = data.indexOf(';');
      var id = data.substring(data.indexOf(':'), semiColon);
      var previousSemiColon;
      var labelProperties = []; // ['font', 'color', 'size', 'x', 'y', 'text']
      for (i = 0; i < 5; i++) {
        previousSemiColon = semiColon + 1;
        semiColon = data.indexOf(';', previousSemiColon);
        labelProperties.push(data.substring(previousSemiColon, semiColon));
      }
      var text = data.substring(semiColon + 1, data.length);
      var labelElement = document.getElementById('label' + id);
      if (labelElement == null) {
        labelElement = document.createElement('div');
        labelElement.id = 'label' + id;
        labelElement.className = 'webotsLabel';
        that.view.x3dNode.appendChild(labelElement);
      }
      labelElement.style.fontFamily = labelProperties[0];
      labelElement.style.color = labelProperties[1];
      labelElement.style.fontSize = $(that.view.x3dNode).height() * labelProperties[2] / 2.25 + 'px'; // 2.25 is an empirical value to match with Webots appearance
      labelElement.style.left = $(that.view.x3dNode).width() * labelProperties[3] + 'px';
      labelElement.style.top = $(that.view.x3dNode).height() * labelProperties[4] + 'px';
      labelElement.innerHTML = text;
    } else
      console.log('WebSocket error: Unknown message received: "' + data + '"');
  };
  this.socket.onerror = function(event) {
    destroyWorld();
    view.onerror('WebSocket error: ' + event.data);
  };
  function destroyWorld() {
    that.view.selection = null;
    if (that.view.x3dScene) {
      while (that.view.x3dScene.hasChildNodes())
        // remove from last to avoid issues with USE/DEF nodes
        that.view.x3dScene.removeChild(that.view.x3dScene.lastChild);
    }

    // remove labels
    var labels = document.getElementsByClassName('webotsLabel');
    for (var i = labels.length - 1; i >= 0; i--) {
      var element = labels.item(i);
      element.parentNode.removeChild(element);
    }
  }
};

webots.Stream.prototype.compareTextureUrl = function(attributeUrl, textureUrl) {
  // url attribute is an array (x3dom.field.MFString)
  if (!attributeUrl || !textureUrl || (typeof attributeUrl !== 'object') || !(attributeUrl instanceof Array))
    return false;

  var length = attributeUrl.length;
  for (var i = 0; i < length; i++) {
    if (attributeUrl[i] === textureUrl)
      return true;
  }
  return false;
};

webots.Stream.prototype.close = function() {
  if (this.socket)
    this.socket.close();
  if (this.videoStream)
    this.videoStream.close();
};

function webotsClampDialogSize(preferredGeometry) {
  if ($('#playerDiv').height === undefined || $('#playerDiv').width === undefined)
    return preferredGeometry;

  var maxHeight = $('#playerDiv').height() - preferredGeometry.top - $('#toolBar').height() - 20; // 20 is chosen arbitrarily
  var maxWidth = $('#playerDiv').width() - preferredGeometry.left - 20; // 20 is chosen arbitrarily
  var height = preferredGeometry.height;
  var width = preferredGeometry.width;
  if (maxHeight < height)
    height = maxHeight;
  if (maxWidth < width)
    width = maxWidth;
  return {width: width, height: height};
}

function webotsResizeDialogOnOpen(dialog) {
  var w = $(dialog).parent().width();
  var h = $(dialog).parent().height();
  var clampedSize = webotsClampDialogSize({left: 0, top: 0, width: w, height: h});
  if (clampedSize.width < w)
    $(dialog).dialog('option', 'width', clampedSize.width);
  if (clampedSize.height < h)
    $(dialog).dialog('option', 'height', clampedSize.height);
}

function webotsOpenDialog() {
  webotsResizeDialogOnOpen(this);
  $(this).parent().css('opacity', 0.9);
  $(this).parent().hover(function() {
    $(this).css('opacity', 0.99);
  }, function(event) {
    $(this).css('opacity', 0.9);
  });
}

function webotsMobileCreateDialog() {
  // mobile only setup
  var closeButton = $('button:contains("WbClose")');
  closeButton.html('');
  closeButton.removeClass('ui-button-text-only');
  closeButton.addClass('mobile-dialog-close-button');
  closeButton.addClass('ui-button-icon-primary');
  closeButton.prepend('<span class="ui-icon ui-icon-closethick"></span>');
}

function webotsAddMobileDialogAttributes(params, panel) {
  params.dialogClass = 'mobile-no-default-buttons';
  params.create = webotsMobileCreateDialog;
  params.buttons = { 'WbClose': function() { $(panel).dialog('close'); } };
}

// the following two functions are used to make the resize and drag of the dialog
// steady (i.e., not loose the grab while resizing/dragging the dialog quickly)
function webotsDisablePointerEvents() {
  document.body.style['pointer-events'] = 'none';
}
function webotsEnablePointerEvents() {
  document.body.style['pointer-events'] = 'auto';
}

webots.Editor = function(parent, view) {
  var that = this;
  function webotsEditorResize() {
    var padding = $('#webotsEditorTab').outerHeight() - $('#webotsEditorTab').height();
    $('#webotsEditorTab').height(that.tabs.clientHeight - that.tabsHeader.scrollHeight - padding);
    that.editor.resize();
  }
  function hideMenu() {
    if ($('#webotsEditorMenu').hasClass('pressed'))
      $('#webotsEditorMenu').removeClass('pressed');
  }
  function openResetConfirmDialog(allFiles) {
    that.resetAllFiles = allFiles;
    var titleText, message;
    message = 'Permanently reset ';
    if (allFiles) {
      message += 'all the files';
      titleText = 'Reset files?';
    } else {
      message += 'this file';
      titleText = 'Reset file?';
    }
    message += ' to the original version?';
    message += '<br/><br/>Your modifications will be lost.';
    var confirmDialog = document.createElement('div');
    that.panel.appendChild(confirmDialog);
    $(confirmDialog).html(message);
    $(confirmDialog).dialog({
      title: titleText,
      modal: true,
      autoOpen: true,
      resizable: false,
      dialogClass: 'alert',
      open: webotsOpenDialog,
      appendTo: that.parent,
      buttons: {
        'Cancel': function() {
          $(this).dialog('close');
          $('#webotsEditorConfirmDialog').remove();
        },
        'Reset': function() {
          $(this).dialog('close');
          $('#webotsEditorConfirmDialog').remove();
          if (that.resetAllFiles) {
            for (var i = 0; i < that.filenames.length; i++)
              that.view.server.resetController(that.dirname + '/' + that.filenames[i]);
          } else
            that.view.server.resetController(that.dirname + '/' + that.filenames[that.currentSession]);
        }
      }
    });
    hideMenu();
  }
  this.view = view;
  this.filenames = [];
  this.needToUploadFiles = [];
  this.sessions = [];
  this.panel = document.createElement('div');
  this.panel.id = 'webotsEditor';
  this.panel.className = 'webotsTabContainer';
  that.parent = parent;
  parent.appendChild(this.panel);
  var clampedSize = webotsClampDialogSize({left: 0, top: 0, width: 800, height: 600});
  var params = {
    title: 'Editor',
    resize: webotsEditorResize,
    resizeStart: webotsDisablePointerEvents,
    resizeStop: webotsEnablePointerEvents,
    dragStart: webotsDisablePointerEvents,
    dragStop: webotsEnablePointerEvents,
    width: clampedSize.width,
    height: clampedSize.height,
    autoOpen: false,
    appendTo: parent,
    open: function() {
      webotsResizeDialogOnOpen(that.panel);
    }
  };
  if (this.view.mobileDevice)
    webotsAddMobileDialogAttributes(params, this.panel);
  $(this.panel).dialog(params).dialogExtend({maximizable: !this.view.mobileDevice});
  var edit = document.createElement('div');
  edit.id = 'webotsEditorTab';
  edit.className = 'webotsTab';
  this.editor = ace.edit(edit);
  this.sessions[0] = this.editor.getSession();
  this.currentSession = 0;
  this.tabs = document.createElement('div');
  this.tabs.id = 'webotsEditorTabs';
  this.tabs.className = 'webotsTabs';
  this.tabsHeader = document.createElement('ul');
  this.tabs.appendChild(this.tabsHeader);
  this.tabs.appendChild(edit);
  $(this.tabs).tabs({activate: function(event, ui) {
    that.currentSession = parseInt(ui.newTab.attr('id').substr(5)); // skip 'file-'
    that.editor.setSession(that.sessions[that.currentSession]);
  }});
  this.panel.appendChild(this.tabs);
  this.menu = document.createElement('div');
  this.menu.id = 'webotsEditorMenu';
  var saveShortcut;
  if (navigator.appVersion.indexOf('Mac') === -1)
    saveShortcut = 'Ctrl-S';
  else // macOS
    saveShortcut = 'Cmd-S';
  this.menu.innerHTML = '<input type="image" id="webotsEditorMenuImage" width="17px" src="' + webots.WwiUrl + '/images/menu.png">' +
                        '<div id="webotsEditorMenuContent">' +
                        '<div id="webotsEditorSaveAction" class="webotsEditorMenuContentItem" title="Save current file">Save<span style="float:right"><i><small>' + saveShortcut + '</small></i></span></div>' +
                        '<div id="webotsEditorSaveAllAction" class="webotsEditorMenuContentItem" title="Save all the files">Save All</div>' +
                        '<div id="webotsEditorResetAction" class="webotsEditorMenuContentItem" title="Reset current file to the original version">Reset</div>' +
                        '<div id="webotsEditorResetAllAction" class="webotsEditorMenuContentItem" title="Reset all the files to the original version">Reset All</div>' +
                        '</div>';
  this.panel.appendChild(this.menu);
  this.editor.commands.addCommand({
    name: 'save',
    bindKey: {win: 'Ctrl-S', mac: 'Cmd-S'},
    exec: function(editor) {
      that.save(that.currentSession);
    }
  });
  $('#webotsEditorSaveAction').click(function() {
    that.save(that.currentSession);
    hideMenu();
  });
  $('#webotsEditorSaveAllAction').click(function() {
    for (var i = 0; i < that.filenames.length; i++)
      that.save(i);
    hideMenu();
  });
  $('#webotsEditorResetAction').click(function() {
    openResetConfirmDialog(false);
  });
  $('#webotsEditorResetAllAction').click(function() {
    openResetConfirmDialog(true);
  });
  $('#webotsEditorMenuImage').click(function() {
    if ($('#webotsEditorMenu').hasClass('pressed'))
      $('#webotsEditorMenu').removeClass('pressed');
    else
      $('#webotsEditorMenu').addClass('pressed');
  });
  $('#webotsEditorMenu').focusout(function() {
    // let the time to handle the menu actions if needed
    window.setTimeout(function() {
      if ($('.webotsEditorMenuContentItem:hover').length > 0)
        return;
      if ($('#webotsEditorMenu').hasClass('pressed'))
        $('#webotsEditorMenu').removeClass('pressed');
    }, 100);
  });
};

webots.Editor.prototype.hasUnsavedChanges = function() {
  for (var i = 0; i < this.filenames.length; i++) {
    if ($('#filename-' + i).html().endsWith('*'))
      return true;
  }
  return false;
};

webots.Editor.prototype.storeUserFile = function(i) {
  var formData = new FormData();
  formData.append('dirname', this.view.server.project + '/controllers/' + this.dirname);
  formData.append('filename', this.filenames[i]);
  formData.append('content', this.sessions[i].getValue());
  $.ajax({
    url: '/ajax/upload-file.php',
    type: 'POST',
    data: formData,
    processData: false,
    contentType: false,
    success: function(data) {
      if (data !== 'OK')
        webots.alert('File saving error', data);
    }
  });
};

webots.Editor.prototype.upload = function(i) { // upload to the simulation server
  this.view.stream.socket.send('set controller:' +
    this.dirname + '/' +
    this.filenames[i] + ':' +
    this.sessions[i].getLength() + '\n' +
    this.sessions[i].getValue());
  this.needToUploadFiles[i] = false;
};

webots.Editor.prototype.save = function(i) { // save to the web site
  if ($('#filename-' + i).html().endsWith('*')) { // file was modified
    $('#filename-' + i).html(this.filenames[i]);
    this.needToUploadFiles[i] = true;
    if (webots.User1Id && webots.User1Authentication) // user logged in
      this.storeUserFile(i);
    else
      this.view.unloggedFileModified = true;

    if (this.view.time === 0)
      this.upload(i);
    else {
      if (!this.statusMessage) {
        this.statusMessage = document.createElement('div');
        this.statusMessage.id = 'webotsEditorStatusMessage';
        this.statusMessage.className = 'webotsEditorStatusMessage';
        this.statusMessage.innerHTML = '<font size="2">Reset the simulation to apply the changes.</font>';
      }
      this.panel.appendChild(this.statusMessage);
      setTimeout(this.hideResetMessage, 1500);
    }
  }
};

webots.Editor.prototype.hideResetMessage = function() {
  $('#webotsEditorStatusMessage').remove();
};

webots.Editor.prototype.textChange = function(index) {
  if (!$('#filename-' + index).html().endsWith('*') && this.editor.curOp && this.editor.curOp.command.name) { // user change
    $('#filename-' + index).html(this.filenames[index] + '*');
  }
};

webots.Editor.prototype.aceMode = function(filename) {
  if (filename.toLowerCase() === 'makefile')
    return 'ace/mode/makefile';
  var extension = filename.split('.').pop().toLowerCase();
  if (extension === 'py')
    return 'ace/mode/python';
  if (extension === 'c' || extension === 'cpp' || extension === 'c++' || extension === 'cxx' || extension === 'cc' ||
      extension === 'h' || extension === 'hpp' || extension === 'h++' || extension === 'hxx' || extension === 'hh')
    return 'ace/mode/c_cpp';
  if (extension === 'java')
    return 'ace/mode/java';
  if (extension === 'm')
    return 'ace/mode/matlab';
  if (extension === 'json')
    return 'ace/mode/json';
  if (extension === 'xml')
    return 'ace/mode/xml';
  if (extension === 'yaml')
    return 'ace/mode/yaml';
  if (extension === 'ini')
    return 'ace/mode/ini';
  if (extension === 'html')
    return 'ace/mode/html';
  if (extension === 'js')
    return 'ace/mode/javascript';
  if (extension === 'css')
    return 'ace/mode/css';
  return 'ace/mode/text';
};

webots.Editor.prototype.addFile = function(filename, content) {
  var index = this.filenames.indexOf(filename);
  if (index >= 0) {
    this.needToUploadFiles[index] = false; // just received from the simulation server
    this.sessions[index].setValue(content);
    if ($('#filename-' + index).html().endsWith('*'))
      $('#filename-' + index).html(filename);
    if (webots.User1Authentication && webots.User1Id)
      this.storeUserFile(index);
    return;
  }

  index = this.filenames.length;
  this.filenames.push(filename);
  this.needToUploadFiles[index] = false;
  if (index === 0) {
    this.sessions[index].setMode(this.aceMode(filename));
    this.sessions[index].setValue(content);
    $('#webotsEditorMenu').show();
    $('#webotsEditorTabs').show();
  } else
    this.sessions.push(ace.createEditSession(content, this.aceMode(filename)));
  var that = this;
  this.sessions[index].on('change', function(e) { that.textChange(index); });
  $('div#webotsEditorTabs ul').append('<li id="file-' + index + '"><a href="#webotsEditorTab" id="filename-' + index + '">' + filename + '</a></li>');
  $('div#webotsEditorTabs').tabs('refresh');
  if (index === 0)
    $('div#webotsEditorTabs').tabs('option', 'active', index);
};

webots.Editor.prototype.closeAllTabs = function() {
  this.editor.setSession(ace.createEditSession('', ''));
  this.filenames = [];
  this.needToUploadFiles = [];
  this.sessions = [];
  this.sessions[0] = this.editor.getSession();
  this.currentSession = 0;
  $('div#webotsEditorTabs ul').empty();
  $('#webotsEditorMenu').hide();
  $('#webotsEditorTabs').hide();
};

webots.Console = function(parent, mobile) {
  function closeConsole() {
    $('#consoleButton').removeClass('toolBarButtonActive');
  }
  this.panel = document.createElement('div');
  this.panel.id = 'webotsConsole';
  this.panel.className = 'webotsConsole';
  parent.appendChild(this.panel);
  var clampedSize = webotsClampDialogSize({left: 0, top: 0, width: 600, height: 400});
  var params = {
    title: 'Console',
    resizeStart: webotsDisablePointerEvents,
    resizeStop: webotsEnablePointerEvents,
    dragStart: webotsDisablePointerEvents,
    dragStop: webotsEnablePointerEvents,
    width: clampedSize.width,
    height: clampedSize.height,
    autoOpen: false,
    appendTo: parent,
    close: closeConsole,
    open: webotsOpenDialog
  };
  if (mobile)
    webotsAddMobileDialogAttributes(params, this.panel);
  $(this.panel).dialog(params).dialogExtend({maximizable: mobile});
};

webots.Console.prototype.scrollDown = function() {
  if (this.panel)
    this.panel.scrollTop = this.panel.scrollHeight;
};

webots.Console.prototype.clear = function() {
  if (this.panel) {
    while (this.panel.firstChild)
      this.panel.removeChild(this.panel.firstChild);
  } else
    console.clear();
};

webots.Console.prototype.log = function(message, type) {
  var para = document.createElement('p');
  var style = 'margin:0;';
  var title = '';
  switch (type) {
    case 0:
      style += 'color:Blue;';
      title = 'Webots stdout';
      break;
    case 1:
      style += 'color:Red;';
      title = 'Webots stderr';
      break;
    case 2:
      style += 'color:Gray;';
      title = 'info';
      break;
    case 3:
      style += 'color:Salmon;';
      title = 'error';
      break;
  }
  if (this.panel) {
    para.style.cssText = style;
    para.title = title + ' (' + hourString() + ')';
    var t = document.createTextNode(message);
    para.appendChild(t);
    this.panel.appendChild(para);
    this.scrollDown();
  } else
    console.log('%c' + message, style);
  function hourString() {
    var d = new Date();
    return d.getHours() + ':' +
         ((d.getMinutes() < 10) ? '0' : '') + d.getMinutes() + ':' +
         ((d.getSeconds() < 10) ? '0' : '') + d.getSeconds();
  }
};

webots.Console.prototype.stdout = function(message) {
  this.log(message, 0);
};

webots.Console.prototype.stderr = function(message) {
  this.log(message, 1);
};

webots.Console.prototype.info = function(message) {
  this.log(message, 2);
};

webots.Console.prototype.error = function(message) {
  this.log(message, 3);
};

webots.HelpWindow = function(parent, webotsDocUrl, mobile) {
  function closeConsole() {
    $('#helpButton').removeClass('toolBarButtonActive');
  }
  function finalize() {
    $('#webotsHelpTabs').tabs('refresh');
    $('#webotsHelpTabs').tabs('option', 'active', 0);
    $(that.panel).dialog('open');
  }
  var that = this;
  this.name = name;
  this.panel = document.createElement('div');
  this.panel.id = 'webotsHelp';
  that.panel.style.overflow = 'hidden';
  this.panel.className += 'webotsTabContainer';
  this.tabs = document.createElement('div');
  this.tabs.id = 'webotsHelpTabs';
  this.tabs.className += 'webotsTabs';
  this.tabsHeader = document.createElement('ul');
  this.tabs.appendChild(this.tabsHeader);
  this.panel.appendChild(this.tabs);
  parent.appendChild(this.panel);
  var clampedSize = webotsClampDialogSize({left: 5, top: 5, width: 600, height: 600});
  var params = {
    title: 'Help',
    resizeStart: webotsDisablePointerEvents,
    resizeStop: webotsEnablePointerEvents,
    dragStart: webotsDisablePointerEvents,
    dragStop: webotsEnablePointerEvents,
    autoOpen: false,
    appendTo: parent,
    close: closeConsole,
    open: webotsOpenDialog,
    position: {at: 'right-5 top+5', my: 'right top', of: parent},
    width: clampedSize.width,
    height: clampedSize.height
  };
  if (mobile)
    webotsAddMobileDialogAttributes(params, this.panel);
  $(this.panel).dialog(params).dialogExtend({maximizable: mobile});

  if (webotsDocUrl) {
    var header = document.createElement('li');
    header.innerHTML = '<a href="#webotsHelpReference">Webots Reference Manual</a>';
    that.tabsHeader.appendChild(header);
    var page = document.createElement('div');
    page.id = 'webotsHelpReference';
    page.innerHTML = '<iframe src="' + webotsDocUrl + '"></iframe>';
    that.tabs.appendChild(page);
    $('#webotsHelpTabs').tabs();
  }

  $.ajax({
    url: webots.currentScriptPath() + 'help.php',
    success: function(data) {
      // we need to fix the img src relative URLs
      var html = data.replace(/ src="images/g, ' src="' + webots.currentScriptPath() + '/images');
      var header = document.createElement('li');
      header.innerHTML = '<a href="#webotsHelpGuide">User Guide</a>';
      $(that.tabsHeader).prepend(header);
      var page = document.createElement('div');
      page.id = 'webotsHelpGuide';
      page.innerHTML = html;
      if (document.getElementById('webotsHelpReference'))
        $('#webotsHelpReference').before(page);
      else {
        that.tabs.appendChild(page);
        $('#webotsHelpTabs').tabs();
      }
      finalize();
    },
    error: function() {
      finalize();
    }
  });
};

webots.RobotWindow = function(parent, name, mobile) {
  this.name = name;
  this.panel = document.createElement('div');
  this.panel.id = name;
  this.panel.className = 'webotsTabContainer';
  parent.appendChild(this.panel);
  var clampedSize = webotsClampDialogSize({left: 5, top: 5, width: 400, height: 400});
  var params = {
    title: 'Robot Window',
    resizeStart: webotsDisablePointerEvents,
    resizeStop: webotsEnablePointerEvents,
    dragStart: webotsDisablePointerEvents,
    dragStop: webotsEnablePointerEvents,
    autoOpen: false,
    appendTo: parent,
    open: webotsOpenDialog,
    position: {at: 'left+5 top+5', my: 'left top', of: parent},
    width: clampedSize.width,
    height: clampedSize.height
  };
  if (mobile)
    webotsAddMobileDialogAttributes(params, this.panel);
  $(this.panel).dialog(params).dialogExtend({maximizable: !mobile});
};

webots.RobotWindow.prototype.setProperties = function(properties) {
  $(this.panel).dialog(properties);
};

webots.RobotWindow.prototype.geometry = function() {
  var webotsTabs = this.panel.getElementsByClassName('webotsTabs');
  var activeTabIndex = -1;
  if (webotsTabs.length > 0)
    activeTabIndex = $(webotsTabs[0]).tabs('option', 'active');
  return {
    width: $(this.panel).dialog('option', 'width'),
    height: $(this.panel).dialog('option', 'height'),
    position: $(this.panel).dialog('option', 'position'),
    activeTabIndex: activeTabIndex,
    open: this.isOpen()
  };
};

webots.RobotWindow.prototype.restoreGeometry = function(data) {
  $(this.panel).dialog({
    width: data.width,
    height: data.height,
    position: data.position
  });
  var webotsTabs = this.panel.getElementsByClassName('webotsTabs');
  if (data.activeTabIndex >= 0 && webotsTabs.length > 0)
    $(webotsTabs[0]).tabs('option', 'active', data.activeTabIndex);
};

webots.RobotWindow.prototype.destroy = function() {
  this.close();
  this.panel.parentNode.removeChild(this.panel);
  this.panel = null;
};

webots.RobotWindow.prototype.setContent = function(content) {
  $(this.panel).html(content);
};

webots.RobotWindow.prototype.open = function() {
  $(this.panel).dialog('open');
};

webots.RobotWindow.prototype.isOpen = function() {
  return $(this.panel).dialog('isOpen');
};

webots.RobotWindow.prototype.close = function() {
  $(this.panel).dialog('close');
};

webots.RobotWindow.prototype.send = function(message, robot) {
  webots.currentView.stream.socket.send('robot:' + robot + ':' + message);
  if (webots.currentView.real_timeButton.style.display === 'inline') // if paused, make a simulation step
    webots.currentView.stream.socket.send('step'); // so that the robot controller handles the message
  // FIXME: there seems to be a bug here: after that step, the current time is not incremented in the web interface,
  // this is because the next 'application/json:' is not received, probably because it gets overwritten by the
  // answer to the robot message...
};

webots.RobotWindow.prototype.receive = function(message, robot) { // to be overriden
  console.log("Robot window '" + this.name + "' received message from Robot '" + robot + "': " + message);
};

webots.window = function(name) {
  var win = webots.currentView.robotWindows[name];
  if (!win)
    console.log("Robot window '" + name + "' not found.");
  return win;
};

webots.alert = function(title, message, callback) {
  webots.currentView.ondialogwindow(true);
  var parent = webots.currentView.view3D;
  var panel = document.getElementById('webotsAlert');
  if (!panel) {
    panel = document.createElement('div');
    panel.id = 'webotsAlert';
    parent.appendChild(panel);
  }
  panel.innerHTML = message;
  $('#webotsAlert').dialog({
    title: title,
    resizeStart: webotsDisablePointerEvents,
    resizeStop: webotsEnablePointerEvents,
    dragStart: webotsDisablePointerEvents,
    dragStop: webotsEnablePointerEvents,
    appendTo: parent,
    open: webotsOpenDialog,
    modal: true,
    width: 400, // enough room to display the social network buttons in a line
    buttons: {Ok: function() { $(this).dialog('close'); }},
    close: function() {
      if (callback !== undefined)
        callback();
      webots.currentView.ondialogwindow(false);
      $(this).remove();
    }
  });
};

webots.confirm = function(title, message, callback) {
  webots.currentView.ondialogwindow(true);
  var parent = webots.currentView.view3D;
  var panel = document.createElement('div');
  panel.id = 'webotsConfirm';
  panel.innerHTML = message;
  parent.appendChild(panel);
  $('#webotsConfirm').dialog({
    title: title,
    resizeStart: webotsDisablePointerEvents,
    resizeStop: webotsEnablePointerEvents,
    dragStart: webotsDisablePointerEvents,
    dragStop: webotsEnablePointerEvents,
    appendTo: parent,
    open: webotsOpenDialog,
    modal: true,
    width: 400, // enough room to display the social network buttons in a line
    buttons: {Ok: function() { $(this).dialog('close'); callback(); }, Cancel: function() { $(this).dialog('close'); }},
    close: function() { $(this).dialog('destroy').remove(); webots.currentView.ondialogwindow(false); }});
};

webots.parseMillisecondsIntoReadableTime = function(milliseconds) {
  var hours = (milliseconds + 0.9) / (1000 * 60 * 60);
  var absoluteHours = Math.floor(hours);
  var h = absoluteHours > 9 ? absoluteHours : '0' + absoluteHours;
  var minutes = (hours - absoluteHours) * 60;
  var absoluteMinutes = Math.floor(minutes);
  var m = absoluteMinutes > 9 ? absoluteMinutes : '0' + absoluteMinutes;
  var seconds = (minutes - absoluteMinutes) * 60;
  var absoluteSeconds = Math.floor(seconds);
  var s = absoluteSeconds > 9 ? absoluteSeconds : '0' + absoluteSeconds;
  var ms = Math.floor((seconds - absoluteSeconds) * 1000);
  if (ms < 10)
    ms = '00' + ms;
  else if (ms < 100)
    ms = '0' + ms;
  return h + ':' + m + ':' + s + ':' + ms;
};

// get the directory path to the currently executing script file
// for example: https://cyberbotics.com/wwi/8.6/
webots.currentScriptPath = function() {
  var scripts = document.querySelectorAll('script[src]');
  for (var i = 0; i < scripts.length; i++) {
    var src = scripts[i].src;
    var index = src.indexOf('?');
    if (index > 0)
      src = src.substring(0, index); // remove query string
    if (!src.endsWith('webots.js'))
      continue;
    index = src.lastIndexOf('/');
    return src.substring(0, index + 1);
  }
  return '';
};

// add startsWith() and endsWith() functions to the String prototype
if (typeof String.prototype.startsWith !== 'function') {
  String.prototype.startsWith = function(prefix) {
    return this.slice(0, prefix.length) === prefix;
  };
}

if (typeof String.prototype.endsWith !== 'function') {
  String.prototype.endsWith = function(suffix) {
    return this.indexOf(suffix, this.length - suffix.length) !== -1;
  };
}
