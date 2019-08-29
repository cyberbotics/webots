var robotWindow = null;
var basicTimeStep = 0.032;
var graphs = {};

function menuTabCallback(category) {
  var i, x, tablinks;
  x = document.getElementsByClassName('content');
  for (i = 0; i < x.length; ++i)
    x[i].style.display = 'none';
  tablinks = document.getElementsByClassName('menu-button');
  for (i = 0; i < x.length; ++i)
    tablinks[i].className = tablinks[i].className.replace(' menu-button-selected', '');
  document.getElementById(category + '-tab').style.display = 'block';
  document.getElementById(category + '-menu-button').className += ' menu-button-selected';

  // force widgets graphs when they are shown.
  Object.keys(graphs).forEach(function(name) {
    graphs[name].forEach(function(widget) {
      if (widget.shown)
        widget.refresh();
    });
  });
}

function enableGraphs(checkbox) {
  robotWindow.send('graphs:' + checkbox.getAttribute('graphtype') + ':' + checkbox.checked);
}

function checkboxCallback(checkbox) {
  if (checkbox.checked)
    robotWindow.send('enable:' + checkbox.getAttribute('marker'));
  else
    robotWindow.send('disable:' + checkbox.getAttribute('marker'));
}

function sliderCallback(slider) {
  var marker = slider.getAttribute('marker');
  document.getElementById('slider_value_' + marker).innerHTML = slider.value;
  robotWindow.send('radius:' + slider.value + ':' + marker);
}

function updateTransparency(slider) {
  document.getElementById('slider_value_transparency').innerHTML = slider.value;
  robotWindow.send('body_transparency:' + slider.value);
}

function updateSpeed(slider) {
  document.getElementById('slider_value_speed').innerHTML = slider.value;
  robotWindow.send('speed:' + slider.value);
}

function colorCallback(color) {
  robotWindow.send('color:' + color.value + ':' + color.getAttribute('marker'));
}

function comboboxCallback(combobox) {
  var markerGraphs = graphs[combobox.getAttribute('marker')];
  markerGraphs.forEach(function(widget) {
    widget.show(false);
  });
  markerGraphs[combobox.selectedIndex].show(true);
}

function hideShowAll(virtual, hide) {
  var div = virtual ? document.getElementById('virtual_markers') : document.getElementById('markers');
  var checkboxes = div.getElementsByClassName('visibilityCheckbox');
  message = hide ? 'disable' : 'enable';
  for (var i = 0; i < checkboxes.length; i++) {
    checkboxes[i].checked = true;
    message += ':' + checkboxes[i].getAttribute('marker');
  }
  robotWindow.send(message);
}

function changeColor(virtual, color) {
  var div = virtual ? document.getElementById('virtual_markers') : document.getElementById('markers');
  var colorSelectors = div.getElementsByClassName('colorSelector');
  message = 'color:' + color;
  for (var i = 0; i < colorSelectors.length; i++) {
    colorSelectors[i].value = color;
    message += ':' + colorSelectors[i].getAttribute('marker');
  }
  robotWindow.send(message);
}

function changeRadius(virtual, radius) {
  var div = virtual ? document.getElementById('virtual_markers') : document.getElementById('markers');
  var radiusSliders = div.getElementsByClassName('radiusSlider');
  message = 'radius:' + radius;
  for (var i = 0; i < radiusSliders.length; i++) {
    radiusSliders[i].value = radius;
    var marker = radiusSliders[i].getAttribute('marker')
    message += ':' + marker;
    document.getElementById('slider_value_' + marker).innerHTML = radius;
  }
  robotWindow.send(message);
}

webots.window('c3d_viewer_window').receive = function(message, robot) {
  robotWindow = this;
  if (message.startsWith('configure:')) {
    var values = message.split(':');
    basicTimeStep = 0.001 * values[1];
  } else if (message.startsWith('labels:')) {
    message = message.slice('labels:'.length).split(':');
    var type = message[0];
    var unit = message[1];
    var names = message[2].split(' ');
    if (type == 'virtual_markers' || type == 'markers') {
      var isVirtual = type == 'virtual_markers';
      var div = document.getElementById(type);
      var content = '';
      if (message[2] == 'None')
        content = 'None';
      else {
        // options
        for (var i = 0; i < names.length; i++) {
          var name = names[i];
          content += '<div class="markerDiv">';
          content += name;
          content += '<input type="checkbox" class="visibilityCheckbox" title="Show/hide this marker." marker="' + name + '" onclick="checkboxCallback(this)"' + (isVirtual ? '' : ' checked') + '/>';
          content += '<input type="range" class="radiusSlider" min="0.001" max="0.1" step = "0.001" value="0.01" data-show-value="true" class="slider" title="Radius of the marker." marker="' + name + '" onchange="sliderCallback(this)"/>';
          content += '<span id="slider_value_' + name + '">0.001</span>';
          content += '<input type="color" class="colorSelector" marker="' + name + '" value="#00ff00" onchange="colorCallback(this)"/>';
          content += '</div>';
        }
      }
      div.innerHTML = content;
      type = 'markers';
    }
    // graph
    if (message[2] == 'None') {
      if (type != 'markers')
        document.getElementById('graphs-' + type).innerHTML = 'None';
    } else {
      for (var i = 0; i < names.length; i++) {
        var name = names[i];
        var tmp = document.createElement('tmp');
        var div = '<div id="' + name + '-graph-container" class="marker-plot">';
        div += '<h3>' + name;
        div += '<select onChange="comboboxCallback(this)" class="view-selector" marker="' + name + '">' +
               '  <option>Time</option>' +
               '  <option>XY</option>' +
               '  <option>YZ</option>' +
               '  <option>XZ</option>' +
               '</select>'
        div += '</h3>';
        div += '<div id="' + name + '-graph" class="marker-plot-content"/></div>';
        div += '</div>';
        tmp.innerHTML = div;
        document.getElementById('graphs-' + type).appendChild(tmp.firstChild);

        var widgetTime = new TimeplotWidget(document.getElementById(name + '-graph'), basicTimeStep, TimeplotWidget.prototype.AutoRangeType.STRETCH, {'min': -1, 'max': 1}, {'x': 'Time [s]', 'y': '[' + unit + ']'}, null);
        var widgetXY = new PlotWidget(document.getElementById(name + '-graph'), TimeplotWidget.prototype.AutoRangeType.STRETCH, {'x': 0, 'y': 1}, {'min': -1, 'max': 1}, {'min': -1, 'max': 1}, {'x': 'x [' + unit + ']', 'y': 'y [' + unit + ']'}, null);
        var widgetYZ = new PlotWidget(document.getElementById(name + '-graph'), TimeplotWidget.prototype.AutoRangeType.STRETCH, {'x': 1, 'y': 2}, {'min': -1, 'max': 1}, {'min': -1, 'max': 1}, {'x': 'y [' + unit + ']', 'y': 'z [' + unit + ']'}, null);
        var widgetXZ = new PlotWidget(document.getElementById(name + '-graph'), TimeplotWidget.prototype.AutoRangeType.STRETCH, {'x': 0, 'y': 2}, {'min': -1, 'max': 1}, {'min': -1, 'max': 1}, {'x': 'x [' + unit + ']', 'y': 'z [' + unit + ']'}, null);

        widgetXY.show(false);
        widgetYZ.show(false);
        widgetXZ.show(false);

        widgetTime.refresh();
        graphs[name] = [widgetTime, widgetXY, widgetYZ, widgetXZ];
      }
    }
  } else if (message.startsWith('positions:')) {
    var positions = message.split(':');
    var n = positions.length / 2;
    time = parseFloat(positions[1]);
    for (var i = 1; i < n; i++) {  // start at 1 because 0 is 'positions:' and time
      var name = positions[i * 2];
      var coordinates = positions[i * 2 + 1].split(',');
      var x = parseFloat(coordinates[0]);
      var y = parseFloat(coordinates[1]);
      var z = parseFloat(coordinates[2]);
      if (name in graphs) {
        graphs[name].forEach(function(widget) {
          widget.addValue({'x': time, 'y': [x, y, z]});
          if (widget.shown)
            widget.refresh();
        });
      }
    }
  } else
    console.log(message);
}

window.onload = function() {
  document.getElementById('select_markers').onclick = function() {
    hideShowAll(false, false);
  };
  document.getElementById('unselect_markers').onclick = function() {
    hideShowAll(false, true);
  };
  document.getElementById('select_virtual_markers').onclick = function() {
    hideShowAll(true, false);
  };
  document.getElementById('unselect_virtual_markers').onclick = function() {
    hideShowAll(true, true);
  };
  document.getElementById('color_virtual_markers').onchange = function() {
    changeColor(true, this.value);
  };
  document.getElementById('color_markers').onchange = function() {
    changeColor(false, this.value);
  };
  document.getElementById('radius_virtual_markers').onchange = function() {
    changeRadius(true, this.value);
  };
  document.getElementById('radius_markers').onchange = function() {
    changeRadius(false, this.value);
  };
  menuTabCallback('config');
};
