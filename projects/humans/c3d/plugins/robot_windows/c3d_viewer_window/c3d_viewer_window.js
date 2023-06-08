/* global Canvas, PlotWidget, TimeplotWidget */
/* eslint no-unused-vars: ["error", { "varsIgnorePattern": "Callback", "argsIgnorePattern": "^_"}] */

import RobotWindow from 'https://cyberbotics.com/wwi/R2023b/RobotWindow.js';

var basicTimeStep = 0.032;
var graphs = {};

window.menuTabCallback = function(category) {
  let i, x, tablinks;
  x = document.getElementsByClassName('content');
  for (i = 0; i < x.length; ++i)
    x[i].style.display = 'none';
  tablinks = document.getElementsByClassName('menu-button');
  for (i = 0; i < tablinks.length; ++i)
    tablinks[i].className = tablinks[i].className.replace(' menu-button-selected', '');
  document.getElementById(category + '-tab').style.display = 'block';
  document.getElementById(category + '-menu-button').className += ' menu-button-selected';

  const canvas = new Canvas();
  canvas.clearCanvas();
  updateTabCallback();
}

window.updateTabCallback = function() {
  const canvas = new Canvas();
  canvas.resizeCanvas();
  Object.keys(graphs).forEach(function(name) {
    graphs[name].forEach(function(widget) {
      if (widget.shown) {
        widget.resize();
        widget.refresh(true);
      }
    });
  });
}

window.checkboxCallback = function(checkbox) {
  if (checkbox.checked)
    window.robotWindow.send('enable:' + checkbox.getAttribute('marker'));
  else
    window.robotWindow.send('disable:' + checkbox.getAttribute('marker'));
}


window.sliderCallback = function(slider) {
  const marker = slider.getAttribute('marker');
  document.getElementById('slider_value_' + marker).innerHTML = slider.value;
  window.robotWindow.send('radius:' + slider.value + ':' + marker);
}

window.colorCallback = function(color) {
  window.robotWindow.send('color:' + color.value + ':' + color.getAttribute('marker'));
}

window.comboboxCallback = function(combobox) {
  let markerGraphs = graphs[combobox.getAttribute('marker')];
  markerGraphs.forEach(function(widget) {
    widget.show(false);
  });
  markerGraphs[combobox.selectedIndex].show(true);
}

window.changeColor = function(virtual, color) {
  let div = virtual ? document.getElementById('virtual_markers') : document.getElementById('markers');
  let colorSelectors = div.getElementsByClassName('colorSelector');
  let message = 'color:' + color;
  for (let i = 0; i < colorSelectors.length; i++) {
    colorSelectors[i].value = color;
    message += ':' + colorSelectors[i].getAttribute('marker');
  }
  window.robotWindow.send(message);
}

window.changeRadius = function(virtual, radius) {
  let div = virtual ? document.getElementById('virtual_markers') : document.getElementById('markers');
  let radiusSliders = div.getElementsByClassName('radiusSlider');
  let message = 'radius:' + radius;
  for (let i = 0; i < radiusSliders.length; i++) {
    radiusSliders[i].value = radius;
    const marker = radiusSliders[i].getAttribute('marker');
    message += ':' + marker;
    document.getElementById('slider_value_' + marker).innerHTML = radius;
  }
  window.robotWindow.send(message);
}

function receive(message, robot) {
  if (message.startsWith('configure:')) {
    var values = message.split(':');
    basicTimeStep = 0.001 * values[1];
  } else if (message.startsWith('labels:')) {
    message = message.slice('labels:'.length).split(':');
    const type = message[0];
    const unit = message[1];
    const names = message[2].split(' ');
    if (type === 'virtual_markers' || type === 'markers') {
      const isVirtual = type === 'virtual_markers';
      let div = document.getElementById(type);
      let content = '';
      if (message[2] === 'None')
        content = 'None';
      else {
        // options
        for (var i = 0; i < names.length; i++) {
          const name = names[i];
          content += '<div class="markerDiv">';
          content += name;
          content += '<input type="checkbox" class="visibilityCheckbox" title="Show/hide this marker." marker="' + name +
            '" onclick="checkboxCallback(this)"' + (isVirtual ? '' : ' checked') + '/>';
          content +=
            '<input type="range" class="radiusSlider" min="0.001" max="0.1" step = "0.001" value="0.01" data-show-value="true" class="slider" title="Radius of the marker." marker="' +
            name + '" onchange="sliderCallback(this)"/>';
          content += '<span id="slider_value_' + name + '">0.001</span>';
          content += '<input type="color" class="colorSelector" marker="' + name +
            '" value="#00ff00" onchange="colorCallback(this)"/>';
          content += '</div>';
        }
      }
      div.innerHTML = content;
    }
    // graph
    if (message[2] === 'None') {
      if (type !== 'markers')
        document.getElementById('graphs-' + type).innerHTML = 'None';
    } else {
      for (let i = 0; i < names.length; i++) {
        const name = names[i];
        let tmp = document.createElement('tmp');
        const div = '<div id="' + name + '-graph-container" class="marker-plot">' +
          '<h3>' + name + ': ' +
          '<select onChange="comboboxCallback(this)" class="view-selector" marker="' + name + '">' +
          '  <option style="font-size:11px">Time</option>' +
          '  <option style="font-size:11px">XY</option>' +
          '  <option style="font-size:11px">YZ</option>' +
          '  <option style="font-size:11px">XZ</option>' +
          '</select>' +
          '</h3>' +
          '<div id="' + name + '-graph" class="marker-plot-content"/></div>' +
          '<div class="plot-background"/>' +
          '</div>';
        tmp.innerHTML = div;
        document.getElementById('graphs-' + type).appendChild(tmp.firstChild);

        let widgetTime = new TimeplotWidget(document.getElementById(name + '-graph'), basicTimeStep, TimeplotWidget.AutoRangeType.STRETCH, {
          'min': -1,
          'max': 1
        }, {
          'x': 'Time [s]',
          'y': '[' + unit + ']'
        }, null);
        let widgetXY = new PlotWidget(document.getElementById(name + '-graph'), TimeplotWidget.AutoRangeType.STRETCH, {
          'x': 0,
          'y': 1
        }, {
          'min': -1,
          'max': 1
        }, {
          'min': -1,
          'max': 1
        }, {
          'x': 'x [' + unit + ']',
          'y': 'y [' + unit + ']'
        }, null);
        let widgetYZ = new PlotWidget(document.getElementById(name + '-graph'), TimeplotWidget.AutoRangeType.STRETCH, {
          'x': 1,
          'y': 2
        }, {
          'min': -1,
          'max': 1
        }, {
          'min': -1,
          'max': 1
        }, {
          'x': 'y [' + unit + ']',
          'y': 'z [' + unit + ']'
        }, null);
        let widgetXZ = new PlotWidget(document.getElementById(name + '-graph'), TimeplotWidget.AutoRangeType.STRETCH, {
          'x': 0,
          'y': 2
        }, {
          'min': -1,
          'max': 1
        }, {
          'min': -1,
          'max': 1
        }, {
          'x': 'x [' + unit + ']',
          'y': 'z [' + unit + ']'
        }, null);

        widgetXY.show(false);
        widgetYZ.show(false);
        widgetXZ.show(false);

        widgetTime.refresh();
        graphs[name] = [widgetTime, widgetXY, widgetYZ, widgetXZ];
      }
    }
  } else if (message.startsWith('positions:')) {
    let positions = message.split(':');
    let n = positions.length / 2;
    let time = parseFloat(positions[1]);
    for (let i = 1; i < n; i++) { // start at 1 because 0 is 'positions:' and time
      const name = positions[i * 2];
      const coordinates = positions[i * 2 + 1].split(',');
      const x = parseFloat(coordinates[0]);
      const y = parseFloat(coordinates[1]);
      const z = parseFloat(coordinates[2]);
      if (name in graphs) {
        graphs[name].forEach(function(widget) {
          widget.addValue({
            'x': time,
            'y': [x, y, z]
          });
          if (widget.shown)
            widget.refresh();
        });
      }
    }
  } else if (message === 'reset') {
    Array.from(document.getElementsByClassName('marker-plot')).forEach(function(element, _index, _array) {
      element.parentNode.removeChild(element);
    });
  } else
    console.log('Unknown message received: "' + message + '"');
};

window.onload = function() {
  window.robotWindow = new RobotWindow();
  window.robotWindow.receive = receive;
  window.robotWindow.send("configure");
  PlotWidget.recordDataInBackground = true;
  TimeplotWidget.recordDataInBackground = true;

  document.getElementById('upload_file').addEventListener('change', function(event) {
    let files = event.target.files;
    let f = files[0];
    let reader = new FileReader();
    reader.onload = (function(_theFile) {
      return function(e) {
        const message = 'c3dfile:' + e.target.result.slice(e.target.result.indexOf(';base64,') + 8); // remove the "*;base64," header
        window.robotWindow.send(message);
      };
    })(f);
    reader.readAsDataURL(f); // perform base64 encoding suitable for sending text through the wwi interface
  });

  function enableGraphs(event) {
    let checkbox = event.target;
    window.robotWindow.send('graphs:' + checkbox.getAttribute('graphtype') + ':' + checkbox.checked);
  }
  document.getElementById('graph-markers-checkbox').addEventListener('click', enableGraphs);
  document.getElementById('graph-virtual-markers-checkbox').addEventListener('click', enableGraphs);
  document.getElementById('graph-angles-checkbox').addEventListener('click', enableGraphs);
  document.getElementById('graph-forces-checkbox').addEventListener('click', enableGraphs);
  document.getElementById('graph-moments-checkbox').addEventListener('click', enableGraphs);
  document.getElementById('graph-powers-checkbox').addEventListener('click', enableGraphs);

  function hideShowAll(virtual, hide) {
    let div = virtual ? document.getElementById('virtual_markers') : document.getElementById('markers');
    let checkboxes = div.getElementsByClassName('visibilityCheckbox');
    let message = hide ? 'disable' : 'enable';
    for (let i = 0; i < checkboxes.length; i++) {
      checkboxes[i].checked = !hide;
      message += ':' + checkboxes[i].getAttribute('marker');
    }
    window.robotWindow.send(message);
  }
  document.getElementById('select_markers').addEventListener('click', function() {
    hideShowAll(false, false);
  });
  document.getElementById('unselect_markers').addEventListener('click', function() {
    hideShowAll(false, true);
  });
  document.getElementById('select_virtual_markers').addEventListener('click', function() {
    hideShowAll(true, false);
  });
  document.getElementById('unselect_virtual_markers').addEventListener('click', function() {
    hideShowAll(true, true);
  });
  document.getElementById('color_virtual_markers').addEventListener('change', function() {
    changeColor(true, this.value);
  });
  document.getElementById('color_markers').addEventListener('change', function() {
    changeColor(false, this.value);
  });
  document.getElementById('radius_virtual_markers').addEventListener('change', function() {
    changeRadius(true, this.value);
  });
  document.getElementById('radius_markers').addEventListener('change', function() {
    changeRadius(false, this.value);
  });
  document.getElementById('transparency-slider').addEventListener('change', function(event) {
    let slider = event.target;
    document.getElementById('slider_value_transparency').innerHTML = slider.value;
    window.robotWindow.send('body_transparency:' + slider.value);
  });
  document.getElementById('speed-slider').addEventListener('change', function(event) {
    let slider = event.target;
    document.getElementById('slider_value_speed').innerHTML = slider.value;
    window.robotWindow.send('speed:' + slider.value);
  });
  document.getElementById('config-menu-button').addEventListener('click', function() {
    menuTabCallback('config');
  });
  document.getElementById('markers-menu-button').addEventListener('click', function() {
    menuTabCallback('markers');
  });
  document.getElementById('virtual_markers-menu-button').addEventListener('click', function() {
    menuTabCallback('virtual_markers');
  });
  document.getElementById('angles-menu-button').addEventListener('click', function() {
    menuTabCallback('angles');
  });
  document.getElementById('forces-menu-button').addEventListener('click', function() {
    menuTabCallback('forces');
  });
  document.getElementById('moments-menu-button').addEventListener('click', function() {
    menuTabCallback('moments');
  });
  document.getElementById('powers-menu-button').addEventListener('click', function() {
    menuTabCallback('powers');
  });
  menuTabCallback('config');

  window.addEventListener('resize', updateTabCallback);
  window.addEventListener('scroll', updateTabCallback);
}
