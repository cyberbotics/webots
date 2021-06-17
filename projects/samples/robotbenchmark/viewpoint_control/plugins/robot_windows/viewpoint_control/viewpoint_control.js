/* global view, webots, THREE */
/* global showBenchmarkRecord, showBenchmarkError, saveCookies, getCookie,  */
/* exported recordPerformance */

// $('#record-button').button();

var benchmarkName = 'Viewpoint Control';
var benchmarkPerformance;
view.ontouchmove = evaluateViewpoint;
view.onmousedrag = evaluateViewpoint;
view.onmousewheel = evaluateViewpoint;

function evaluateViewpoint(event) {
  var camera = view.x3dScene.viewpoint.camera;
  var currentOrientation = webots.quaternionToAxisAngle(camera.quaternion);
  currentOrientation.axis.negate();
  var targetPosition = new THREE.Vector3(0, 0.45, 0.333);
  var targetOrientation = { 'axis': new THREE.Vector3(0, 0.7071, 0.7071), 'angle': Math.PI };
  var maxIndex;
  if (Math.abs(currentOrientation.axis.x) > Math.abs(currentOrientation.axis.y)) {
    if (Math.abs(currentOrientation.axis.x) > Math.abs(currentOrientation.axis.z))
      maxIndex = 'x';
    else
      maxIndex = 'z';
  } else if (Math.abs(currentOrientation.axis.y) > Math.abs(currentOrientation.axis.z))
    maxIndex = 'y';
  else
    maxIndex = 'z';
  if (currentOrientation[maxIndex] < 0) {
    currentOrientation.axis.negate();
    currentOrientation.angle = -currentOrientation.angle;
  }
  var positionDifference = camera.position.distanceTo(targetPosition);
  var orientationDifference = currentOrientation.axis.distanceToSquared(targetOrientation.axis);
  if (currentOrientation.angle < 0)
    currentOrientation.angle += 2 * Math.PI;
  var angleDifference = Math.abs(currentOrientation.angle - targetOrientation.angle);
  benchmarkPerformance = angleDifference + orientationDifference + positionDifference;
  benchmarkPerformance = 100 - benchmarkPerformance * 20;
  if (benchmarkPerformance < 0)
    benchmarkPerformance = 0;
  benchmarkPerformance -= 90;
  benchmarkPerformance *= 10;
  var performanceString = "<font color='";
  if (benchmarkPerformance > 0) {
    performanceString += 'green';
    if (document.getElementById('record-button').getAttribute('disabled')) {
      document.getElementById('record-button').getAttribute('disabled', false).classList.remove('ui-state-disabled');
      document.getElementById('record-button').style.fontWeight = 'bold';
    }
  } else {
    performanceString += 'red';
    if (!document.getElementById('record-button').getAttribute('disabled')) {
      document.getElementById('record-button').getAttribute('disabled', true).classList.add('ui-state-disabled');
      document.getElementById('record-button').style.fontWeight = 'normal';
    }
  }
  performanceString += "'>" + benchmarkPerformance.toFixed(2) + '%</font>';
  document.getElementById('achievement').html(performanceString);
}

function recordPerformance() {
  var p = benchmarkPerformance / 100;
  saveCookies(benchmarkName, parseFloat(p.toFixed(4)));
  var email = getCookie('email');
  var password = getCookie('password');
  if (email === undefined || password === undefined) {
    webots.alert(
      benchmarkName + ' complete.',
      benchmarkPerformance.toFixed(2) + '% complete<br>' +
      "<p>You should log in at <a href='https://robotbenchmark.net'>robotbenchmark.net</a> to record your performance.</p>");
    return false;
  }
  var record = benchmarkPerformance / 100;
  email = decodeURIComponent(email);

  var request = new XMLHttpRequest();
  request.open('POST', '/record.php', true);
  request.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded; charset=UTF-8');
  request.send({
    email: email,
    password: password,
    benchmark: 'viewpoint_control',
    record: record,
    key: '1'
  });

  request.onreadystatechange = data => function(data) {
    if (request.readyState === 4 && request.status === 200) {
      var result = showBenchmarkRecord('record:' + data, benchmarkName, metricToString);
      if (!result['isNewRecord']) {
        // current record is worst than personal record
        var text = "<p style='font-weight:bold'>You did not outperform your personal record.</p>" +
                   '<p>Your personal record is: ' + metricToString(result['personalRecord']) + '.</p>' +
                   '<p>Your current performance is: ' + metricToString(record) + '.</p>';
        webots.alert(benchmarkName + ' result', text);
        return false;
      }
    } else
      showBenchmarkError('record:' + data, benchmarkName);
  };

  return true;
}

function metricToString(metric) {
  return (metric * 100).toFixed(2) + '%';
}
