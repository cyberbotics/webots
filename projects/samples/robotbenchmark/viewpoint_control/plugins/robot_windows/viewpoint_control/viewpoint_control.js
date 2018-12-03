$('#infotabs').tabs();
$('#record-button').button();

var benchmarkName = "Viewpoint Control";
view.onmousedrag = evaluateViewpoint;
view.onmousewheel = evaluateViewpoint;

function evaluateViewpoint(event) {
  var viewpoint = view.x3dScene.getElementsByTagName('Viewpoint')[0];
  var currentPosition = x3dom.fields.SFVec3f.parse(viewpoint.getAttribute('position'));
  var currentOrientation = x3dom.fields.SFVec4f.parse(viewpoint.getAttribute('orientation'));
  var targetPosition = new x3dom.fields.SFVec3f(0, 0.45, 0.333);
  var targetOrientation = new x3dom.fields.SFVec4f(0, 0.7071, 0.7071, 3.1415927);
  var maxIndex;
  if (Math.abs(currentOrientation.x) > Math.abs(currentOrientation.y)) {
    if (Math.abs(currentOrientation.x) > Math.abs(currentOrientation.z))
      maxIndex = 'x';
    else
      maxIndex = 'z';
  } else if (Math.abs(currentOrientation.y) > Math.abs(currentOrientation.z))
    maxIndex = 'y';
  else
    maxIndex = 'z';
  if (currentOrientation[maxIndex] < 0) {
    currentOrientation.x = -currentOrientation.x;
    currentOrientation.y = -currentOrientation.y;
    currentOrientation.z = -currentOrientation.z;
    currentOrientation.w = -currentOrientation.w;
  }
  var positionDifference = currentPosition.subtract(targetPosition).length();
  var dx = currentOrientation.x - targetOrientation.x;
  var dy = currentOrientation.y - targetOrientation.y;
  var dz = currentOrientation.z - targetOrientation.z;
  var orientationDifference = Math.sqrt(dx * dx + dy * dy + dz * dz);
  if (currentOrientation.w < 0)
    currentOrientation.w += 2 * Math.PI;
  var angleDifference = Math.abs(currentOrientation.w - targetOrientation.w);
  performance = angleDifference + orientationDifference + positionDifference;
  performance = 100 - performance * 20
  if (performance < 0)
    performance = 0;
  performance -= 90;
  performance *= 10;
  var performanceString = "<font color='";
  if (performance > 0) {
    performanceString += 'green';
    if ($('#record-button').attr('disabled')) {
      $('#record-button').attr('disabled', false).removeClass('ui-state-disabled');
      $('#record-button').css('font-weight', 'bold');
    }
  } else {
    performanceString += 'red';
    if (!$('#record-button').attr('disabled')) {
      $('#record-button').attr('disabled', true).addClass('ui-state-disabled');
      $('#record-button').css('font-weight', 'normal');
    }
  }
  performanceString += "'>" + performance.toFixed(2) + "%</font>";
  $('#achievement').html(performanceString);
}

function recordPerformance() {
  var p = performance / 100;
  saveCookies(benchmarkName, parseFloat(p.toFixed(4)));
  var email = getCookie('email');
  var password = getCookie('password');
  if (email === undefined || password === undefined) {
    webots.alert(benchmarkName + " complete.",
                 performance.toFixed(2) + "% complete<br>" +
                 "<p>You should log in at <a href='https://robotbenchmark.net'>robotbenchmark.net</a> to record your performance.</p>");
    return false;
  }
  var record = performance / 100;
  email = decodeURIComponent(email);
  $.post('/record.php', {email: email,
                         password: password,
                         benchmark: 'viewpoint_control',
                         record: record,
                         key: '1'}).done(function(data) {
                           if (data.startsWith('OK:')) {
                             var result = showBenchmarkRecord('record:' + data, benchmarkName, metricToString);
                             if (!result['isNewRecord']) {
                               // current record is worst than personal record
                               text = "<p style='font-weight:bold'>You did not outperform your personal record.</p>" +
                                      "<p>Your personal record is: " + metricToString(result['personalRecord']) + ".</p>" +
                                      "<p>Your current performance is: " + metricToString(record) + ".</p>";
                               webots.alert(benchmarkName + " result", text);
                               return false;
                             }
                           } else
                             showBenchmarkError('record:' + data, benchmarkName);
                         });
  return true;
}

function metricToString(metric) {
  return (metric * 100).toFixed(2) + '%';
}
