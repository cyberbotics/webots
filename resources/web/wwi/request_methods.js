// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Retrieve the given GET value defined by its "variableName"
// if not found, assign it "defaultValue" instead

/* exported getGETQueryValue */
/* exported getGETQueriesMatchingRegularExpression */

function getGETQueryValue(variableName, defaultValue) {
  var query = window.location.search.substring(1);
  var vars = query.split('&');
  for (let i = 0; i < vars.length; i++) {
    var pair = vars[i].split('=');
    if (pair[0] === variableName)
      return pair[1];
  }
  return defaultValue;
}

function getGETQueriesMatchingRegularExpression(pattern) {
  var values = {};
  var query = window.location.search.substring(1);
  if (query === '')
    return values;
  var vars = query.split('&');
  var regex = new RegExp(pattern);
  for (let i = 0; i < vars.length; i++) {
    var pair = vars[i].split('=');
    if (regex.test(pair[0]))
      values[pair[0].toLowerCase()] = pair[1].toLowerCase();
  }
  return values;
}
