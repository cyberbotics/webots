function loadScript(scriptUrl) {
  return new Promise(function(resolve, reject) {
    let script = document.createElement('script');
    script.onload = resolve;
    script.src = scriptUrl;
    document.head.appendChild(script);
  });
}

async function init() {
  let script = document.createElement('script');
  script.textContent = `var Module = [];
    Module['locateFile'] = function(path, prefix) {

    // if it's a data file, use a custom dir
    if (path.endsWith(".data"))
      return "https://cyberbotics.com/wwi/R2022a/" + path;

    // otherwise, use the default, the prefix (JS file's dir) + the path
    return prefix + path;
    }`;

  document.head.appendChild(script);

  let promises = [];
  promises.push(loadScript('https://cdn.jsdelivr.net/npm/glm-js@0.0.6-c/build/glm-js.min.js'));
  promises.push(loadScript('https://cyberbotics.com/wwi/R2022a/enum.js'));
  promises.push(loadScript('https://cyberbotics.com/wwi/R2022a/wrenjs.js'));

  await Promise.all(promises);
  script = document.createElement('script');
  script.src = 'https://cyberbotics.com/wwi/R2022a/viewer.js';
  script.type = 'module';
  document.head.appendChild(script);
}

init();
