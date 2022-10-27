import {webots} from './webots.js';

export function loadMeshData(prefix, url) {
  if (typeof url === 'undefined')
    return;

  let worldsPath;
  if (typeof webots.currentView.stream === 'undefined')
    worldsPath = '';
  else {
    worldsPath = webots.currentView.stream.view.currentWorld;
    worldsPath = worldsPath.substring(0, worldsPath.lastIndexOf('/')) + '/';
  }

  if (url.startsWith('webots://')) {
    if (typeof webots.currentView.repository === 'undefined')
      webots.currentView.repository = 'cyberbotics';
    if (typeof webots.currentView.branch === 'undefined' || webots.currentView.branch === '')
      webots.currentView.branch = 'released';
    url = url.replace('webots://', 'https://raw.githubusercontent.com/' + webots.currentView.repository + '/webots/' + webots.currentView.branch + '/');
  }

  if (typeof prefix !== 'undefined' && !url.startsWith('http'))
    url = prefix + worldsPath + url;

  const urls = [url];

  if (typeof loadMeshData.assimpjs === 'undefined')
    loadMeshData.assimpjs = assimpjs();

  if (url.endsWith('.obj')) {
    return fetch(url)
      .then(response => response.text())
      .then(text => {
        const regex = new RegExp('[A-Za-z0-9_-]+\\.mtl');
        const mtl = text.match(regex);
        let mtlPath;
        if (mtl) {
          if (typeof prefix !== 'undefined' && !mtl[0].startsWith('http'))
            mtl[0] = url.substring(0, url.lastIndexOf('/') + 1) + mtl[0];
          urls.push(mtl[0]);
          mtlPath = mtl[0].substring(0, mtl[0].lastIndexOf('/') + 1);
        }
        return loadInAssimp(urls, mtlPath);
      });
  } else
    return loadInAssimp(urls);
}

function loadInAssimp(urls, mtlPath) {
  return loadMeshData.assimpjs.then(function(ajs) {
    // fetch the files to import
    return Promise.all(urls.map((file) => fetch(file))).then((responses) => {
      return Promise.all(responses.map((res) => res.arrayBuffer()));
    }).then((arrayBuffers) => {
      // create new file list object, and add the files
      let fileList = new ajs.FileList();
      for (let i = 0; i < urls.length; i++)
        fileList.AddFile(urls[i], new Uint8Array(arrayBuffers[i]));

      // convert file list to assimp json
      let result = ajs.ConvertFileList(fileList, 'assjson', true);

      // check if the conversion succeeded
      if (!result.IsSuccess() || result.FileCount() === 0) {
        console.error(result.GetErrorCode());
        return;
      }

      // get the result file, and convert to string
      let resultFile = result.GetFile(0);
      let jsonContent = new TextDecoder().decode(resultFile.GetContent());

      return [JSON.parse(jsonContent), mtlPath];
    });
  });
}
