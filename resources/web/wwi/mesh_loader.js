import {webots} from './webots.js';

export function loadMeshData(prefix, urls) {
  if (typeof urls === 'undefined')
    return;

  let worldsPath;
  if (typeof webots.currentView.stream === 'undefined')
    worldsPath = '';
  else {
    worldsPath = webots.currentView.stream.view.currentWorld;
    worldsPath = worldsPath.substring(0, worldsPath.lastIndexOf('/')) + '/';
  }

  for (let i = 0; i < urls.length; i++) {
    if (urls[i].startsWith('webots://')) {
      if (typeof webots.currentView.repository === 'undefined')
        webots.currentView.repository = 'cyberbotics';
      if (typeof webots.currentView.branch === 'undefined' || webots.currentView.branch === '')
        webots.currentView.branch = 'released';
      urls[i] = urls[i].replace('webots://', 'https://raw.githubusercontent.com/' + webots.currentView.repository + '/webots/' + webots.currentView.branch + '/');
    }
    if (typeof prefix !== 'undefined' && !urls[i].startsWith('http'))
      urls[i] = prefix + worldsPath + urls[i];
  }
  if (typeof loadMeshData.assimpjs === 'undefined')
    loadMeshData.assimpjs = assimpjs();

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

      return JSON.parse(jsonContent);
    });
  });
}
