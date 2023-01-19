export default class MeshLoader {
  static loadMeshData(prefix, url) {
    if (typeof url === 'undefined')
      return;

    let worldsPath;
    if (!MeshLoader.stream)
      worldsPath = '';
    else {
      worldsPath = MeshLoader.currentWorld;
      worldsPath = worldsPath.substring(0, worldsPath.lastIndexOf('/')) + '/';
    }

    if (url.startsWith('webots://')) {
      if (typeof MeshLoader.repository === 'undefined')
        MeshLoader.repository = 'cyberbotics';
      if (typeof MeshLoader.branch === 'undefined' || MeshLoader.branch === '')
        MeshLoader.branch = 'released';
      url = url.replace('webots://', 'https://raw.githubusercontent.com/' + MeshLoader.repository + '/webots/' + MeshLoader.branch + '/');
    }

    if (typeof prefix !== 'undefined' && !url.startsWith('http'))
      url = prefix + worldsPath + url;

    const urls = [url];

    if (typeof MeshLoader.assimpjs === 'undefined')
      MeshLoader.assimpjs = assimpjs();

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
          return this.#loadInAssimp(urls, mtlPath);
        });
    } else
      return this.#loadInAssimp(urls);
  }

  static #loadInAssimp(urls, mtlPath) {
    return MeshLoader.assimpjs.then(function(ajs) {
      // fetch the files to import
      return Promise.all(urls.map((file) => fetch(file))).then((responses) => {
        return Promise.all(responses.map((res) => res.arrayBuffer()));
      }).then((arrayBuffers) => {
        // create new file list object, and add the files
        let fileList = new ajs.FileList();
        for (let i = 0; i < urls.length; i++)
          fileList.AddFile(urls[i], new Uint8Array(arrayBuffers[i]));

        // convert file list to assimp json
        const result = ajs.ConvertFileList(fileList, 'assjson', true);

        // check if the conversion succeeded
        if (!result.IsSuccess() || result.FileCount() === 0) {
          console.error(result.GetErrorCode());
          return;
        }

        // get the result file, and convert to string
        const resultFile = result.GetFile(0);
        const jsonContent = new TextDecoder().decode(resultFile.GetContent());

        return [JSON.parse(jsonContent), mtlPath];
      });
    });
  }
}
