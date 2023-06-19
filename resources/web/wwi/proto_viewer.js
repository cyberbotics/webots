/* global showdown */
/* global hljs */

function populateProtoViewDiv(mdContent, imgPrefix, infoArray) {
  const view = document.getElementsByClassName('proto-doc')[0];
  while (view.firstChild)
    view.removeChild(view.firstChild);

  // markdown to html
  window.mermaidGraphCounter = 0;
  window.mermaidGraphs = {};
  const converter = new showdown.Converter({tables: 'True',
    extensions: [
      'wbTabComponent', 'wbSpoiler', 'wbChart', 'wbVariables', 'wbAPI', 'wbFigure', 'wbAnchors',
      'wbIllustratedSection', 'youtube'
    ]});
  const html = converter.makeHtml(mdContent);
  view.innerHTML = html;

  setupModalWindow('#main-container');
  renderGraphs();
  redirectImages(view, imgPrefix);
  updateModalEvents(view);
  redirectUrls(view);
  highlightCode(view);
  view.insertBefore(infoArray, view.firstChild);
}

function renderGraphs() {
  for (let id in window.mermaidGraphs) {
    window.mermaidAPI.render(id, window.mermaidGraphs[id], function(svgCode, bindFunctions) {
      document.querySelector('#' + id + 'Div').innerHTML = svgCode;
      // set min-width to be 2/3 of the max-width otherwise the text might become too small
      const element = document.querySelector('#' + id);
      const style = element.getAttribute('style');
      element.setAttribute('style',
        style + ' min-width:' + Math.floor(0.66 * parseInt(style.split('max-width:')[1].split('px'))) + 'px;');
    });
  }
}

function highlightCode(view) {
  const supportedLanguages = ['c', 'cpp', 'java', 'python', 'matlab', 'sh', 'ini', 'tex', 'makefile', 'lua', 'xml',
    'javascript'];

  for (let i = 0; i < supportedLanguages.length; i++) {
    const language = supportedLanguages[i];
    hljs.configure({languages: [ language ]});
    const codes = document.querySelectorAll('.' + language);
    for (let j = 0; j < codes.length; j++) {
      const code = codes[j];
      hljs.highlightBlock(code);
    }
  }
}

function redirectUrls(node) {
  // redirect a's href
  const as = node.querySelectorAll('a');
  for (const a of as) {
    const href = a.getAttribute('href');
    if (!href)
      continue;
    else if (href.startsWith('http')) // open external links in a new window
      a.setAttribute('target', '_blank');
  }
}

function redirectImages(node, prefix) {
  // redirect img's src
  const imgs = node.querySelectorAll('img');
  for (let i = 0; i < imgs.length; i++) {
    const img = imgs[i];
    const src = img.getAttribute('src');
    const match = /^images\/(.*)$/.exec(src);
    if (match && match.length === 2)
      img.setAttribute('src', prefix + 'images/' + match[1]);
  }
}

function updateModalEvents(view) {
  const modal = document.querySelector('#modal-window');
  const image = modal.querySelector('.modal-window-image-content');
  const loadImage = modal.querySelector('.modal-window-load-image');
  const caption = modal.querySelector('.modal-window-caption');

  // Add the modal events on each image.
  const imgs = view.querySelectorAll('img');
  for (let i = 0; i < imgs.length; i++) {
    imgs[i].onclick = function(event) {
      const img = event.target;
      // The modal window is only enabled on big enough images and on thumbnail.
      if (img.src.indexOf('thumbnail') === -1 && !(img.naturalWidth > 128 && img.naturalHeight > 128))
        return;

      // Show the modal window and the caption.
      modal.style.display = 'block';
      caption.innerHTML = (typeof this.parentNode.childNodes[1] !== 'undefined') ? this.parentNode.childNodes[1].innerHTML : '';

      if (img.src.indexOf('.thumbnail.') === -1) {
        // this is not a thumbnail => show the image directly.
        image.src = img.src;
        loadImage.style.display = 'none';
        image.style.display = 'block';
      } else {
        // this is a thumbnail => load the actual image.
        let url = img.src.replace('.thumbnail.', '.');
        if (image.src === url) {
          // The image has already been loaded.
          loadImage.style.display = 'none';
          image.style.display = 'block';
          return;
        } else {
          // The image has to be loaded: show the loading image.
          loadImage.style.display = 'block';
          image.style.display = 'none';
        }
        // In case of thumbnail, search for the original png or jpg
        image.onload = function() {
          // The original image has been loaded successfully => show it.
          loadImage.style.display = 'none';
          image.style.display = 'block';
        };
        image.onerror = function() {
          // The original image has not been loaded successfully => try to change the extension and reload it.
          image.onerror = function() {
            // The original image has not been loaded successfully => abort.
            modal.style.display = 'none';
            loadImage.style.display = 'block';
            image.style.display = 'none';
          };
          url = img.src.replace('.thumbnail.jpg', '.png');
          image.src = url;
        };
        image.src = url;
      }
    };
  }
}

function setupModalWindow(container) {
  const doc = document.querySelector(container);

  // Create the following HTML tags:
  // <div id="modal-window" class="modal-window">
  //   <span class="modal-window-close-button">&times;</span>
  //   <img class="modal-window-image-content" />
  //   <div class="modal-window-caption"></div>
  // </div>

  const close = document.createElement('span');
  close.classList.add('modal-window-close-button');
  close.innerHTML = '&times;';
  close.onclick = function() {
    modal.style.display = 'none';
  };

  const loadImage = document.createElement('img');
  loadImage.classList.add('modal-window-load-image');
  loadImage.setAttribute('src', 'https://raw.githubusercontent.com/cyberbotics/webots/R2023b/resources/web/wwi/images/loading/load_animation.gif');

  const image = document.createElement('img');
  image.classList.add('modal-window-image-content');

  const caption = document.createElement('div');
  caption.classList.add('modal-window-caption');

  const modal = document.createElement('div');
  modal.setAttribute('id', 'modal-window');
  modal.classList.add('modal-window');

  modal.appendChild(close);
  modal.appendChild(loadImage);
  modal.appendChild(image);
  modal.appendChild(caption);
  doc.appendChild(modal);

  window.onclick = function(event) {
    if (event.target === modal) {
      modal.style.display = 'none';
      loadImage.style.display = 'block';
      image.style.display = 'none';
    }
  };
}

export {setupModalWindow, populateProtoViewDiv, highlightCode, updateModalEvents, renderGraphs};
