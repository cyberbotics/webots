/* global showdown */
/* global hljs */

let localSetup = {};

function populateProtoViewDiv(mdContent, imgPrefix) {
  const view = document.getElementsByClassName('proto-doc')[0];
  while (view.firstChild)
    view.removeChild(view.firstChild);

  // console.log('Raw MD content:\n\n');
  // console.log(mdContent);

  // markdown to html
  window.mermaidGraphCounter = 0;
  window.mermaidGraphs = {};
  const converter = new showdown.Converter({tables: 'True',
    extensions: [
      'wbTabComponent', 'wbSpoiler', 'wbChart', 'wbVariables', 'wbAPI', 'wbFigure', 'wbAnchors',
      'wbIllustratedSection', 'youtube'
    ]});
  const html = converter.makeHtml(mdContent);

  // console.log('HTML content: \n\n')
  // console.log(html);

  view.innerHTML = html;

  setupModalWindow();
  renderGraphs();
  redirectImages(view, imgPrefix);
  updateModalEvents(view);
  redirectUrls(view);
  // collapseMovies(view);
  //
  applyAnchorIcons(view);
  highlightCode(view);
  //
  // updateSelection();
  createIndex(view);
  //
  // setupBlogFunctionalitiesIfNeeded();
  // addNavigationToBlogIfNeeded();
  //
  const images = view.querySelectorAll('img');
  if (images.length > 0) {
    // apply the anchor only when the images are loaded,
    // otherwise, the anchor can be overestimated.
    const lastImage = images[images.length - 1];
    lastImage.onload = () => applyAnchor();
  } else
    applyAnchor();
  // applyTabs();
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

function applyAnchorIcons(view) {
  let elements = [];
  const tags = ['figcaption', 'h1', 'h2', 'h3', 'h4', 'h5'];
  for (let i = 0; i < tags.length; i++) {
    const array = Array.prototype.slice.call(view.querySelectorAll(tags[i]));
    elements = elements.concat(array);
  }
  for (let i = 0; i < elements.length; i++) {
    const el = elements[i];
    let name = null;
    if (el.parentNode && el.tagName.toLowerCase() === 'figcaption' && el.parentNode.tagName.toLowerCase() === 'figure')
      name = el.parentNode.getAttribute('name');
    else
      name = el.getAttribute('name');
    if (name) {
      el.classList.add('anchor-header');
      const span = document.createElement('span');
      span.classList.add('anchor-link-image');
      const a = document.createElement('a');
      a.setAttribute('href', '#' + name);
      a.classList.add('anchor-link');
      a.appendChild(span);
      el.insertBefore(a, el.firstChild);
    }
  }
}

function redirectUrls(node) {
  // redirect a's href
  const as = node.querySelectorAll('a');
  for (let i = 0; i < as.length; i++) {
    const a = as[i];
    const href = a.getAttribute('href');
    if (!href)
      continue;
    else if (href.startsWith('#'))
      addDynamicAnchorEvent(a); // on firefox, the second click on the anchor is not dealt cleanly
    else if (href.startsWith('http')) // open external links in a new window
      a.setAttribute('target', '_blank');
    else if (href.endsWith('.md') || href.indexOf('.md#') > -1) {
      let match, newPage, anchor;
      if (href.startsWith('../')) { // Cross-book hyperlink case.
        match = /^..\/([\w-]+)\/([\w-]+).md(#[\w-]+)?$/.exec(href);
        if (match && match.length >= 3) {
          const book = match[1];
          newPage = match[2];
          anchor = match[3];
          if (anchor)
            anchor = anchor.substring(1); // remove the '#' character
          a.setAttribute('href', forgeUrl(book, newPage, localSetup.tabs, anchor));
        }
      } else { // Cross-page hyperlink case.
        addDynamicLoadEvent(a);
        match = /^([\w-]+).md(#[\w-]+)?$/.exec(href);
        if (match && match.length >= 2) {
          newPage = match[1];
          anchor = match[2];
          if (anchor)
            anchor = anchor.substring(1); // remove the '#' character
          a.setAttribute('href', forgeUrl(localSetup.book, newPage, localSetup.tabs, anchor));
        }
      }
    }
  }
}

function addDynamicLoadEvent(el) {
  if (el.classList.contains('dynamicLoad'))
    return;
  el.addEventListener('click',
    function(event) {
      if (event.ctrlKey)
        return;
      aClick(event.target);
      event.preventDefault();
    },
    false
  );
  el.classList.add('dynamicLoad');
}

function aClick(el) {
  setupUrl(el.getAttribute('href'));
  getMDFile();
  updateBrowserUrl();
}

function getMDFile() {
  const target = computeTargetPath() + localSetup.page + '.md';
  console.log('Get MD file: ' + target);
  fetch(target)
    .then(response => response.text())
    .then(content => populateProtoViewDiv(content))
    .catch(error => {
      console.error('Error: ' + error);
      const mainPage = 'index';
      // get the main page instead
      if (localSetup.page !== mainPage) {
        localSetup.page = mainPage;
        getMDFile();
      }
    });
}

function computeTargetPath() {
  let branch = 'released';
  let targetPath = '';
  if (localSetup.branch)
    branch = localSetup.branch;
  if (localSetup.url.startsWith('http'))
    targetPath = localSetup.url + branch + '/docs/';

  targetPath += localSetup.book + '/';

  return targetPath;
}

function updateBrowserUrl() {
  const url = forgeUrl(localSetup.book, localSetup.page, localSetup.tabs, localSetup.anchor);
  if (history.pushState) {
    try {
      history.pushState({state: 'new'}, null, url);
    } catch (err) {
    }
  }
  const canonicalUrl = 'https://cyberbotics.com/doc/' + localSetup.book + '/' + localSetup.page;
  const canonical = document.querySelector('link[rel="canonical"]');
  if (canonical !== null)
    canonical.href = canonicalUrl;
}

function setupUrl(url) {
  setupDefaultUrl(url);

  let tabsQuery = '';
  for (let option in localSetup.tabs) {
    if (!localSetup.tabs[option])
      continue;
    if (tabsQuery)
      tabsQuery += ',';
    tabsQuery += option + '=' + localSetup.tabs[option];
  }
  tabsQuery = '[' + tabsQuery + ']';
  console.log('book=' + localSetup.book + ' page=' + localSetup.page + ' branch=' + localSetup.branch +
    ' tabs=' + tabsQuery + ' anchor=' + localSetup.anchor);
}

function setupDefaultUrl(url) {
  let m;

  m = url.match(/page=([^&#]*)/);
  if (m)
    localSetup.page = m[1].replace(/.md$/, '');
  else
    localSetup.page = 'index';

  m = url.match(/book=([^&#]*)/);
  if (m)
    localSetup.book = m[1];
  else if (!localSetup.book)
    localSetup.book = 'guide';

  // Extract tab options
  if (!localSetup.tabs)
    localSetup.tabs = {};
  const tabRegex = /[?&](tab-[^=]+)=([^&#]+)/g;
  while ((m = tabRegex.exec(url)) !== null)
    localSetup.tabs[m[1]] = m[2];

  m = url.match(/#([^&#]*)/);
  if (m)
    localSetup.anchor = m[1];
  else
    localSetup.anchor = '';
}

function addDynamicAnchorEvent(el) {
  if (el.classList.contains('dynamicAnchor'))
    return;
  el.addEventListener('click',
    function(event) {
      if (event.ctrlKey)
        return;
      let node = event.target;
      while (node && !node.hasAttribute('href'))
        node = node.getParent();
      if (node) {
        localSetup.anchor = extractAnchor(node.getAttribute('href'));
        applyAnchor();
        event.preventDefault();
      }
    },
    false
  );
  el.classList.add('dynamicAnchor');
}

function applyAnchor() {
  const firstAnchor = document.querySelector("[name='" + localSetup.anchor + "']");
  console.log(firstAnchor)
  if (firstAnchor) {
    firstAnchor.scrollIntoView(true);
    if (document.querySelector('.contribution-banner'))
      window.scrollBy(0, -38); // GitHub banner.
  } else
    window.scrollTo(0, 0);
}

function extractAnchor(url) {
  const match = /#([\w-]+)/.exec(url);
  if (match && match.length === 2)
    return match[1];
  return '';
}

function forgeUrl(book, page, tabs, anchor) {
  let tabOption;
  let isFirstArgument;
  const tabsWithUrl = ['tab-language', 'tab-os'];
  const anchorString = (anchor && anchor.length > 0) ? ('#' + anchor) : '';
  let url = location.href;
  isFirstArgument = (url.indexOf('?') < 0);

  // Remove anchor from url
  url = url.split('#')[0];

  // Add or replace the book argument.
  if (url.indexOf('book=') > -1)
    url = url.replace(/book=([^&]+)?/, 'book=' + book);
  else
    url += (isFirstArgument ? '?' : '&') + 'book=' + book;

  // Add or replace the page argument.
  if (url.indexOf('page=') > -1)
    url = url.replace(/page=([\w-]+)?/, 'page=' + page);
  else
    url += '&page=' + page;

  // Add or replace the tab argument.
  for (tabOption in tabs) {
    if (tabsWithUrl.includes(tabOption)) {
      let tabName = tabs[tabOption] ? tabs[tabOption] : '';
      if (url.indexOf(tabOption + '=') > -1)
        url = url.replace(new RegExp(tabOption + '=([^&]+)(#[\\w-]+)?'), tabOption + '=' + tabName);
      else if (tabName)
        url += '&' + tabOption + '=' + tabName;
    }
  }

  url += anchorString;
  return url;
}

function createIndex(view) {
  // Note: the previous index is cleaned up when the parent title is destroyed.

  // Get all the view headings.
  const headings = [].slice.call(view.querySelectorAll('h1, h2, h3, h4'));

  // Do not create too small indexes.
  const content = document.getElementsByClassName('proto-doc')[0];
  if ((content.offsetHeight < 2 * window.innerHeight || headings.length < 4) && headings.length < 2)
    return;

  let level = 3; // current heading level.

  // Create an empty index, and insert it before the second heading.
  const indexTitle = document.createElement('h' + level);
  indexTitle.textContent = 'Index';
  indexTitle.setAttribute('id', 'indexTitle');
  headings[0].parentNode.insertBefore(indexTitle, headings[0]);
  let ul = document.createElement('ul');
  ul.setAttribute('id', 'index');
  headings[0].parentNode.insertBefore(ul, headings[0]);

  headings.forEach(function(heading, i) {
    if (i === 0) // Skip the first heading.
      return;

    // Update current level and ul.
    const newLevel = parseInt(heading.tagName[1]);
    while (newLevel > level) {
      const newUl = document.createElement('ul');
      ul.appendChild(newUl);
      ul = newUl;
      level += 1;
    }
    while (newLevel < level) {
      ul = ul.parentNode;
      level -= 1;
    }

    // Add the <li> tag.
    const anchor = heading.getAttribute('name');
    const a = document.createElement('a');
    a.setAttribute('href', '#' + anchor);
    a.textContent = heading.textContent;
    const li = document.createElement('li');
    li.appendChild(a);
    ul.appendChild(li);
  });
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

function setupModalWindow() {
  const doc = document.querySelector('#main-container');

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
  loadImage.setAttribute('src', 'https://raw.githubusercontent.com/cyberbotics/webots/R2023a/resources/web/wwi/images/loading/load_animation.gif');

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

export {populateProtoViewDiv};
