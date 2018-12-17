/* eslint no-extend-native: ["error", { "exceptions": ["String"] }] */
/* global getGETQueryValue */
/* global setup */
/* global showdown */
/* global hljs */
/* global webots */
/* exported resetRobotComponent */
/* exported toggleDeviceComponent */
/* exported highlightX3DElement */
/* exported openTabFromEvent */

"use strict";

var handle;

if (typeof String.prototype.startsWith !== 'function') {
  String.prototype.startsWith = function(prefix) {
    return this.slice(0, prefix.length) === prefix;
  };
}

if (typeof String.prototype.endsWith !== 'function') {
  String.prototype.endsWith = function(suffix) {
    return this.indexOf(suffix, this.length - suffix.length) !== -1;
  };
}

function isInternetExplorer() {
  var userAgent = navigator.userAgent;
  return userAgent.indexOf('MSIE') !== -1 || userAgent.indexOf('Trident') !== -1;
};

var localSetup = (typeof setup === 'undefined') ? {} : setup;

var isCyberboticsUrl = location.href.indexOf('cyberbotics.com/doc') !== -1;

function setupCyberboticsUrl(url) {
  localSetup.book = 'guide';
  localSetup.page = 'index';
  localSetup.anchor = '';
  localSetup.tab = '';

  var m = url.match(new RegExp('/([^/]+)/([^/\\?#]+)([^/]*)$'));
  if (m) {
    localSetup.book = m[1];
    localSetup.page = m[2];
    var args = m[3];

    m = url.match(/version=([^&#]*)/);
    if (m) {
      var version = m[1];
      var n = version.indexOf(':');
      if (n === -1)
        localSetup.branch = version;
      else
        localSetup.branch = version.substr(n + 1);
    }

    m = url.match(/tab=([^&#]*)/);
    if (m)
      localSetup.tab = m[1];
    else
      localSetup.tab = '';

    m = args.match(/#([^&#]*)/);
    if (m)
      localSetup.anchor = m[1];
    else
      localSetup.anchor = '';
  }
}

function setupDefaultUrl(url) {
  var m;

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

  m = url.match(/tab=([^&#]*)/);
  if (m)
    localSetup.tab = m[1].toLowerCase();
  else if (!localSetup.tab)
    localSetup.tab = '';

  m = url.match(/#([^&#]*)/);
  if (m)
    localSetup.anchor = m[1];
  else
    localSetup.anchor = '';
}

function setupUrl(url) {
  if (isCyberboticsUrl)
    setupCyberboticsUrl(url);
  else
    setupDefaultUrl(url);
  console.log('book=' + localSetup.book + ' page=' + localSetup.page + ' branch=' + localSetup.branch + ' tab=' + localSetup.tab + ' anchor=' + localSetup.anchor);
}

function computeTargetPath() {
  var branch = 'master';
  var targetPath = '';
  if (localSetup.branch)
    branch = localSetup.branch;
  if (localSetup.url.startsWith('http'))
    targetPath = localSetup.url + branch + '/docs/';
  targetPath += localSetup.book + '/';
  return targetPath;
}

function redirectUrls(node) {
  // redirect a's href
  var as = node.querySelectorAll('a');
  for (var i = 0; i < as.length; i++) {
    var a = as[i];
    var href = a.getAttribute('href');
    if (!href)
      continue;
    else if (href.startsWith('#'))
      addDynamicAnchorEvent(a); // on firefox, the second click on the anchor is not dealt cleanly
    else if (href.startsWith('http')) // open external links in a new window
      a.setAttribute('target', '_blank');
    else if (href.endsWith('.md') || href.indexOf('.md#') > -1) {
      var match, newPage, anchor;
      if (href.startsWith('../')) { // Cross-book hyperlink case.
        match = /^..\/([\w-]+)\/([\w-]+).md(#[\w-]+)?$/.exec(href);
        if (match && match.length >= 3) {
          var book = match[1];
          newPage = match[2];
          anchor = match[3];
          if (anchor)
            anchor = anchor.substring(1); // remove the '#' character
          a.setAttribute('href', forgeUrl(book, newPage, localSetup.tab, anchor));
        }
      } else { // Cross-page hyperlink case.
        addDynamicLoadEvent(a);
        match = /^([\w-]+).md(#[\w-]+)?$/.exec(href);
        if (match && match.length >= 2) {
          newPage = match[1];
          anchor = match[2];
          if (anchor)
            anchor = anchor.substring(1); // remove the '#' character
          a.setAttribute('href', forgeUrl(localSetup.book, newPage, localSetup.tab, anchor));
        }
      }
    }
  }
}

function collapseMovies(node) {
  if (location.href.startsWith('file:')) { // if it's the offline documentation embedded in Webots (possibly without network):
    var iframes = node.querySelectorAll('iframe');
    for (var i = 0; i < iframes.length; i++) { // foreach iframe:
      var iframe = iframes[i];
      var src = iframe.getAttribute('src');
      if (src && src.indexOf('youtube')) { // if the iframe is a youtube frame:
        // then, replace the iframe by a text and an hyperlink to the youtube page.
        src = src.replace(/embed\/(.*)\?rel=0/, 'watch?v=$1'); // e.g. https://www.youtube.com/embed/vFwNwT8dZTU?rel=0 to https://www.youtube.com/watch?v=vFwNwT8dZTU
        var p = document.createElement('p');
        p.innerHTML = '<a href="' + src + '">Click here to see the youtube movie.</a>';
        iframe.parentNode.replaceChild(p, iframe);
      }
    }
  }
}

function forgeUrl(book, page, tab, anchor) {
  var anchorString = (anchor && anchor.length > 0) ? ('#' + anchor) : '';
  var url = location.href;
  if (isCyberboticsUrl) {
    var i = location.href.indexOf('cyberbotics.com/doc');
    url = location.href.substr(0, i) + 'cyberbotics.com/doc/' + book + '/' + page;
    if (localSetup.branch !== '' && localSetup.repository && localSetup.repository !== 'omichel')
      url += '?version=' + localSetup.repository + ':' + localSetup.branch;
    else if (localSetup.branch !== '')
      url += '?version=' + localSetup.branch;
    if (localSetup.tab != '' && localSetup.branch == '')
      url += '?tab=' + localSetup.tab;
    else if (localSetup.tab != '')
       url += '&tab=' + localSetup.tab;
    url += anchorString;
  } else {
    var isFirstArgument;

    // Add or replace the book argument.
    if (url.indexOf('book=') > -1)
      url = url.replace(/book=([^&]+)?/, 'book=' + book);
    else {
      isFirstArgument = (url.indexOf('?') < 0);
      url = url + (isFirstArgument ? '?' : '&') + 'book=' + book;
    }

    // Add or replace the page argument.
    if (url.indexOf('page=') > -1)
      url = url.replace(/page=([\w-]+)?/, 'page=' + page);
    else {
      isFirstArgument = (url.indexOf('?') < 0);
      url = url + (isFirstArgument ? '?' : '&') + 'page=' + page;
    }

    // Add or replace the tab argument.
    if (url.indexOf('tab=') > -1)
      url = url.replace(/tab=([^&]+)(#[\w-]+)?/, 'tab=' + tab + anchorString);
    else {
      isFirstArgument = (url.indexOf('?') < 0);
      url = url + (tab != '' ? (isFirstArgument ? '?' : '&') + 'tab=' + tab : '') + anchorString;
    }
  }
  return url;
}

function addDynamicAnchorEvent(el) {
  if (el.classList.contains('dynamicAnchor'))
    return;
  el.addEventListener('click',
    function(event) {
      var node = event.target;
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

function addDynamicLoadEvent(el) {
  if (el.classList.contains('dynamicLoad'))
    return;
  el.addEventListener('click',
    function(event) {
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
  updateContributionBannerUrl();
}

function redirectImages(node) {
  // redirect img's src
  var imgs = node.querySelectorAll('img');
  var targetPath = computeTargetPath();
  for (var i = 0; i < imgs.length; i++) {
    var img = imgs[i];
    var src = img.getAttribute('src');
    var match = /^images\/(.*)$/.exec(src);
    if (match && match.length === 2)
      img.setAttribute('src', targetPath + 'images/' + match[1]);
  }
}

function redirectTextures(node, robotName) {
  // redirect ImageTexture's url
  var textures = node.querySelectorAll('ImageTexture');
  var targetPath = computeTargetPath();
  for (var i = 0; i < textures.length; i++) {
    var texture = textures[i];
    var url = texture.getAttribute('url').slice(1, -1);
    texture.setAttribute('url', targetPath + 'scenes/' + robotName + '/' + url);
  }
}

function applyAnchor() {
  var firstAnchor = document.querySelector("[name='" + localSetup.anchor + "']");
  if (firstAnchor) {
    firstAnchor.scrollIntoView(true);
    if (document.querySelector('.contribution-banner'))
      window.scrollBy(0, -38); // GitHub banner.
    if (isCyberboticsUrl)
      window.scrollBy(0, -44); // Cyberbotics header.
  } else
    window.scrollTo(0, 0);
}

function applyToTitleDiv() {
  var titleContentElement = document.querySelector('#title-content');
  if (titleContentElement) {
    var newTitle;
    if (localSetup.book === 'guide')
      newTitle = 'Webots User Guide';
    else if (localSetup.book === 'reference')
      newTitle = 'Webots Reference Manual';
    else if (localSetup.book === 'blog')
      newTitle = 'Webots Blog';
    else if (localSetup.book === 'automobile')
      newTitle = 'Webots for automobiles';
    else
      newTitle = '';
    if (newTitle.length > 0) {
      newTitle += " <div class='release-tag'>" + getWebotsVersion() + '</div>';
      titleContentElement.innerHTML = newTitle;
    }
  }
}

function addContributionBanner() {
  // if we're on the website we need to move the banner down by the height of the navbar
  var displacement = isCyberboticsUrl ? '44px' : '0px';

  // append contribution sticker to primary doc element
  document.querySelector('#center').innerHTML += '<div style="top:' + displacement + '" class="contribution-banner">' +
                                                 'Found an error?' +
                                                 '<a target="_blank" class="contribution-banner-url" href="https://github.com/omichel/webots/tree/master/docs"> ' +
                                                 'Contribute on GitHub!' +
                                                 '<span class=github-logo />' +
                                                 '</a>' +
                                                 '<p id="contribution-close">X</p>' +
                                                 '</div>';
  updateContributionBannerUrl();

  var contributionBanner = document.querySelector('.contribution-banner');

  document.querySelector('#contribution-close').onclick = function() {
    contributionBanner.parentNode.removeChild(contributionBanner);
  };
}

function updateContributionBannerUrl() {
  var contributionBanner = document.querySelector('.contribution-banner-url');
  if (contributionBanner)
    contributionBanner.href = 'https://github.com/omichel/webots/edit/master/docs/' + localSetup.book + '/' + localSetup.page + '.md';
}

function addNavigationToBlogIfNeeded() {
  if (!document.querySelector("#next-previous-section") && localSetup.book === "blog") {
    let menu = document.querySelector("#menu");
    let lis = menu.querySelectorAll("li");
    let currentPageIndex = -1
    for (i = 0; i < lis.length; ++i) {
      if (lis[i].className === "selected") {
        currentPageIndex = i;
        break;
      }
    }

    if (currentPageIndex == -1)
      return;

    // console.log(currentPageIndex, lis.length);
    // console.log(lis);
    let div = document.createElement("div");
    div.setAttribute("id", "next-previous-section");
    // previous post
    if (currentPageIndex > 0) {
      let previous = lis[currentPageIndex - 1]
      let a = previous.firstChild.cloneNode();
      a.innerHTML += "<< Previous Post: " + previous.textContent;
      a.setAttribute("class", "post-selector left");
      div.appendChild(a);
    }

    if (currentPageIndex < lis.length - 1) {
      let next = lis[currentPageIndex + 1];
      let a = next.firstChild.cloneNode();
      a.innerHTML += "Next Post: " + next.textContent + " >>";
      a.setAttribute("class", "post-selector right");
      div.appendChild(a);
    }

    document.querySelector("#publish-data").parentNode.insertBefore(div, document.querySelector("#publish-data").nextSibling);
  }
}

function setupBlogFunctionalitiesIfNeeded() {
  if (localSetup.book === 'blog') {
    // hide index, this doesn't make sense for a blog post
    let index = document.querySelector("#index");
    let indexTitle = document.querySelector("#indexTitle");

    if (index !== null)
      index.style.display = "none";

    if (indexTitle !== null)
      indexTitle.style.display = "none";

    // hide the release tag, this is also nonsensical here
    document.querySelector(".release-tag").style.display = "none";

    document.title = document.title.replace("documentation", "Blog");

    var figures = document.querySelectorAll('figure');
    if (figures.length > 0) {
      var modal = document.createElement('div');
      var caption = document.createElement('div');
      var close = document.createElement('span');
      var modalContent = document.createElement('img');

      modal.setAttribute('class', 'modal');
      modalContent.setAttribute('class', 'modal-content');
      caption.setAttribute('id', 'caption');

      close.setAttribute('class', 'close');
      close.innerHTML = '&times;';
      close.onclick = function() {
        modal.style.display = 'none';
      };

      modal.appendChild(close);
      modal.appendChild(modalContent);
      modal.appendChild(caption);

      figures[0].parentNode.appendChild(modal);

      window.onclick = function(event) {
        if (event.target === modal)
          modal.style.display = 'none';
      };

      var images = [];
      for (var i = figures.length - 1; i >= 0; i--) {
        figures[i].onclick = null;
        images[i] = figures[i].firstChild;
        images[i].onclick = function() {
          modal.style.display = 'block';
          modalContent.src = this.src;
          caption.innerHTML = this.parentNode.childNodes[1].innerHTML;
        };
      }
    }
  }
}

function createIndex(view) {
  // Note: the previous index is cleaned up when the parent title is destroyed.

  // Get all the view headings.
  var headings = [].slice.call(view.querySelectorAll('h1, h2, h3, h4'));

  // Do not create too small indexes.
  var content = document.querySelector('#content');
  if (content.offsetHeight < 2 * window.innerHeight || headings.length < 4)
    return;

  var level = parseInt(headings[0].tagName[1]) + 1; // current heading level.

  // Create an empty index, and insert it before the second heading.
  var indexTitle = document.createElement('h' + level);
  indexTitle.textContent = 'Index';
  indexTitle.setAttribute("id", "indexTitle");
  headings[0].parentNode.insertBefore(indexTitle, headings[1]);
  var ul = document.createElement('ul');
  ul.setAttribute('id', 'index');
  headings[0].parentNode.insertBefore(ul, headings[1]);

  headings.forEach(function(heading, i) {
    if (i === 0) // Skip the first heading.
      return;

    // Update current level and ul.
    var newLevel = parseInt(heading.tagName[1]);
    while (newLevel > level) {
      var newUl = document.createElement('ul');
      ul.appendChild(newUl);
      ul = newUl;
      level += 1;
    }
    while (newLevel < level) {
      ul = ul.parentNode;
      level -= 1;
    }

    // Add the <li> tag.
    var anchor = heading.getAttribute('name');
    var a = document.createElement('a');
    a.setAttribute('href', '#' + anchor);
    a.textContent = heading.textContent;
    var li = document.createElement('li');
    li.appendChild(a);
    ul.appendChild(li);
  });
}

function getWebotsVersion() {
  if (localSetup.branch)
    return localSetup.branch;
  // Get the Webots version from the showdown wbVariables extension
  var version = '{{ webots.version.full }}';
  var converter = new showdown.Converter({extensions: ['wbVariables']});
  var html = converter.makeHtml(version);
  var tmp = document.createElement('div');
  tmp.innerHTML = html;
  return tmp.textContent || tmp.innerText || '';
}

function applyToPageTitle(mdContent) {
  var hashtagIndex = mdContent.indexOf('#');
  if (hashtagIndex >= 0) {
    while (hashtagIndex + 1 < mdContent.length && mdContent[hashtagIndex + 1] === '#')
      hashtagIndex += 1;
    var hashtagCarriageReturn = mdContent.indexOf('\n', hashtagIndex);
    if (hashtagCarriageReturn >= 0) {
      var title = mdContent.substring(hashtagIndex + 1, hashtagCarriageReturn).trim();
      document.title = 'Webots documentation: ' + title;
    }
  }
}

function populateViewDiv(mdContent) {
  setupUrl(document.location.href);

  var view = document.querySelector('#view');
  while (view.firstChild)
    view.removeChild(view.firstChild);

  // console.log('Raw MD content:\n\n');
  // console.log(mdContent);

  applyToPageTitle(mdContent);

  // markdown to html
  window.mermaidGraphCounter = 0;
  window.mermaidGraphs = {};
  var converter = new showdown.Converter({tables: 'True', extensions: ['wbTabComponent', 'wbRobotComponent', 'wbChart', 'wbVariables', 'wbAPI', 'wbFigure', 'wbAnchors', 'wbIllustratedSection', 'youtube']});
  var html = converter.makeHtml(mdContent);

  // console.log('HTML content: \n\n')
  // console.log(html);

  view.innerHTML = html;

  createRobotComponent(view);
  renderGraphs();
  redirectImages(view);
  redirectUrls(view);
  collapseMovies(view);

  applyAnchorIcons(view);
  highlightCode(view);

  updateSelection();
  createIndex(view);

  setupBlogFunctionalitiesIfNeeded();
  addNavigationToBlogIfNeeded();

  var images = view.querySelectorAll('img');
  if (images.length > 0) {
    // apply the anchor only when the images are loaded,
    // otherwise, the anchor can be overestimated.
    var lastImage = images[images.length - 1];
    $(lastImage).load(applyAnchor);
  } else
    applyAnchor();
  applyTabs();
}

// replace the browser URL after a dynamic load
function updateBrowserUrl() {
  var url = forgeUrl(localSetup.book, localSetup.page, localSetup.tab, localSetup.anchor);
  if (history.pushState) {
    try {
      history.pushState({state: 'new'}, null, url);
    } catch (err) {
    }
  }
  var canonicalUrl = 'https://cyberbotics.com/doc/' + localSetup.book + '/' + localSetup.page;
  $('link[rel="canonical"]').attr('href', canonicalUrl);
}

// Make in order that the back button is working correctly
window.onpopstate = function(event) {
  setupUrl(document.location.href);
  getMDFile();
};

function highlightCode(view) {
  var supportedLanguages = ['c', 'cpp', 'java', 'python', 'matlab', 'sh', 'ini', 'tex', 'makefile', 'lua', 'xml'];

  for (var i = 0; i < supportedLanguages.length; i++) {
    var language = supportedLanguages[i];
    hljs.configure({languages: [ language ]});
    var codes = document.querySelectorAll('.' + language);
    for (var j = 0; j < codes.length; j++) {
      var code = codes[j];
      hljs.highlightBlock(code);
    }
  }
}

function resetRobotComponent(robot) {
  unhighlightX3DElement(robot);
  // Reset the Viewpoint and the motor sliders.
  var robotComponent = document.querySelector('#' + robot + '-robot-component');
  var viewpoint = robotComponent.querySelector('Viewpoint');
  viewpoint.setAttribute('orientation', viewpoint.getAttribute('initialOrientation'));
  viewpoint.setAttribute('position', viewpoint.getAttribute('initialPosition'));
  var sliders = robotComponent.querySelectorAll('.motor-slider');
  for (var s = 0; s < sliders.length; s++) {
    var slider = sliders[s];
    slider.value = slider.getAttribute('webots-position');
    sliderMotorCallback(robot, slider);
  }
}

function toggleDeviceComponent(robot) {
  var deviceMenu = document.querySelector('#' + robot + '-device-component');
  var robotView = document.querySelector('.robot-view');
  if (deviceMenu.style.display === 'none') {
    deviceMenu.style.display = '';
    robotView.style.width = '70%';
  } else {
    deviceMenu.style.display = 'none';
    robotView.style.width = '100%';
  }
}

function sliderMotorCallback(robot, slider) {
  var view3d = document.querySelector('#' + robot + '-robot-webots-view');
  var transform = view3d.querySelector('[id=' + slider.getAttribute('webots-transform-id') + ']');
  if (!transform)
    return; // This may occur when the x3d is loading.
  var axis = slider.getAttribute('webots-axis');
  var position = parseFloat(slider.getAttribute('webots-position'));

  if (slider.getAttribute('webots-type') === 'LinearMotor') {
    var translation = null;
    if (transform.hasAttribute('initalTranslation')) // Get initial translation.
      translation = transform.getAttribute('initalTranslation');
    else if (transform.hasAttribute('translation')) { // Store initial translation.
      translation = transform.getAttribute('translation');
      transform.setAttribute('initalTranslation', translation);
    }
    translation = translation.split(/[\s,]+/);
    axis = axis.split(/[\s,]+/);
    for (var a = 0; a < axis.length; a++)
      translation[a] = parseFloat(translation[a]) + parseFloat(axis[a]) * slider.value;

    transform.setAttribute('translation', translation.join(','));
  } else {
    var angle = 0.0;
    if (transform.hasAttribute('initialAngle')) // Get initial angle.
      angle = parseFloat(transform.getAttribute('initialAngle'));
    else if (transform.hasAttribute('rotation')) { // Store initial angle.
      angle = parseFloat(transform.getAttribute('rotation').split(/[\s,]+/)[3]);
      transform.setAttribute('initialAngle', angle);
    }
    angle += parseFloat(slider.value); // Add the slider value.
    angle -= position;

    // Apply the new axis-angle.
    axis = axis.split(' ').join(',');
    transform.setAttribute('rotation', axis + ',' + angle);
  }
}

function unhighlightX3DElement(robot) {
  var view3d = document.querySelector('#' + robot + '-robot-webots-view');
  var billboards = view3d.querySelectorAll('Transform[highlighted]');
  for (var b = 0; b < billboards.length; b++) {
    var billboard = billboards[b];
    billboard.parentNode.removeChild(billboard);
  }

  var materials = view3d.querySelectorAll('Material[highlighted]');
  for (var m = 0; m < materials.length; m++) {
    var material = materials[m];
    material.removeAttribute('highlighted');
    material.setAttribute('emissiveColor', material.getAttribute('emissiveColorBack'));
  }
}

function highlightX3DElement(robot, deviceElement) {
  unhighlightX3DElement(robot);

  var view3d = document.querySelector('#' + robot + '-robot-webots-view');
  var id = deviceElement.getAttribute('webots-transform-id');
  var transform = view3d.querySelector('[id=' + id + ']');
  if (transform) {
    if (deviceElement.getAttribute('webots-type') === 'LED') {
      var materialsIDs = deviceElement.getAttribute('ledMaterialsIDs').split(' ');
      for (var m = 0; m < materialsIDs.length; m++) {
        var materialID = materialsIDs[m];
        if (materialID) {
          var material = view3d.querySelector('[id=' + materialID + ']');
          if (material) {
            material.setAttribute('highlighted', 'true');
            material.setAttribute('emissiveColorBack', material.getAttribute('emissiveColor'));
            material.setAttribute('emissiveColor', deviceElement.getAttribute('targetColor'));
          }
        }
      }
    }

    var scale = parseFloat(view3d.querySelector('Viewpoint').getAttribute('robotScale')) / 50.0;
    var billboard = document.createElement('Transform');
    billboard.setAttribute('highlighted', 'true');
    if (deviceElement.hasAttribute('webots-transform-offset'))
      billboard.setAttribute('translation', deviceElement.getAttribute('webots-transform-offset'));

    billboard.innerHTML =
      '<Billboard axisOfRotation="0 0 0">\n' +
      '  <Shape>\n' +
      '    <Appearance sortType="transparent" sortKey="10000">\n' +
      '      <Material transparency="0.7"></Material>\n' +
      '      <DepthMode depthfunc="always"></DepthMode>\n' +
      '      <ImageTexture url="' + computeTargetPath() + '../css/images/center.png"></ImageTexture>\n' +
      '    </Appearance>\n' +
      '    <Plane size="' + scale + ' ' + scale + '"></Plane>\n' +
      '  </Shape>\n' +
      '</Billboard>\n';
    transform.appendChild(billboard);
  }
}

function estimateRobotScale(robot) {
  // Estimate roughly the robot scale based on the number of transform and their scaled translation.

  function x3domAttributeToFloatArray(el, name) {
    // Convert x3dom string attribute to an array of floats.
    if (!el.hasAttribute(name))
      return [];
    var arr = el.getAttribute(name).split(/[\s,]+/);
    for (var a = 0; a < arr.length; a++)
      arr[a] = parseFloat(arr[a]);
    return arr;
  }
  function estimateRobotScaleRec(el, s) {
    if (!el.tagName)
      return 0.0;
    // Get the max scale component.
    var scale = x3domAttributeToFloatArray(el, 'scale');
    if (scale.length > 0)
      s *= Math.max.apply(null, scale);
    // Get the max translation component.
    var max = 0.0;
    var translation = x3domAttributeToFloatArray(el, 'translation');
    if (translation.length > 0)
      max = s * Math.max.apply(null, translation);
    // Recursion
    for (var c = 0; c < el.childNodes.length; c++)
      max = Math.max(max, estimateRobotScaleRec(el.childNodes[c], s));
    return max;
  }

  var nTransforms = robot.querySelectorAll('transform').length + 1;
  return Math.log2(nTransforms) * estimateRobotScaleRec(robot, 1.0);
}

function createRobotComponent(view) {
  var webotsViewElements = document.querySelectorAll('.robot-webots-view');
  for (var e = 0; e < webotsViewElements.length; e++) { // foreach robot components of this page.
    var webotsViewElement = webotsViewElements[e];
    var robotName = webotsViewElement.getAttribute('id').replace('-robot-webots-view', '');
    var webotsView = new webots.View(webotsViewElement);
    webotsView.onready = function() { // When Webots View has been successfully loaded.
      // correct the URL textures.
      redirectTextures(webotsViewElement, robotName);
      // Store viewpoint.
      var viewpoint = webotsViewElement.querySelector('Viewpoint');
      viewpoint.setAttribute('initialOrientation', viewpoint.getAttribute('orientation'));
      viewpoint.setAttribute('initialPosition', viewpoint.getAttribute('position'));
      // Rough estimation of the robot scale.
      var robotScale = Math.max(0.05, estimateRobotScale(webotsViewElement));
      viewpoint.setAttribute('robotScale', robotScale);
    };

    // Load the robot X3D file.
    webotsView.open(computeTargetPath() + 'scenes/' + robotName + '/' + robotName + '.x3d');

    // Load the robot meta JSON file.
    $.ajax({
      type: 'GET',
      url: computeTargetPath() + 'scenes/' + robotName + '/' + robotName + '.meta.json',
      dataType: 'text',
      success: function(content) { // When successfully loaded.
        // Populate the device component from the JSON file.
        var deviceComponent = view.querySelector('#' + robotName + '-device-component');
        var data = JSON.parse(content);
        var categories = {};
        if (data['devices'].length === 0)
          toggleDeviceComponent(robotName);
        for (var d = 0; d < data['devices'].length; d++) {
          var device = data['devices'][d];
          var deviceName = device['name'];
          var deviceType = device['type'];

          // Create or retrieve the device category container.
          var category = null;
          if (deviceType in categories)
            category = categories[deviceType];
          else {
            category = document.createElement('div');
            category.classList.add('device-category');
            category.innerHTML = '<div class="device-title">' + deviceType + '</div>';
            deviceComponent.appendChild(category);
            categories[deviceType] = category;
          }

          // Create the new device.
          var deviceDiv = document.createElement('div');
          deviceDiv.classList.add('device');
          deviceDiv.setAttribute('onmouseover', 'highlightX3DElement("' + robotName + '", this)');
          deviceDiv.setAttribute('webots-type', deviceType);
          deviceDiv.setAttribute('webots-transform-id', device['transformID']);
          if ('transformOffset' in device) // The Device Transform has not been exported. The device is defined relatively to it's Transform parent.
            deviceDiv.setAttribute('webots-transform-offset', device['transformOffset']);
          deviceDiv.innerHTML = '<div class="device-name">' + deviceName + '</div>';

          // Create the new motor.
          if (deviceType.endsWith('Motor')) {
            var minLabel = document.createElement('div');
            minLabel.classList.add('motor-label');
            var maxLabel = document.createElement('div');
            maxLabel.classList.add('motor-label');
            var slider = document.createElement('input');
            slider.classList.add('motor-slider');
            slider.setAttribute('type', 'range');
            slider.setAttribute('step', 'any');
            if (device['minPosition'] === device['maxPosition']) { // infinite range.
              slider.setAttribute('min', -Math.PI);
              slider.setAttribute('max', Math.PI);
              minLabel.innerHTML = -3.14; // 2 decimals.
              maxLabel.innerHTML = 3.14;
            } else { // fixed range.
              var epsilon = 0.000001; // To solve Windows browser bugs on slider when perfectly equals to 0.
              slider.setAttribute('min', device['minPosition'] - epsilon);
              slider.setAttribute('max', device['maxPosition'] + epsilon);
              minLabel.innerHTML = Math.round(device['minPosition'] * 100) / 100; // 2 decimals.
              maxLabel.innerHTML = Math.round(device['maxPosition'] * 100) / 100;
            }
            slider.setAttribute('value', device['position']);
            slider.setAttribute('webots-position', device['position']);
            slider.setAttribute('webots-transform-id', device['transformID']);
            slider.setAttribute('webots-axis', device['axis']);
            slider.setAttribute('webots-type', deviceType);
            if (isInternetExplorer()) {
              slider.addEventListener('change', function(e) {
                sliderMotorCallback(robotName, e.target);
              });
            } else
              slider.setAttribute('oninput', 'sliderMotorCallback("' + robotName + '", this)');

            var motorDiv = document.createElement('div');
            motorDiv.classList.add('motor-component');
            motorDiv.appendChild(minLabel);
            motorDiv.appendChild(slider);
            motorDiv.appendChild(maxLabel);
            deviceDiv.appendChild(motorDiv);
          }

          // LED case: set the target color.
          if (deviceType === 'LED' && 'ledColors' in device && 'ledMaterialsIDs' in device) {
            // For now, simply take the first color. More complex mechanism could be implemented if required.
            var targetColor = (device['ledColors'].length > 0) ? device['ledColors'][0] : '0 0 1';
            deviceDiv.setAttribute('targetColor', targetColor);
            deviceDiv.setAttribute('ledMaterialsIDs', device['ledMaterialsIDs'].join(' '));
          }

          category.appendChild(deviceDiv);
        }
      },
      error: function(XMLHttpRequest, textStatus, errorThrown) {
        console.log('Status: ' + textStatus);
        console.log('Error: ' + errorThrown);
      }
    });
  }
}

// Open a tab component tab
function openTabFromEvent(evt, name) {
  // update links
  var a = document.querySelectorAll('a');
  for (var i = 0; i < a.length; i++) {
    var href = a[i].getAttribute('href');
    if (!href)
      continue;
    if (href.includes('tab=' + localSetup.tab))
      a[i].setAttribute('href', href.replace('tab=' + localSetup.tab, 'tab=' + name.toLowerCase()));
    else if (!href.startsWith('#'))
      a[i].setAttribute('href', href + (href.indexOf('?') > -1 ? '&' : '?') + 'tab=' + name.toLowerCase());
  }
  // open tab
  localSetup.tab = name.toLowerCase();
  updateBrowserUrl();
  openTab(evt.target.parentNode, localSetup.tab);
}

// Open a tab component tab
function openTab(tabcomponent, name) {
  var tabID = tabcomponent.getAttribute('tabid');

  var tabcontent = tabcomponent.parentNode.querySelectorAll('.tab-content[tabid="' + tabID + '"][name="' + name + '"]')[0];
  if (typeof tabcontent === 'undefined')
    return;

  var tabcontents = tabcomponent.parentNode.querySelectorAll('.tab-content[tabid="' + tabID + '"]');
  for (var i = 0; i < tabcontents.length; i++)
    tabcontents[i].style.display = 'none';

  var tablinks = tabcomponent.querySelectorAll('.tab-links');
  for (var j = 0; j < tablinks.length; j++)
    tablinks[j].classList.remove('active');

  var tabcontent = tabcomponent.parentNode.querySelectorAll('.tab-content[tabid="' + tabID + '"][name="' + name + '"]')[0];
  tabcontent.style.display = 'block';

  var tablink = tabcomponent.querySelectorAll('.tab-links[name="' + name + '"]')[0];
  tablink.classList.add('active');
}

function applyTabs() {
  var tabComponents = document.querySelectorAll('.tab-component');
  for (var k = 0; k < tabComponents.length; k++)
    openTab(tabComponents[k], localSetup.tab)
}

function renderGraphs() {
  for (var id in window.mermaidGraphs) {
    window.mermaidAPI.render(id, window.mermaidGraphs[id], function(svgCode, bindFunctions) {
      document.querySelector('#' + id + 'Div').innerHTML = svgCode;
      // set min-width to be 2/3 of the max-width otherwise the text might become too small
      var element = document.querySelector('#' + id);
      var style = element.getAttribute('style');
      element.setAttribute('style', style + ' min-width:' + Math.floor(0.66 * parseInt(style.split('max-width:')[1].split('px'))) + 'px;');
    });
  }
}

function applyAnchorIcons(view) {
  var elements = [];
  var tags = ['figcaption', 'h1', 'h2', 'h3', 'h4'];
  var i;
  for (i = 0; i < tags.length; i++) {
    var array = Array.prototype.slice.call(view.querySelectorAll(tags[i]));
    elements = elements.concat(array);
  }
  for (i = 0; i < elements.length; i++) {
    var el = elements[i];
    var name = null;
    if (el.parentNode && el.tagName.toLowerCase() === 'figcaption' && el.parentNode.tagName.toLowerCase() === 'figure')
      name = el.parentNode.getAttribute('name');
    else
      name = el.getAttribute('name');
    if (name) {
      el.classList.add('anchor-header');
      var span = document.createElement('span');
      span.classList.add('anchor-link-image');
      var a = document.createElement('a');
      a.setAttribute('href', '#' + name);
      a.classList.add('anchor-link');
      a.appendChild(span);
      el.insertBefore(a, el.firstChild);
    }
  }
}

function receiveMenuContent(menuContent) {
  // console.log('Menu content:\n\n');
  // console.log(menuContent);

  var menu = null;

  var converter = new showdown.Converter();
  var html = converter.makeHtml(menuContent);
  var div = document.createElement('div');
  div.innerHTML = html;

  for (var i = 0; i < div.childNodes.length; i++) {
    var child = div.childNodes[i];
    if (child && child.tagName && child.tagName.length > 0 && child.tagName.toLowerCase() === 'ul') {
      menu = child;
      break;
    }
  }

  if (!menu) {
    console.error('Cannot extract Menu.');
    return;
  }

  populateMenu(menu);
  redirectUrls(menu);
  updateSelection();
  addNavigationToBlogIfNeeded();
}

function updateMenuScrollbar() {
  var e = document.documentElement;
  var t = document.documentElement.scrollTop || document.body.scrollTop;
  var p = e.scrollHeight - t - e.clientHeight;
  if (p < 244) // 244 is the height in pixels of the footer of Cyberbotics web page
    document.querySelector('#left').style.height = (e.clientHeight - 290 + p) + 'px';
  else // 44 is the height in pixels of the header of Cyberbotics web page (44 + 244 = 290)
    document.querySelector('#left').style.height = 'calc(100% - 44px)';
}

function updateSelection() {
  var selected = changeMenuSelection();
  populateNavigation(selected);
  if (isCyberboticsUrl)
    updateMenuScrollbar();
}

function changeMenuSelection() {
  var menu = document.querySelector('#menu');
  var selecteds = [].slice.call(menu.querySelectorAll('.selected'));
  var i;
  var selected;
  for (i = 0; i < selecteds.length; i++) {
    selected = selecteds[i];
    selected.classList.remove('selected');
  }
  var as = menu.querySelectorAll('a');
  for (i = 0; i < as.length; i++) {
    var a = as[i];
    var href = a.getAttribute('href');
    var selection;
    if (!isCyberboticsUrl) {
      var pageIndex = href.indexOf('page=' + localSetup.page);
      // Notes:
      // - the string length test is done to avoid wrong positive cases
      //   where a page is a prefix of another.
      // - 5 matches with the 'page=' string length.
      if (pageIndex > -1 && (5 + pageIndex + localSetup.page.length) === href.length)
        selection = true;
      else
        selection = false;
    } else {
      var n = href.indexOf('?');
      if (n > -1)
        href = href.substring(0, n);
      n = href.indexOf('#');
      if (n > -1)
        href = href.substring(0, n);
      if (href.endsWith('/doc/' + localSetup.book + '/' + localSetup.page))
        selection = true;
      else
        selection = false;
    }
    if (selection) {
      selected = a.parentNode;
      selected.classList.add('selected');
      if (selected.parentNode.parentNode.tagName.toLowerCase() === 'li') {
        selected.parentNode.parentNode.classList.add('selected');
        var firstChild = selected.parentNode.parentNode.firstChild;
        if (firstChild.tagName.toLowerCase() === 'a')
          showAccodionItem(firstChild);
      } else
        showAccodionItem(a);
      return selected;
    }
  }
}

function populateNavigation(selected) {
  var next = document.querySelector('#next');
  var previous = document.querySelector('#previous');
  var up = document.querySelector('#up');
  var toc = document.querySelector('#toc');
  var as;

  toc.setAttribute('href', forgeUrl(localSetup.book, 'menu'));
  addDynamicLoadEvent(toc);

  if (!selected) {
    next.classList.add('disabled');
    previous.classList.add('disabled');
    up.classList.add('disabled');
    return;
  }

  if (next) {
    var nextElement = null;

    var nextLiSibling = selected.nextSibling;
    while (nextLiSibling) {
      if (nextLiSibling.tagName && nextLiSibling.tagName.toLowerCase() === 'li')
        break;
      nextLiSibling = nextLiSibling.nextSibling;
    }
    if (nextLiSibling) {
      as = nextLiSibling.querySelectorAll('a');
      if (as.length > 0)
        nextElement = as[0];
    }

    if (nextElement) {
      next.classList.remove('disabled');
      next.setAttribute('href', nextElement.getAttribute('href'));
      addDynamicLoadEvent(next);
    } else
      next.classList.add('disabled');
  }

  if (previous) {
    var previousElement = null;

    var previousLiSibling = selected.previousSibling;
    while (previousLiSibling) {
      if (previousLiSibling.tagName && previousLiSibling.tagName.toLowerCase() === 'li')
        break;
      previousLiSibling = previousLiSibling.previousSibling;
    }
    if (previousLiSibling) {
      as = previousLiSibling.querySelectorAll('a');
      if (as.length > 0)
        previousElement = as[0];
    }

    if (previousElement) {
      previous.classList.remove('disabled');
      previous.setAttribute('href', previousElement.getAttribute('href'));
      addDynamicLoadEvent(previous);
    } else
      previous.classList.add('disabled');
  }

  if (up) {
    var upElement = null;
    var parentLi = null;
    if (selected.parentNode.parentNode.tagName.toLowerCase() === 'li')
      parentLi = selected.parentNode.parentNode;
    if (parentLi) {
      as = parentLi.querySelectorAll('a');
      if (as.length > 0)
        upElement = as[0];
    }

    if (upElement) {
      up.classList.remove('disabled');
      up.setAttribute('href', upElement.getAttribute('href'));
      addDynamicLoadEvent(up);
    } else {
      up.setAttribute('href', forgeUrl(localSetup.book, 'index'));
      addDynamicLoadEvent(up);
      up.classList.remove('disabled');
    }
  }
}

function populateMenu(menu) {
  // make in order that the <li> tags above the <a> are also clickable
  var lis = menu.querySelectorAll('li');
  for (var i = 0; i < lis.length; i++) {
    var li = lis[i];
    li.addEventListener('click',
      function(event) {
        var as = event.target.querySelectorAll('a');
        if (as.length > 0)
          aClick(as[0]);
      }
    );
  }

  var menuDiv = document.querySelector('#menu');
  menuDiv.appendChild(menu);

  menu.setAttribute('id', 'accordion');
  $('#accordion > li > a').click(function() {
    showAccodionItem(this);
  });
}

function showAccodionItem(item) {
  if (!$(item).hasClass('active')) {
    $('#accordion li ul').slideUp();
    $(item).next().slideToggle();
    $('#accordion li a').removeClass('active');
    $(item).addClass('active');
  }
}

function getMDFile() {
  var target = computeTargetPath() + localSetup.page + '.md';
  console.log('Get MD file: ' + target);
  $.ajax({
    type: 'GET',
    url: target,
    dataType: 'text',
    success: populateViewDiv,
    error: function(XMLHttpRequest, textStatus, errorThrown) {
      console.log('Status: ' + textStatus);
      console.log('Error: ' + errorThrown);
      var mainPage = 'index';
      // get the main page instead
      if (localSetup.page !== mainPage) {
        localSetup.page = mainPage;
        getMDFile();
      }
    }
  });
}

function getMenuFile() {
  var target = computeTargetPath() + 'menu.md';
  console.log('Get menu file: ' + target);
  $.ajax({
    type: 'GET',
    url: target,
    dataType: 'text',
    success: receiveMenuContent,
    error: function(XMLHttpRequest, textStatus, errorThrown) {
      console.log('Status: ' + textStatus);
      console.log('Error: ' + errorThrown);
    }
  });
}

function extractAnchor(url) {
  var match = /#([\w-]+)/.exec(url);
  if (match && match.length === 2)
    return match[1];
  return '';
}

// width: in pixels
function setHandleWidth(width) {
  handle.left.css('width', width + 'px');
  handle.handle.css('left', width + 'px');
  handle.center.css('left', width + 'px');
  handle.center.css('width', 'calc(100% - ' + width + 'px)');
}

function initializeHandle() {
  // inspired from: http://stackoverflow.com/questions/17855401/how-do-i-make-a-div-width-draggable
  handle = {}; // structure where all the handle info is stored

  handle.left = $('#left');
  handle.center = $('#center');
  handle.handle = $('#handle');
  handle.container = $('#webots-doc');

  // dimension bounds of the handle in pixels
  handle.min = 0;
  handle.minThreshold = 75; // under this threshold, the handle is totally hidden
  if (localSetup.menuWidth && localSetup.menuWidth !== '')
    handle.initialWidth = localSetup.menuWidth;
  else
    handle.initialWidth = handle.left.width();
  handle.max = Math.max(250, handle.initialWidth);

  handle.isResizing = false;
  handle.lastDownX = 0;

  if (isCyberboticsUrl) {
    handle.left.addClass('cyberbotics');
    handle.handle.addClass('cyberbotics');
    handle.center.addClass('cyberbotics');
  } else {
    handle.left.addClass('default');
    handle.handle.addClass('default');
    handle.center.addClass('default');
  }

  setHandleWidth(handle.initialWidth);

  handle.handle.on('mousedown', function(e) {
    handle.isResizing = true;
    handle.lastDownX = e.clientX;
    handle.container.css('user-select', 'none');
  }).on('dblclick', function(e) {
    if (handle.left.css('width').startsWith('0'))
      setHandleWidth(handle.initialWidth);
    else
      setHandleWidth(0);
  });

  $(document).on('mousemove', function(e) {
    if (!handle.isResizing)
      return;
    var mousePosition = e.clientX - handle.container.offset().left; // in pixels
    if (mousePosition < handle.minThreshold / 2) {
      setHandleWidth(0);
      return;
    } else if (mousePosition < handle.minThreshold)
      return;
    if (mousePosition < handle.min || mousePosition > handle.max)
      return;
    setHandleWidth(mousePosition);
  }).on('mouseup', function(e) {
    handle.isResizing = false;
    handle.container.css('user-select', 'auto');
  });
}

window.onscroll = function() {
  if (!isCyberboticsUrl)
    return;
  updateMenuScrollbar();
};

document.addEventListener('DOMContentLoaded', function() {
  window.mermaidAPI.initialize({startOnLoad: false});
  initializeHandle();

  if (!isCyberboticsUrl) {
    if (!localSetup.url)
      localSetup.url = getGETQueryValue('url', 'https://raw.githubusercontent.com/omichel/webots/');
    if (!localSetup.book)
      localSetup.book = getGETQueryValue('book', 'guide');
    if (!localSetup.page)
      localSetup.page = getGETQueryValue('page', 'index');
    if (!localSetup.anchor)
      localSetup.anchor = window.location.hash.substring(1);
    if (!localSetup.branch)
      localSetup.branch = getGETQueryValue('branch', 'master');
    if (!localSetup.tab)
      localSetup.tab = getGETQueryValue('tab', '').toLowerCase();
  }

  // prevent FOUC for blog
  if (localSetup.book == "blog") {
    var center = document.querySelector('#center');
    center.setAttribute('class', 'blog');
    setHandleWidth(0);
  }

  addContributionBanner();
  applyToTitleDiv();
  getMDFile();
  getMenuFile();
});
