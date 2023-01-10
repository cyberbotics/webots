/* global showdown */

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

  // renderGraphs();
  redirectImages(view, imgPrefix);
  // updateModalEvents(view);
  // redirectUrls(view);
  // collapseMovies(view);
  //
  // applyAnchorIcons(view);
  // highlightCode(view);
  //
  // updateSelection();
  createIndex(view);
  //
  // setupBlogFunctionalitiesIfNeeded();
  // addNavigationToBlogIfNeeded();
  //
  // const images = view.querySelectorAll('img');
  // if (images.length > 0) {
  //   // apply the anchor only when the images are loaded,
  //   // otherwise, the anchor can be overestimated.
  //   const lastImage = images[images.length - 1];
  //   lastImage.onload = () => applyAnchor();
  // } else
  //   applyAnchor();
  // applyTabs();
}

function createIndex(view) {
  // Note: the previous index is cleaned up when the parent title is destroyed.

  // Get all the view headings.
  const headings = [].slice.call(view.querySelectorAll('h1, h2, h3, h4'));

  // Do not create too small indexes.
  const content = document.getElementsByClassName('proto-doc')[0];
  if ((content.offsetHeight < 2 * window.innerHeight || headings.length < 4) && headings.length < 2)
    return;

  let level = parseInt(headings[0].tagName[1]) + 1; // current heading level.

  // Create an empty index, and insert it before the second heading.
  const indexTitle = document.createElement('h' + level);
  indexTitle.textContent = 'Index';
  indexTitle.setAttribute('id', 'indexTitle');
  headings[0].parentNode.insertBefore(indexTitle, headings[1]);
  let ul = document.createElement('ul');
  ul.setAttribute('id', 'index');
  headings[0].parentNode.insertBefore(ul, headings[1]);

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

export {populateProtoViewDiv};
