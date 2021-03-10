import {DialogWindow} from './dialog_window.js';
import {DefaultUrl} from './default_url.js';
import {webots} from './webots.js';

class HelpWindow extends DialogWindow {
  constructor(parent, mobile, webotsDocUrl) {
    super(parent, mobile);

    this.panel.id = 'webotsHelp';
    this.panel.style.overflow = 'hidden';
    this.panel.className += 'webotsTabContainer';
    this.tabs = document.createElement('div');
    this.tabs.id = 'webotsHelpTabs';
    this.tabs.className += 'webotsTabs';
    this.tabsHeader = document.createElement('ul');
    this.tabs.appendChild(this.tabsHeader);
    this.panel.appendChild(this.tabs);

    if (webotsDocUrl) {
      const header = document.createElement('li');
      header.innerHTML = '<a href="#webotsHelpReference">Webots Reference Manual</a>';
      this.tabsHeader.appendChild(header);
      const page = document.createElement('div');
      page.id = 'webotsHelpReference';
      page.innerHTML = '<iframe src="' + webotsDocUrl + '"></iframe>';
      this.tabs.appendChild(page);
      $('#webotsHelpTabs').tabs();
    }

    const clampedSize = DialogWindow.clampDialogSize({left: 5, top: 5, width: 600, height: 600});
    this.params.width = clampedSize.width;
    this.params.height = clampedSize.height;
    this.params.close = () => { $('#helpButton').removeClass('toolBarButtonActive'); };
    this.params.position = {at: 'right-5 top+5', my: 'right top', of: this.parent};
    this.params.title = 'Help';

    $(this.panel).dialog(this.params).dialogExtend({maximizable: !mobile});

    const finalize = () => {
      $('#webotsHelpTabs').tabs('refresh');
      $('#webotsHelpTabs').tabs('option', 'active', 0);
      $(this.panel).dialog('open');
    };
    const currentUrl = DefaultUrl.currentScriptUrl();
    let query = '';
    if (webots.showRun)
      query = '?run=true';
    $.ajax({
      url: currentUrl + 'help.php' + query,
      success: (data) => {
        // Fix the img src relative URLs.
        const html = data.replace(/ src="images/g, ' src="' + currentUrl + 'images');
        const header = document.createElement('li');
        header.innerHTML = '<a href="#webotsHelpGuide">User Guide</a>';
        $(this.tabsHeader).prepend(header);
        const page = document.createElement('div');
        page.id = 'webotsHelpGuide';
        page.innerHTML = html;
        if (document.getElementById('webotsHelpReference'))
          $('#webotsHelpReference').before(page);
        else {
          this.tabs.appendChild(page);
          $('#webotsHelpTabs').tabs();
        }
        finalize();
      },
      error: finalize
    });
  }
}

export {HelpWindow};
