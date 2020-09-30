/* global DialogWindow, DefaultUrl, webots */
'use strict';

class HelpWindow extends DialogWindow { // eslint-disable-line no-unused-vars
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
      var header = document.createElement('li');
      header.innerHTML = '<a href="#webotsHelpReference">Webots Reference Manual</a>';
      this.tabsHeader.appendChild(header);
      var page = document.createElement('div');
      page.id = 'webotsHelpReference';
      page.innerHTML = '<iframe src="' + webotsDocUrl + '"></iframe>';
      this.tabs.appendChild(page);
      $('#webotsHelpTabs').tabs();
    }

    var clampedSize = DialogWindow.clampDialogSize({left: 5, top: 5, width: 600, height: 600});
    this.params.width = clampedSize.width;
    this.params.height = clampedSize.height;
    this.params.close = () => { $('#helpButton').removeClass('toolBarButtonActive'); };
    this.params.position = {at: 'right-5 top+5', my: 'right top', of: this.parent};
    this.params.title = 'Help';

    $(this.panel).dialog(this.params).dialogExtend({maximizable: !mobile});

    var finalize = () => {
      $('#webotsHelpTabs').tabs('refresh');
      $('#webotsHelpTabs').tabs('option', 'active', 0);
      $(this.panel).dialog('open');
    };
    var currentUrl = DefaultUrl.currentScriptUrl();
    var query = '';
    if (webots.showRun)
      query = '?run=true';
    $.ajax({
      url: currentUrl + 'help.php' + query,
      success: (data) => {
        // Fix the img src relative URLs.
        var html = data.replace(/ src="images/g, ' src="' + currentUrl + '/images');
        var header = document.createElement('li');
        header.innerHTML = '<a href="#webotsHelpGuide">User Guide</a>';
        $(this.tabsHeader).prepend(header);
        var page = document.createElement('div');
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
