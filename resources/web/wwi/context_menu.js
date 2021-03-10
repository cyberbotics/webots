'use strict';

class ContextMenu {
  constructor(authenticatedUser, parentObject, selection) {
    this.object = null;
    this.visible = false;
    this.authenticatedUser = authenticatedUser;

    // Callbacks
    this.onFollowObject = null;
    this.onEditController = null;
    this.onOpenRobotWindow = null;
    this.isRobotWindowValid = null;

    // Create context menu.
    const domElement = document.createElement('ul');
    domElement.id = 'contextMenu';
    domElement.innerHTML = "<li class='ui-widget-header'><div id='contextMenuTitle'>Object</div></li>" +
                           "<li id='contextMenuFollow'><div>Follow</div></li>" +
                           "<li id='contextMenuUnfollow'><div>Unfollow</div></li>" +
                           "<li><div class='ui-state-disabled'>Zoom</div></li>" +
                           '<hr>' +
                           "<li id='contextMenuRobotWindow'><div id='contextMenuRobotWindowDiv'>Robot window</div></li>" +
                           "<li id='contextMenuEditController'><div id='contextMenuEditControllerDiv'>Edit controller</div></li>" +
                           "<li><div class='ui-state-disabled'>Delete</div></li>" +
                           "<li><div class='ui-state-disabled'>Properties</div></li>" +
                           '<hr>' +
                           "<li id='contextMenuHelp'><div id='contextMenuHelpDiv' class='ui-state-disabled'>Help...</div></li>";
    $(parentObject).append(domElement);
    $('#contextMenu').menu({items: '> :not(.ui-widget-header)'});
    $('#contextMenu').css('position', 'absolute');
    $('#contextMenu').css('z-index', 1);
    $('#contextMenu').css('display', 'none');

    $('#contextMenu').on('menuselect', (event, ui) => {
      if (ui.item.children().hasClass('ui-state-disabled'))
        return;
      const id = ui.item.attr('id');
      if (id === 'contextMenuFollow') {
        if (typeof this.onFollowObject === 'function')
          this.onFollowObject(this.object.name, '1'); // object followed with mode 1
      } else if (id === 'contextMenuUnfollow') {
        if (typeof this.onFollowObject === 'function')
          this.onFollowObject('none', '0'); // object not followed
      } else if (id === 'contextMenuEditController') {
        const controller = this.object.controller;
        $('#webotsEditor').dialog('open');
        $('#webotsEditor').dialog('option', 'title', 'Controller: ' + controller);
        if (typeof this.onEditController === 'function')
          this.onEditController(controller);
      } else if (id === 'contextMenuRobotWindow') {
        const robotName = this.object.name;
        if (typeof this.onOpenRobotWindow === 'function')
          this.onOpenRobotWindow(robotName);
      } else if (id === 'contextMenuHelp')
        window.open(this.object.docUrl, '_blank');
      else
        console.log('Unknown menu item: ' + id);
      $('#contextMenu').css('display', 'none');
    });
  }

  disableEdit() {
    $('#contextMenuRobotWindowDiv').addClass('ui-state-disabled');
    $('#contextMenuEditControllerDiv').addClass('ui-state-disabled');
  }

  toggle() {
    const visible = this.visible;
    if (visible)
      this.hide();
    return visible;
  }

  hide() {
    $('#contextMenu').css('display', 'none');
    this.visible = false;
  }

  // object = {'id', 'name', 'controller', 'docUrl', 'follow'}
  show(object, position) {
    this.object = object;
    if (typeof object === 'undefined')
      return;

    let title = object.name;
    if (title == null || title === '')
      title = 'Object';

    $('#contextMenuTitle').html(title);
    const controller = object.controller;
    if (controller && controller !== '') { // the current selection is a robot
      $('#contextMenuEditController').css('display', 'inline');
      if (controller === 'void' || controller.length === 0 || !this.authenticatedUser)
        $('#contextMenuEditController').children().addClass('ui-state-disabled');
      const robotName = object.name;
      let isValid = false;
      if (typeof this.isRobotWindowValid === 'function')
        this.isRobotWindowValid(robotName, (result) => { isValid = result; });
      if (isValid)
        $('#contextMenuRobotWindow').css('display', 'inline');
      else
        $('#contextMenuRobotWindow').css('display', 'none');
    } else {
      $('#contextMenuEditController').css('display', 'none');
      $('#contextMenuRobotWindow').css('display', 'none');
    }

    if (object.follow < 0) { // follow option not supported
      $('#contextMenuFollow').css('display', 'none');
      $('#contextMenuUnfollow').css('display', 'none');
    } else if (object.follow === 0) { // object not followed
      $('#contextMenuFollow').css('display', 'inline');
      $('#contextMenuUnfollow').css('display', 'none');
    } else { // object followed
      $('#contextMenuFollow').css('display', 'none');
      $('#contextMenuUnfollow').css('display', 'inline');
    }

    if (this.object.docUrl)
      $('#contextMenuHelpDiv').removeClass('ui-state-disabled');
    else
      $('#contextMenuHelpDiv').addClass('ui-state-disabled');

    // Ensure that the context menu is completely visible.
    const w = $('#contextMenu').width();
    const h = $('#contextMenu').height();
    const maxWidth = $('#playerDiv').width();
    const maxHeight = $('#playerDiv').height();
    let left;
    let top;
    if (maxWidth != null && (w + position.x) > maxWidth)
      left = maxWidth - w;
    else
      left = position.x;
    if (maxHeight != null && (h + position.y) > maxHeight)
      top = maxHeight - h - $('#toolBar').height();
    else
      top = position.y;
    $('#contextMenu').css('left', left + 'px');
    $('#contextMenu').css('top', top + 'px');
    $('#contextMenu').css('display', 'block');
    this.visible = true;
  }
}

export {ContextMenu};
