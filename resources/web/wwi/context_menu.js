'use strict';

class ContextMenu { // eslint-disable-line no-unused-vars
  constructor(authenticatedUser, parentObject, selection) {
    this.object = null;
    this.visible = false;
    this.authenticatedUser = authenticatedUser;

    // Callbacks
    this.onFollowObject = null;
    this.onEditController = null;
    this.onOpenRobotWindow = null;
    this.isFollowedObject = null;
    this.isRobotWindowValid = null;

    // Create context menu.
    var domElement = document.createElement('ul');
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
      var id = ui.item.attr('id');
      if (id === 'contextMenuFollow') {
        if (typeof this.onFollowObject === 'function')
          this.onFollowObject(this.object.name);
      } else if (id === 'contextMenuUnfollow') {
        if (typeof this.onFollowObject === 'function')
          this.onFollowObject('none');
      } else if (id === 'contextMenuEditController') {
        var controller = this.object.userData.controller;
        $('#webotsEditor').dialog('open');
        $('#webotsEditor').dialog('option', 'title', 'Controller: ' + controller);
        if (typeof this.onEditController === 'function')
          this.onEditController(controller);
      } else if (id === 'contextMenuRobotWindow') {
        var robotName = this.object.userData.name;
        if (typeof this.onOpenRobotWindow === 'function')
          this.onOpenRobotWindow(robotName);
      } else if (id === 'contextMenuHelp')
        window.open(this.object.userData.docUrl, '_blank');
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
    var visible = this.visible;
    if (visible)
      this.hide();
    return visible;
  }

  hide() {
    $('#contextMenu').css('display', 'none');
    this.visible = false;
  }

  show(object, position) {
    this.object = object;
    if (typeof object === 'undefined')
      return;

    var title = object.userData.name;
    if (title == null || title === '')
      title = 'Object';

    $('#contextMenuTitle').html(title);
    var controller = object.userData.controller;
    if (controller && controller !== '') { // the current selection is a robot
      $('#contextMenuEditController').css('display', 'inline');
      if (controller === 'void' || controller.length === 0 || !this.authenticatedUser)
        $('#contextMenuEditController').children().addClass('ui-state-disabled');
      var robotName = object.userData.name;
      var isValid = false;
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

    var isFollowed = false;
    if (typeof this.isFollowedObject === 'function')
      this.isFollowedObject(object, (result) => { isFollowed = result; });
    if (isFollowed) {
      $('#contextMenuFollow').css('display', 'none');
      $('#contextMenuUnfollow').css('display', 'inline');
    } else {
      $('#contextMenuFollow').css('display', 'inline');
      $('#contextMenuUnfollow').css('display', 'none');
    }

    if (this.object.userData.docUrl)
      $('#contextMenuHelpDiv').removeClass('ui-state-disabled');
    else
      $('#contextMenuHelpDiv').addClass('ui-state-disabled');

    // Ensure that the context menu is completely visible.
    var w = $('#contextMenu').width();
    var h = $('#contextMenu').height();
    var maxWidth = $('#playerDiv').width();
    var maxHeight = $('#playerDiv').height();
    var left;
    var top;
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
