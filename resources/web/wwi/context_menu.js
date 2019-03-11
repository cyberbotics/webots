'use strict';

function ContextMenu(authenticatedUser, parentObject, selection) {
  this.object = null;
  this.visible = false;
  this.authenticatedUser = authenticatedUser;

  // callbacks
  this.onFollowObject = null;
  this.onEditController = null;
  this.onOpenRobotWindow = null;
  this.isFollowedObject = null;
  this.isRobotWindowValid = null;

  // create context menu
  var domElement = document.createElement('ul');
  domElement.id = 'contextMenu';
  domElement.innerHTML = "<li class='ui-widget-header'><div id='contextMenuTitle'>Object</div></li>" +
                         "<li id='contextMenuFollow'><div>Follow</div></li>" +
                         "<li id='contextMenuUnfollow'><div>Unfollow</div></li>" +
                         "<li><div class='ui-state-disabled'>Zoom</div></li>" +
                         "<li id='contextMenuRobotWindow'><div id='contextMenuRobotWindowDiv'>Robot window</div></li>" +
                         "<li id='contextMenuEditController'><div id='contextMenuEditControllerDiv'>Edit controller</div></li>" +
                         "<li><div class='ui-state-disabled'>Delete</div></li>" +
                         "<li><div class='ui-state-disabled'>Properties</div></li>";
  $(parentObject).append(domElement);
  $('#contextMenu').menu({items: '> :not(.ui-widget-header)'});
  $('#contextMenu').css('position', 'absolute');
  $('#contextMenu').css('z-index', 1);
  $('#contextMenu').css('display', 'none');

  var that = this;
  $('#contextMenu').on('menuselect', function(event, ui) {
    if (ui.item.children().hasClass('ui-state-disabled'))
      return;
    var id = ui.item.attr('id');
    if (id === 'contextMenuFollow') {
      if (that.onFollowObject)
        that.onFollowObject(that.object.name);
    } else if (id === 'contextMenuUnfollow') {
      if (that.onFollowObject)
        that.onFollowObject('none');
    } else if (id === 'contextMenuEditController') {
      var controller = that.object.userData.controller;
      $('#webotsEditor').dialog('open');
      $('#webotsEditor').dialog('option', 'title', 'Controller: ' + controller);
      if (that.onEditController)
        that.onEditController(controller);
    } else if (id === 'contextMenuRobotWindow') {
      var robotName = that.object.userData.name;
      if (that.onOpenRobotWindow)
        that.onOpenRobotWindow(robotName);
    } else
      console.log('Unknown menu item: ' + id);
    $('#contextMenu').css('display', 'none');
  });
};

ContextMenu.prototype = {
  constructor: ContextMenu,

  disableEdit: function() {
    $('#contextMenuRobotWindowDiv').addClass('ui-state-disabled');
    $('#contextMenuEditControllerDiv').addClass('ui-state-disabled');
  },

  toggle: function() {
    var visible = this.visible;
    if (visible)
      this.hide();
    return visible;
  },

  hide: function() {
    $('#contextMenu').css('display', 'none');
    this.visible = false;
  },

  show: function(object, position) {
    this.object = object;
    var title = object.userData.name;
    if (title == null || title === '') {
      title = object.userData.name.defName;
      if (title == null || title === '')
        title = 'Object';
    }

    $('#contextMenuTitle').html(title);
    var controller = object.userData.controller;
    if (controller && controller !== '') { // the current selection is a robot
      $('#contextMenuEditController').css('display', 'inline');
      if (controller === 'void' || controller.length === 0 || this.authenticatedUser)
        $('#contextMenuEditController').children().addClass('ui-state-disabled');
      var robotName = object.userData.name;
      var isValid = false;
      if (this.isRobotWindowValid)
        this.isRobotWindowValid(robotName, function(result) { isValid = result; });
      if (isValid)
        $('#contextMenuRobotWindow').css('display', 'inline');
      else
        $('#contextMenuRobotWindow').css('display', 'none');
    } else {
      $('#contextMenuEditController').css('display', 'none');
      $('#contextMenuRobotWindow').css('display', 'none');
    }

    var isFollowed = false;
    if (this.isFollowedObject)
      this.isFollowedObject(object, function(result) { isFollowed = result; });
    if (isFollowed) {
      $('#contextMenuFollow').css('display', 'none');
      $('#contextMenuUnfollow').css('display', 'inline');
    } else {
      $('#contextMenuFollow').css('display', 'inline');
      $('#contextMenuUnfollow').css('display', 'none');
    }

    // ensure that the context menu is completely visible
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
};
