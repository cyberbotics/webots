/**
 * Handles and keeps track of loaded and pending data. A default global instance of this class is created and used by loaders if not supplied manually - see {@link zen3d.DefaultLoadingManager}.
 * In general that should be sufficient, however there are times when it can be useful to have seperate loaders - for example if you want to show seperate loading bars for objects and textures.
 * @constructor
 * @author mrdoob / http://mrdoob.com/
 * @param {Function} onLoad — (optional) this function will be called when all loaders are done.
 * @param {Function} onProgress — (optional) this function will be called when an item is complete.
 * @param {Function} onError — (optional) this function will be called a loader encounters errors.
 */
function LoadingManager(onLoad, onProgress, onError) {
	var scope = this;
	var isLoading = false;
	var itemsLoaded = 0;
	var itemsTotal = 0;
	var urlModifier = undefined;
	// Refer to #5689 for the reason why we don't set .onStart
	// in the constructor
	this.onStart = undefined;
	this.onLoad = onLoad;
	this.onProgress = onProgress;
	this.onError = onError;
	this.itemStart = function (url) {
		itemsTotal++;
		if (isLoading === false) {
			if (scope.onStart !== undefined) {
				scope.onStart(url, itemsLoaded, itemsTotal);
			}
		}
		isLoading = true;
	};
	this.itemEnd = function (url) {
		itemsLoaded++;
		if (scope.onProgress !== undefined) {
			scope.onProgress(url, itemsLoaded, itemsTotal);
		}
		if (itemsLoaded === itemsTotal) {
			isLoading = false;
			if (scope.onLoad !== undefined) {
				scope.onLoad();
			}
		}
	};
	this.itemError = function (url) {
		if (scope.onError !== undefined) {
			scope.onError(url);
		}
	};
	this.resolveURL = function (url) {
		if (urlModifier) {
			return urlModifier(url);
		}
		return url;
	};
	this.setURLModifier = function (transform) {
		urlModifier = transform;
		return this;
	};
}
var DefaultLoadingManager = new LoadingManager();
export { DefaultLoadingManager, LoadingManager };
