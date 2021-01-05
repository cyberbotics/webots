import { DefaultLoadingManager } from './loading_manager.js';
/**
 * A low level class for loading resources with XMLHttpRequest, used internaly by most loaders.
 * It can also be used directly to load any file type that does not have a loader.
 * @constructor
 * @memberof zen3d
 * @param {zen3d.LoadingManager} manager — The loadingManager for the loader to use. Default is zen3d.DefaultLoadingManager.
 */
function FileLoader(manager) {
	this.path = undefined;
	this.responseType = undefined;
	this.withCredentials = undefined;
	this.mimeType = undefined;
	this.requestHeader = undefined;
	this.manager = (manager !== undefined) ? manager : DefaultLoadingManager;
}
Object.assign(FileLoader.prototype, /** @lends zen3d.FileLoader.prototype */{
	/**
     * Load the URL and pass the response to the onLoad function.
     * @param {string} url — the path or URL to the file. This can also be a Data URI.
     * @param {Function} [onLoad=] — Will be called when loading completes. The argument will be the loaded response.
     * @param {Function} [onProgress=] — Will be called while load progresses. The argument will be the XMLHttpRequest instance, which contains .total and .loaded bytes.
     * @param {Function} [onError=] — Will be called if an error occurs.
     */
	load: function(url, onLoad, onProgress, onError) {
		if (url === undefined) url = '';
		if (this.path != undefined) url = this.path + url;
		url = this.manager.resolveURL(url);
		var scope = this;
		// Check for data: URI
		var dataUriRegex = /^data:(.*?)(;base64)?,(.*)$/;
		var dataUriRegexResult = url.match(dataUriRegex);
		var request;
		if (dataUriRegexResult) { // Safari can not handle Data URIs through XMLHttpRequest so process manually
			var mimeType = dataUriRegexResult[1];
			var isBase64 = !!dataUriRegexResult[2];
			var data = dataUriRegexResult[3];
			data = decodeURIComponent(data);
			if (isBase64) data = atob(data); // decode base64
			try {
				var response;
				var responseType = (this.responseType || '').toLowerCase();
				switch (responseType) {
				case 'arraybuffer':
				case 'blob':
					response = new ArrayBuffer(data.length);
					var view = new Uint8Array(response);
					for (var i = 0; i < data.length; i++) {
						view[i] = data.charCodeAt(i);
					}
					if (responseType === 'blob') {
						response = new Blob([response], {
							type: mimeType
						});
					}
					break;
				case 'document':
					var parser = new DOMParser();
					response = parser.parseFromString(data, mimeType);
					break;
				case 'json':
					response = JSON.parse(data);
					break;
				default: // 'text' or other
					response = data;
					break;
				}
				// Wait for next browser tick
				setTimeout(function() {
					if (onLoad) onLoad(response);
					scope.manager.itemEnd(url);
				}, 0);
			} catch (error) {
				// Wait for next browser tick
				setTimeout(function() {
					onError && onError(error);
					scope.manager.itemError(url);
					scope.manager.itemEnd(url);
				}, 0);
			}
		} else {
			request = new XMLHttpRequest();
			request.open('GET', url, true);
			request.addEventListener('load', function(event) {
				var response = this.response;
				if (this.status === 200) {
					if (onLoad) onLoad(response);
					scope.manager.itemEnd(url);
				} else if (this.status === 0) {
					// Some browsers return HTTP Status 0 when using non-http protocol
					// e.g. 'file://' or 'data://'. Handle as success.
					console.warn('zen3d.FileLoader: HTTP Status 0 received.');
					if (onLoad) onLoad(response);
					scope.manager.itemEnd(url);
				} else {
					if (onError) onError(event);
					scope.manager.itemError(url);
					scope.manager.itemEnd(url);
				}
			}, false);
			if (onProgress !== undefined) {
				request.addEventListener('progress', function(event) {
					onProgress(event);
				}, false);
			}
			if (onError !== undefined) {
				request.addEventListener('error', function(event) {
					onError(event);
					scope.manager.itemError(url);
					scope.manager.itemEnd(url);
				}, false);
			}
			if (this.responseType !== undefined) request.responseType = this.responseType;
			if (this.withCredentials !== undefined) request.withCredentials = this.withCredentials;
			if (request.overrideMimeType) request.overrideMimeType(this.mimeType !== undefined ? this.mimeType : 'text/plain');
			for (var header in this.requestHeader) {
				request.setRequestHeader(header, this.requestHeader[header]);
			}
			request.send(null);
		}
		scope.manager.itemStart(url);
		return request;
	},
	/**
     * Set the base path or URL from which to load files.
     * This can be useful if you are loading many models from the same directory.
     * @param {string} value
     * @return {zen3d.FileLoader}
     */
	setPath: function(value) {
		this.path = value;
		return this;
	},
	/**
     * Change the response type. Valid values are:
     * text or empty string (default) - returns the data as string.
     * arraybuffer - loads the data into a ArrayBuffer and returns that.
     * blob - returns the data as a Blob.
     * document - parses the file using the DOMParser.
     * json - parses the file using JSON.parse.
     * @param {string} value
     * @return {zen3d.FileLoader}
     */
	setResponseType: function(value) {
		this.responseType = value;
		return this;
	},
	/**
     * Whether the XMLHttpRequest uses credentials such as cookies, authorization headers or TLS client certificates.
     * See {@link https://developer.mozilla.org/en-US/docs/Web/API/XMLHttpRequest/withCredentials XMLHttpRequest.withCredentials}.
     * Note that this has no effect if you are loading files locally or from the same domain.
     * @param {boolean} value
     * @return {zen3d.FileLoader}
     */
	setWithCredentials: function(value) {
		this.withCredentials = value;
		return this;
	},
	/**
     * Set the expected mimeType of the file being loaded.
     * Note that in many cases this will be determined automatically, so by default it is undefined.
     * @param {string} value
     * @return {zen3d.FileLoader}
     */
	setMimeType: function(value) {
		this.mimeType = value;
		return this;
	},
	/**
     * The request header used in HTTP request.
     * Default is undefined.
     * @param {string} value
     * @return {zen3d.FileLoader}
     */
	setRequestHeader: function(value) {
		this.requestHeader = value;
		return this;
	}
});
export { FileLoader };
