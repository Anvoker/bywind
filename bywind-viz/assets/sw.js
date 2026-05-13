var cacheName = 'bywind-viz';
// Trunk emits the wasm/js bundle with a content hash in the filename
// (e.g. bywind-viz-<hash>_bg.wasm), so it can't be pre-cached by literal
// name here. Pre-cache only the shell; the fetch handler below picks up
// the hashed assets from the network and the next visit serves them
// from the HTTP cache.
var filesToCache = [
  './',
  './index.html',
];

/* Start the service worker and cache all of the app's content */
self.addEventListener('install', function (e) {
  e.waitUntil(
    caches.open(cacheName).then(function (cache) {
      return cache.addAll(filesToCache);
    })
  );
});

/* Serve cached content when offline */
self.addEventListener('fetch', function (e) {
  e.respondWith(
    caches.match(e.request).then(function (response) {
      return response || fetch(e.request);
    })
  );
});
