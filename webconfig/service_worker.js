//
// Service worker for WARBL configuration tool offline use resource caching
//
//
// Updated 4/21/26
//

//
// Service worker for WARBL configuration tool offline use resource caching
//

const cacheName = "warbl_config_78";

const contentToCache = [
  "/configure.html",
  "/favicon.ico",
  "/manifest.json",

  "/css/config.min.css",
  "/css/nouislider.css",

  "/js/jquery-3.3.1.min.js",
  "/js/nouislider.js",
  "/js/constants.min.js",
  "/js/midi.min.js",
  "/js/d3.min.js",

  "/img/small_logo.png",
  "/img/downarrow.png",
  "/img/info-circle.svg",
  "/img/volume-off.svg",
  "/img/volume-up.svg",
  "/img/angle-up.svg",
  "/img/angle-down.svg",
  "/img/bars.svg",
  "/img/warbl-app-36x36.png",
  "/img/warbl-app-48x48.png",
  "/img/warbl-app-72x72.png",
  "/img/warbl-app-96x96.png",
  "/img/warbl-app-144x144.png",
  "/img/warbl-app-192x192.png",

  "/soundfonts/WARBL_SoundFonts2.sf2",

  "/spessasynth/spessasynth_lib_4.2.0_index.js",
  "/spessasynth/spessasynth_core_4.2.0_index.js",
  "/spessasynth/spessasynth_processor.min.js"
];

// Only these pages should be treated as app pages
const allowedPages = new Set([
  "/configure.html"
]);

// Only these folders/resources should be cached/served by this SW
const allowedPrefixes = [
  "/js/",
  "/img/",
  "/soundfonts/",
  "/spessasynth/"
];

const allowedExactFiles = new Set([
  "/favicon.ico",
  "/manifest.json",
    // Only these CSS files
  "/css/config.min.css",
  "/css/nouislider.css",
]);

function shouldHandleRequest(request) {
  if (request.method !== "GET") return false;

  const url = new URL(request.url);

  // Only same-origin requests
  if (url.origin !== self.location.origin) return false;

  const path = url.pathname;

  if (allowedPages.has(path)) return true;
  if (allowedExactFiles.has(path)) return true;

  return allowedPrefixes.some(prefix => path.startsWith(prefix));
}

// Install: precache the config tool shell
self.addEventListener("install", (event) => {
  self.skipWaiting();

  event.waitUntil((async () => {
    const cache = await caches.open(cacheName);
    await cache.addAll(contentToCache);
  })());
});

// Activate: claim clients and remove older config-tool caches
self.addEventListener("activate", (event) => {
  event.waitUntil((async () => {
    await clients.claim();

    const keys = await caches.keys();
    await Promise.all(
      keys
        .filter((key) => key !== cacheName && key.startsWith("warbl_config_"))
        .map((key) => caches.delete(key))
    );
  })());
});

// Fetch: only handle configure tool pages and their resources
self.addEventListener("fetch", (event) => {
  const req = event.request;

  if (!shouldHandleRequest(req)) {
    return;
  }

  event.respondWith((async () => {
    const cached = await caches.match(req, {
      ignoreSearch: false,
      ignoreVary: true
    });

    if (cached) {
      return cached;
    }

    try {
      const response = await fetch(req);

      if (response && response.ok && response.type === "basic") {
        const cache = await caches.open(cacheName);
        await cache.put(req, response.clone());
      }

      return response;
    } catch (error) {
      const url = new URL(req.url);

      // Offline fallback for the two app pages
      if (req.mode === "navigate" && allowedPages.has(url.pathname)) {
        const fallback = await caches.match("/configure.html");
        if (fallback) return fallback;
      }

      throw error;
    }
  })());
});