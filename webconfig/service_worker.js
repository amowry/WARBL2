//
// Service worker for WARBL configuration tool offline use resource caching
//
//
//
// Updated 19 September 2024 at 0810
//
//
//

const cacheName = "warbl_4";

const contentToCache = [
    "https://warbl.xyz/configure.html",
    "https://warbl.xyz/favicon.ico",
    "https://warbl.xyz/manifest.json",
    "https://warbl.xyz/css/config.min.css",
    "https://warbl.xyz/css/nouislider.css",
    "https://warbl.xyz/css/modal.css", 
    "https://warbl.xyz/js/jquery-3.3.1.min.js",
    "https://warbl.xyz/js/nouislider.js",
    "https://warbl.xyz/js/WebAudioFontPlayer.js",
    "https://warbl.xyz/js/0650_SBLive_sf2.js",
    "https://warbl.xyz/js/constants.min.js",
    "https://warbl.xyz/js/midi.min.js",
    "https://warbl.xyz/js/d3.min.js",
    "https://warbl.xyz/js/daypilot-modal.min-3.10.1.js",
    "https://warbl.xyz/js/update.js",
    "https://warbl.xyz/img/small_logo.png",
    "https://warbl.xyz/img/downarrow.png",
    "https://warbl.xyz/img/info-circle.svg",
    "https://warbl.xyz/img/volume-off.svg",
    "https://warbl.xyz/img/volume-up.svg",
    "https://warbl.xyz/img/angle-up.svg",
    "https://warbl.xyz/img/angle-down.svg"
];

// Installing Service Worker
self.addEventListener("install", (e) => {

    console.log("[Service Worker] Install");

    // Make this the current service worker
    self.skipWaiting();
    
    e.waitUntil((async () => {
      const cache = await caches.open(cacheName);
      console.log("[Service Worker] Caching all: app shell and content");
      await cache.addAll(contentToCache);
      console.log("[Service Worker] Cache addAll complete!");
    })());


  });

self.addEventListener("activate", event => {

    console.log("[Service Worker] Activate event");

    clients.claim().then(() => {
        //claim means that the html file will use this new service worker.
        console.log(
          "[Service Worker] - The service worker has now claimed all pages so they use the new service worker."
        );
    });

   event.waitUntil(
        caches.keys().then((keys) => {
          return Promise.all(
            keys.filter((key) => key != cacheName).map((key) => {if (key.indexOf("warbl") != -1){caches.delete(key)}})
          );
        })
    );

});
  
// Fetching content using Service Worker
self.addEventListener("fetch", (e) => {

    console.log(`[Service Worker] fetching: ${e.request.url}`);

    // Cache http and https only, skip unsupported chrome-extension:// and file://...
    if (!(
        e.request.url.startsWith("http:") || e.request.url.startsWith("https:")
    )) {
        return; 
    }

    e.respondWith((async () => {

        const r = await caches.match(e.request,{ignoreSearch: true});

        if (r){

            console.log(`[Service Worker] Returning cached resource: ${e.request.url}`);

            return r;
        }

        try{

            console.log(`[Service Worker] Not in cache, fetching resource: ${e.request.url}`);

            const response = await fetch(e.request);
 
            return response;
            
        }
        catch (error){

            console.log("[Service Worker] fetch error: "+error);
    
        }
    })());
});


