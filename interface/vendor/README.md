# interface/vendor/

Local copies of the JS libraries the GUI depends on. Vendored so the
browser at `http://localhost:5002` works **fully offline** — no internet,
no LAN, no DNS required.

| File | Source | License |
|---|---|---|
| `three.min.js` | https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js | MIT |
| `roslib.min.js` | https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js | BSD-3-Clause |

## Why these are committed

Earlier versions of `index.html` loaded three.js and roslib.js (and a
Google Fonts CSS) directly from public CDNs. On a laptop with a LAN cable
plugged into a captive-portal / no-internet network, those fetches stalled
on TCP timeout and the page hung forever on `Loading point cloud…`. WiFi
worked because WiFi had real internet.

By vendoring the libraries, the GUI is independent of network state.

## Refreshing

If you need to update either library:

```bash
curl -sSL -o interface/vendor/three.min.js  https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js
curl -sSL -o interface/vendor/roslib.min.js https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js
```

Note: `index.html` is pinned to **three.js r128**. Newer major versions
(r150+) moved a lot of the API into ES modules and will break the current
`<script>`-based imports — bump the version only if you also rewrite the
3D code.
