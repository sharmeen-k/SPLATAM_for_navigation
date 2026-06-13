# assets/

Media for the project site (`/index.html`). Reference files from the HTML
with a path **relative to the repo root**, e.g.:

```html
<img src="assets/seq4_compare.png" alt="seq4 render vs ground truth" />
<video controls src="assets/demo.mp4"></video>
```

## What goes here
- Result images / renders (PNG or JPG).
- Pipeline diagram (PNG/SVG).
- The demo video, **if** it's small enough (see size note below).

## Naming
Lowercase, no spaces, descriptive: `seq4_compare.png`, `seq1_drift.png`,
`pipeline_diagram.svg`, `office0_render.jpg`.

## Size / video note
GitHub blocks files **> 100 MB** and warns over 50 MB, and Pages sites have a
soft ~1 GB total limit. So:
- **Images:** fine here. Keep them web-sized (resize to ~1600px wide, compress).
- **Video:** if it's more than a few tens of MB, **don't commit it** — upload to
  YouTube and embed instead of using a local `assets/demo.mp4`:
  ```html
  <iframe src="https://www.youtube.com/embed/VIDEO_ID"
          style="width:100%;height:100%;border:0" allowfullscreen></iframe>
  ```
  Only drop an `.mp4` in here if it's small (a short, compressed clip).
