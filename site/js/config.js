// tangent: the website's settings, in one place.
//
// Colours are not here: they are the CSS tokens at the top of
// site/css/site.css, which the stage reads, so the page and the part change
// together.
export default {
  // The part, as written by tools/site_model.cpp. Every file the pages load
  // ends in ?v=dev, which the Pages workflow swaps for the commit, so a
  // deploy reaches phones at once instead of after GitHub's ten-minute cache.
  model: 'site/model/part.bin?v=dev',
  modelInfo: 'site/model/part.json?v=dev',

  // A video to play behind the hero instead of the live part: a path such as
  // 'site/media/hero.mp4', or null for the live part. A video skips WebGL
  // entirely, so the scroll choreography below does not apply to it.
  heroVideo: null,

  // Radians per second the part turns by itself. 0 holds it still.
  spin: 0.16,

  // Draw the dimension callouts beside the part.
  callouts: true,

  // A glow over the brightest parts of the picture. 0 leaves it off, which
  // is the look the site is designed for; it costs a pass per frame.
  bloom: 0,
};
