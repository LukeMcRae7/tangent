// tangent: the page: nav state, entrances, the copy button, and the stage.
import config from './config.js';

const nav = document.getElementById('nav');
// Only the links to this page's own sections follow the scroll: the nav's on
// the home page, the contents list's on the about page.
const links = [...document.querySelectorAll('.nav-links a[href^="#"], .toc a')];
const sections = links.map(a => document.querySelector(a.getAttribute('href')));

function onScroll() {
  nav.classList.toggle('scrolled', scrollY > 12);
  // The section whose top has passed the middle of the screen is the one we are in.
  let current = -1;
  sections.forEach((s, i) => { if (s && s.getBoundingClientRect().top < innerHeight * 0.5) current = i; });
  links.forEach((a, i) => a.classList.toggle('active', i === current));
}
addEventListener('scroll', onScroll, { passive: true });
onScroll();

// Things fade up as they arrive, once.
const reveal = new IntersectionObserver(entries => {
  for (const e of entries) if (e.isIntersecting) { e.target.classList.add('in'); reveal.unobserve(e.target); }
}, { rootMargin: '0px 0px -10% 0px' });
document.querySelectorAll('.section-head, .more, .download-card, .footer-mark, .doc-section, .page-head')
  .forEach(el => { el.classList.add('reveal'); reveal.observe(el); });

document.querySelectorAll('[data-copy]').forEach(btn => {
  btn.addEventListener('click', async () => {
    const text = document.getElementById(btn.dataset.copy).innerText;
    try {
      await navigator.clipboard.writeText(text);
      btn.classList.add('done');
      btn.querySelector('span').textContent = 'Copied';
      setTimeout(() => { btn.classList.remove('done'); btn.querySelector('span').textContent = 'Copy'; }, 1600);
    } catch { /* no clipboard: the text is still there to select */ }
  });
});

// The part's own numbers, from what the kernel wrote.
if (document.getElementById('readout')) fetch(config.modelInfo).then(r => r.json()).then(info => {
  const set = (k, v) => { const el = document.querySelector(`[data-part="${k}"]`); if (el) el.textContent = v; };
  set('faces', `${info.faces} faces`);
  set('volume', `${(info.volume_mm3 / 1000).toFixed(2)} cm³`);
  set('size', info.size_mm.map(v => +v.toFixed(1)).join(' × ') + ' mm');
  const ok = document.querySelector('.readout .ok');
  if (ok && !info.solid) { ok.lastChild.textContent = 'not solid'; ok.style.color = 'var(--warn, #E0A13A)'; }
}).catch(() => {});

const stage = document.getElementById('stage');
if (!stage) {
  // A page without the part.
} else if (config.heroVideo) {
  const v = Object.assign(document.createElement('video'), {
    src: config.heroVideo, autoplay: true, muted: true, loop: true, playsInline: true,
  });
  stage.append(v);
  stage.classList.add('ready');
} else {
  import('./stage.js').then(m => m.start(stage, config))
    .catch(err => { console.warn('stage:', err); stage.classList.add('no-webgl'); fadeWithPage(stage); });
}

// Without WebGL the still stands in for the part, and leaves the page where
// the part would: fading out as the rest of the bench comes up, on the same
// curve stage.js uses, so the sections below stand on plain ground.
function fadeWithPage(el) {
  const more = document.querySelector('.more');
  if (!more) return;
  const smooth = x => { x = Math.min(1, Math.max(0, x)); return x * x * (3 - 2 * x); };
  const fade = () => {
    const top = more.getBoundingClientRect().top;
    el.style.opacity = smooth((top - innerHeight * 0.3) / (innerHeight * 0.45));
  };
  addEventListener('scroll', fade, { passive: true });
  addEventListener('resize', fade);
  fade();
}
