// Rail collapse toggles (F8 / DS §2.1.2 / §2.7).
//
// Two slim toggle buttons (one per rail, already in index.html) flip an
// `.is-collapsed` class on their rail. The grid uses `auto` side tracks so a
// collapsed rail (44px) reflows the stage automatically (see layout.css).
//
// State persists in its own tiny localStorage key — deliberately separate
// from main.js's `tiling-viz-prefs` so the two never entangle.
//
// Narrow-screen default: at ≤1279px, when there is no stored preference, the
// right rail starts collapsed (matchMedia, not a CSS display:none rule — so
// the toggle can always re-open it).

const RAILS_KEY = 'tiling-viz-rails';
const NARROW_MQ = '(max-width: 1279px)';

function loadRailPrefs() {
  try {
    const raw = localStorage.getItem(RAILS_KEY);
    if (!raw) return null;
    const p = JSON.parse(raw);
    const out = {};
    if (typeof p.left === 'boolean') out.left = p.left;
    if (typeof p.right === 'boolean') out.right = p.right;
    return Object.keys(out).length ? out : null;
  } catch {
    return null;
  }
}

function saveRailPrefs(state) {
  try {
    localStorage.setItem(RAILS_KEY, JSON.stringify(state));
  } catch {
    /* storage unavailable — non-fatal */
  }
}

export function initRails() {
  const leftRail = document.getElementById('left-rail');
  const rightRail = document.getElementById('right-rail');
  if (!leftRail && !rightRail) return;

  const stored = loadRailPrefs();
  const narrow =
    typeof window !== 'undefined' &&
    window.matchMedia &&
    window.matchMedia(NARROW_MQ).matches;

  // Resolve initial collapse state: stored preference wins; otherwise the
  // right rail starts collapsed on narrow screens.
  const collapsed = {
    left: stored ? !!stored.left : false,
    right: stored ? !!stored.right : narrow,
  };

  function apply(rail, isCollapsed) {
    if (!rail) return;
    rail.classList.toggle('is-collapsed', isCollapsed);
    const toggle = rail.querySelector('.rail-toggle');
    if (toggle) toggle.setAttribute('aria-expanded', isCollapsed ? 'false' : 'true');
  }

  apply(leftRail, collapsed.left);
  apply(rightRail, collapsed.right);

  function bind(rail, key) {
    if (!rail) return;
    const toggle = rail.querySelector('.rail-toggle');
    if (!toggle) return;
    toggle.addEventListener('click', () => {
      collapsed[key] = !collapsed[key];
      apply(rail, collapsed[key]);
      saveRailPrefs(collapsed);
    });
  }

  bind(leftRail, 'left');
  bind(rightRail, 'right');
}
