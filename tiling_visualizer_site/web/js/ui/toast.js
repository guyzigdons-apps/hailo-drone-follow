// Bottom-center transient toast (DS §4.9). Auto-dismiss after 4s unless
// sticky; sticky/error toasts carry a manual Dismiss button.
export function showToast(msg, { kind = 'info', sticky = false } = {}) {
  const root = document.getElementById('toast-root');
  if (!root) return () => {};

  // Cap simultaneous toasts to avoid flooding the UI.
  while (root.children.length >= 4) root.removeChild(root.firstChild);

  const el = document.createElement('div');
  el.className = 'toast' + (kind === 'error' ? ' toast--error' : '');

  const text = document.createElement('span');
  text.className = 'toast__msg';
  text.textContent = msg;
  el.appendChild(text);

  let timer = null;
  const remove = () => {
    if (timer) clearTimeout(timer);
    el.remove();
  };

  if (sticky || kind === 'error') {
    const btn = document.createElement('button');
    btn.className = 'btn toast__dismiss';
    btn.type = 'button';
    btn.textContent = 'Dismiss';
    btn.addEventListener('click', remove);
    el.appendChild(btn);
  } else {
    timer = setTimeout(remove, 4000);
  }

  root.appendChild(el);
  return remove;
}
