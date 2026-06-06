// Load + minimally validate data/manifest.json.
export async function loadManifest(url = 'data/manifest.json') {
  const res = await fetch(url, { cache: 'no-cache' });
  if (!res.ok) throw new Error(`manifest fetch failed: HTTP ${res.status}`);
  const m = await res.json();
  if (!Array.isArray(m.videos)) throw new Error('manifest: missing videos[]');
  for (const v of m.videos) {
    if (!v.id || !Array.isArray(v.variants)) throw new Error(`manifest: bad video entry ${v.id}`);
  }
  return m;
}

export async function loadRunFrames(url) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`frames fetch failed: HTTP ${res.status} ${url}`);
  return res.json();
}

export async function loadTrials(url) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`trials fetch failed: HTTP ${res.status} ${url}`);
  return res.json();
}
