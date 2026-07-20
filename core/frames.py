"""rosbag2 (sqlite) reading, JPEG extraction, nearest-timestamp synchronization."""
import sqlite3, bisect

def jpeg_from_blob(blob):
    b = bytes(blob); s = b.find(b"\xff\xd8"); e = b.rfind(b"\xff\xd9")
    return b[s:e+2] if (s != -1 and e != -1) else None

def read_image_topics(db3):
    con = sqlite3.connect(db3); cur = con.cursor()
    out = {}
    for i, name, ttype in cur.execute("SELECT id,name,type FROM topics"):
        if name.endswith("/image_rgb/compressed"):
            out[name.split("/")[1]] = i
    con.close(); return out

def extract_synced(db3, want=None, tol_ms=25.0, step=1):
    con = sqlite3.connect(db3); cur = con.cursor()
    tid = read_image_topics(db3)
    cams = list(want) if want else list(tid)
    tl = {c: cur.execute("SELECT timestamp,id FROM messages WHERE topic_id=? ORDER BY timestamp",
                         (tid[c],)).fetchall() for c in cams}
    keys = {c: [r[0] for r in tl[c]] for c in cams}
    ref = min(cams, key=lambda c: len(tl[c])); tol = tol_ms*1e6
    def near(c, ts):
        ks = keys[c]; i = bisect.bisect_left(ks, ts); best = None
        for j in (i, i-1):
            if 0 <= j < len(ks):
                d = abs(ks[j]-ts)
                if d <= tol and (best is None or d < best[0]): best = (d, j)
        return None if best is None else tl[c][best[1]]
    def blob(mid): return cur.execute("SELECT data FROM messages WHERE id=?", (mid,)).fetchone()[0]
    out = []; k = 0
    for idx,(ts,_) in enumerate(tl[ref]):
        if idx % step: continue
        sel = {}; tss = {}; ok = True
        for c in cams:
            r = near(c, ts)
            if r is None: ok = False; break
            sel[c] = r[1]; tss[c] = r[0]
        if not ok: continue
        imgs = {c: jpeg_from_blob(blob(sel[c])) for c in cams}
        if any(v is None for v in imgs.values()): continue
        dts = list(tss.values())
        out.append(dict(setidx=k, images=imgs, tss=tss, max_dt_ms=(max(dts)-min(dts))/1e6)); k += 1
    con.close(); return out
