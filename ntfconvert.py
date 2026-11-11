#!/usr/bin/env python3
"""
ntfconvert.py

Converts Navionics Boating .nts/.ntf file pairs (protobuf wire format) into
KML/GPX for Google Earth. Validated format:

  Coordinates:  zigzag varints, degrees = zigzag(v) / 1e6,
                submessage {1: lat, 2: lon}.
  .nts  descriptor (one protobuf message):
        $.3.1/$.3.2   start/end absolute positions
        $.4/$.5       start/end time {unix_s, counter} + tz submessage
                      {offset_s, "Europe/Oslo", offset_s}
        $.11.x        leg records; $.11.11.x = named waypoints
                      {1: id, 2: name, 3: {1: lat, 2: lon}, 5: time...}
        $.7           LE double (likely total distance, m)
  .ntf  track (sequence of <varint len><frame> records):
        frame 1       start time + tz
        later frames  fix records; position under $.3.1 = {lat, lon}:
                      the FIRST fix is the ABSOLUTE anchor pair, all
                      following are offsets from that anchor.
                      Validation: offset minima reproduce the .nts
                      end-point to +/-2 units (1e-6 deg).

USAGE
  Single pair (auto-detects sidecar .nts for name/date/waypoints):
    ntfconvert.py kml  TRACK.ntf -o out.kml
    ntfconvert.py gpx  TRACK.ntf -o out.gpx
  Batch-convert a whole directory of pairs:
    ntfconvert.py batch ~/Boating -o OUTDIR [--fmt both]
    options: --fmt kml|gpx|both   --overwrite   --report FILE.md
  Diagnostics (rarely needed now):
    dump/fields FILE [FROM TO] | survey FILE | series FILE PATH [pairs]
    scan FILE [--bbox LA LO LA LO] | track FILE
"""

import argparse
import datetime
import os
import re
import sys
from collections import Counter, defaultdict


# ================================================================ wire format

def read_varint(buf, pos):
    result = shift = 0
    while True:
        if pos >= len(buf):
            raise IndexError("varint runs past end of buffer")
        b = buf[pos]
        pos += 1
        result |= (b & 0x7F) << shift
        if not b & 0x80:
            return result, pos
        shift += 7
        if shift > 70:
            raise ValueError("varint too long")


def zigzag(v):
    return (v >> 1) ^ -(v & 1)


class Msg:
    __slots__ = ("fields",)

    def __init__(self, fields):
        self.fields = fields


def parse_message(buf, pos, end, depth=0):
    if depth > 32:
        raise ValueError("too deeply nested")
    fields = []
    while pos < end:
        key, pos = read_varint(buf, pos)
        fnum, wt = key >> 3, key & 7
        if fnum == 0:
            raise ValueError("invalid field number 0")
        if wt == 0:
            val, pos = read_varint(buf, pos)
            fields.append((fnum, val))
        elif wt == 1:
            if pos + 8 > end:
                raise ValueError("fixed64 past end")
            fields.append((fnum, bytes(buf[pos:pos + 8])))
            pos += 8
        elif wt == 2:
            ln, pos = read_varint(buf, pos)
            if pos + ln > end:
                raise ValueError("chunk past end")
            try:
                sub, _ = parse_message(buf, pos, pos + ln, depth + 1)
            except (ValueError, IndexError):
                sub = bytes(buf[pos:pos + ln])
            fields.append((fnum, sub))
            pos += ln
        elif wt == 5:
            if pos + 4 > end:
                raise ValueError("fixed32 past end")
            fields.append((fnum, bytes(buf[pos:pos + 4])))
            pos += 4
        elif wt == 3:
            sub, pos = _parse_group(buf, pos, fnum, end, depth)
            fields.append((fnum, sub))
        elif wt == 4:
            return Msg(fields), pos
        else:
            raise ValueError("unknown wire type %d" % wt)
    return Msg(fields), pos


def _parse_group(buf, pos, fnum, end, depth):
    sub, pos = parse_message(buf, pos, end, depth + 1)
    key, pos = read_varint(buf, pos)
    if key & 7 != 4 or key >> 3 != fnum:
        raise ValueError("unbalanced group")
    return sub, pos


def parse_frames(data):
    """Sequence of <varint len><protobuf frame> (the .ntf layout)."""
    entries = []
    pos, n, covered = 0, len(data), 0
    while pos < n:
        try:
            ln, p = read_varint(data, pos)
        except (IndexError, ValueError):
            break
        if ln <= 0 or p + ln > n:
            break
        try:
            msg, end = parse_message(data, p, p + ln, 0)
        except (ValueError, IndexError):
            break
        if end != p + ln:
            break
        entries.append(("frame", p, (ln, msg)))
        covered = pos = p + ln
    return entries, covered >= 0.9 * n and len(entries) >= 5


MAX_RESYNC = 64


def parse_stream(data):
    """Single-message or best-effort resync parse (the .nts layout)."""
    try:
        msg, end = parse_message(data, 0, len(data), 0)
        if end == len(data):
            return [("msg", 0, msg)], True
    except (ValueError, IndexError):
        pass
    entries, pos, n = [], 0, len(data)
    while pos < n:
        best = None
        for off in range(0, MAX_RESYNC + 1):
            p = pos + off
            if p >= n:
                break
            try:
                msg, newpos = parse_message(data, p, n, 0)
            except (ValueError, IndexError):
                continue
            if newpos <= p:
                continue
            cand = (newpos - p, off, msg, newpos)
            if off == 0:
                best = cand
                break
            if best is None or cand[0] > best[0]:
                best = cand
        if best is None:
            entries.append(("gap", pos, n))
            break
        _s, off, msg, newpos = best
        if off:
            entries.append(("gap", pos, pos + off))
        entries.append(("msg", pos + off, msg))
        pos = newpos
    return entries, False


def load(path):
    with open(path, "rb") as fh:
        data = fh.read()
    entries, framed = parse_frames(data)
    if framed and len(entries) > 1:
        return data, entries, "frames"
    entries2, clean = parse_stream(data)
    return data, entries2, "single" if clean else "resync"


def iter_msgs(entries):
    for kind, _s, item in entries:
        if kind == "gap":
            continue
        yield item[1] if kind == "frame" else item


# ================================================================== traversal

def walk_tree(msg, path="$"):
    yield path, msg
    for fnum, val in msg.fields:
        if isinstance(val, Msg):
            yield from walk_tree(val, "%s.%d" % (path, fnum))


def varints_of(msg):
    return [zigzag(v) for _, v in msg.fields if isinstance(v, int)]


def strings_of(msg, path="$", out=None):
    """All UTF-8 human-readable byte fields with their paths."""
    if out is None:
        out = []
    for fnum, val in msg.fields:
        p = "%s.%d" % (path, fnum)
        if isinstance(val, bytes):
            try:
                s = val.decode("utf-8")
                if len(s) >= 2 and all(31 < ord(c) < 127 or c in "\n\r\t"
                                       for c in s):
                    out.append((p, s))
            except UnicodeDecodeError:
                pass
        elif isinstance(val, Msg):
            strings_of(val, p, out)
    return out


def find_times(msg):
    """Unix-range varints anywhere in the tree, in encounter order."""
    times = []
    for _p, m in walk_tree(msg):
        for _f, v in m.fields:
            if isinstance(v, int) and 1_500_000_000 < v < 1_900_000_000:
                times.append(v)
    return times


def find_abs_pairs(msg, scale):
    """Submessages that hold a plausible absolute {1: lat, 2: lon} pair."""
    pairs = []
    for p, m in walk_tree(msg):
        vs = varints_of(m)
        if len(vs) >= 2:
            lat, lon = vs[0], vs[1]
            if 1_000_000 < abs(lat) <= 90_000_000 * scale \
                    and 1_000_000 < abs(lon) <= 180_000_000 * scale:
                pairs.append((p, lat / scale, lon / scale))
    return pairs


def find_waypoints(desc_msg, scale):
    """Named waypoints: submessages with a name string (field 2) and a
    coordinate submessage (field 3) holding an absolute pair."""
    wps = []
    for p, m in walk_tree(desc_msg):
        name = None
        coord = None
        for fnum, val in m.fields:
            if fnum == 2 and isinstance(val, bytes):
                try:
                    s = val.decode("utf-8")
                    if len(s) >= 2:
                        name = s
                except UnicodeDecodeError:
                    pass
            elif fnum == 3 and isinstance(val, Msg):
                vs = varints_of(val)
                if len(vs) >= 2:
                    lat, lon = vs[0], vs[1]
                    if (1_000_000 < abs(lat) <= 90_000_000 * scale
                            and 1_000_000 < abs(lon) <= 180_000_000 * scale):
                        coord = (lat / scale, lon / scale)
        if name and coord:
            wps.append((name, coord))
    return wps


# ======================================================== track reconstruction

ABS_LIMIT = 5_000_000      # offsets beyond this magnitude = absolute pair


def decode_track(entries, anchor=None, scale=1e6):
    """Rebuild the track. The anchor is auto-detected: the unique
    two-varint submessage holding a real degree-range pair (values at
    the 1e6 scale). All other pairs at that same path are offsets."""
    msgs = list(iter_msgs(entries))

    def is_abs(v0, v1):
        return (1_000_000 < abs(v0) <= 90_000_000
                and 1_000_000 < abs(v1) <= 180_000_000)

    anc_lat = anc_lon = None
    anchor_path = None
    if anchor:
        anc_lat, anc_lon = anchor
        # fall back to the most common two-varint path
        freq = Counter()
        for msg in msgs:
            for p, m in walk_tree(msg):
                if len(varints_of(m)) >= 2:
                    freq[p] += 1
        if freq:
            anchor_path = freq.most_common(1)[0][0]
    else:
        for msg in msgs:
            for p, m in walk_tree(msg):
                vs = varints_of(m)
                if len(vs) >= 2 and is_abs(vs[0], vs[1]):
                    if anchor_path is None:
                        anchor_path = p
                    if anc_lat is None:
                        anc_lat, anc_lon = vs[0] / scale, vs[1] / scale

    if anchor_path is None or anc_lat is None:
        return [], (None, None)

    points = []
    for msg in msgs:
        for p, m in walk_tree(msg):
            if p != anchor_path:
                continue
            vs = varints_of(m)
            if len(vs) < 2:
                continue
            v0, v1 = vs[0], vs[1]
            if is_abs(v0, v1):
                anc_lat, anc_lon = v0 / scale, v1 / scale
                points.append((anc_lat, anc_lon))
            elif anc_lat is not None:
                lat = anc_lat + v0 / scale
                lon = anc_lon + v1 / scale
                if -90 <= lat <= 90 and -180 <= lon <= 180:
                    points.append((lat, lon))
    return points, (anc_lat, anc_lon)


# ============================================================== .nts metadata

class Descriptor:
    def __init__(self):
        self.name = None
        self.tz = None
        self.start_ts = None
        self.end_ts = None
        self.start_pt = None
        self.end_pt = None
        self.waypoints = []
        self.candidate_names = []


def parse_descriptor(path, scale):
    d = Descriptor()
    if not path or not os.path.exists(path):
        return d
    _data, entries, layout = load(path)
    if layout == "frames" or not entries:
        return d
    msg = entries[0][2]
    times = find_times(msg)
    if times:
        d.start_ts, d.end_ts = times[0], times[-1]
    strs = strings_of(msg)
    abs_pairs = find_abs_pairs(msg, scale)
    for _p, s in strs:
        if re.match(r"^[A-Za-z]+(/[A-Za-z0-9_+\-]+)+$", s):
            d.tz = s
    d.waypoints = find_waypoints(msg, scale)
    wp_names = {w[0] for w in d.waypoints}
    for p, s in strs:
        if s == d.tz or s in wp_names:
            continue
        d.candidate_names.append((p, s))
    # heuristic: a top-level-ish string (short path, not under $.11 legs)
    top = [(p, s) for p, s in d.candidate_names
           if p.count(".") <= 2 and not p.startswith("$.11")]
    if top:
        d.name = top[0][1]
    elif d.candidate_names:
        d.name = d.candidate_names[0][1]
    # start/end points from $.3.1/$.3.2 if present
    for p, lat, lon in abs_pairs:
        if p.startswith("$.3.1"):
            d.start_pt = (lat, lon)
        elif p.startswith("$.3.2"):
            d.end_pt = (lat, lon)
    return d


def safe_name(s):
    s = re.sub(r"[^\w\s-]", "", s, flags=re.UNICODE).strip()
    return re.sub(r"[\s]+", "_", s) or "track"


def local_tz(name):
    try:
        from zoneinfo import ZoneInfo
        return ZoneInfo(name)
    except Exception:
        return datetime.timezone.utc


# =================================================================== exports

def iso_utc(ts):
    return datetime.datetime.fromtimestamp(
        ts, datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def synthesize_times(points, start_ts, end_ts):
    n = len(points)
    if not n or not start_ts:
        return [(None, la, lo) for la, lo in points]
    if not end_ts or end_ts <= start_ts:
        end_ts = start_ts + n
    step = (end_ts - start_ts) / max(n - 1, 1)
    return [(start_ts + i * step, la, lo) for i, (la, lo) in enumerate(points)]


def kml_escape(s):
    return (s.replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;"))


def write_kml(points, name, waypoints=None, start_end=None):
    timed = [p for p in points if p[0] is not None]
    out = ['<?xml version="1.0" encoding="UTF-8"?>',
           '<kml xmlns="http://www.opengis.net/kml/2.2" '
           'xmlns:gx="http://www.google.com/kml/ext/2.2">',
           "<Document>\n  <name>%s</name>" % kml_escape(name)]
    if waypoints:
        for wname, (lat, lon) in waypoints:
            out.append(
                "  <Placemark>\n    <name>%s</name>\n"
                "    <Style><IconStyle><scale>1.2</scale></IconStyle>"
                "</Style>\n"
                '    <Point><coordinates>%.6f,%.6f,0</coordinates></Point>\n'
                "  </Placemark>"
                % (kml_escape(wname), lon, lat))
    if start_end:
        for label, (lat, lon) in start_end:
            out.append(
                "  <Placemark>\n    <name>%s</name>\n"
                "    <Style><IconStyle><scale>1.4</scale></IconStyle>"
                "</Style>\n"
                '    <Point><coordinates>%.6f,%.6f,0</coordinates></Point>\n'
                "  </Placemark>"
                % (kml_escape(label), lon, lat))
    out += ["  <Placemark>\n    <name>%s</name>" % kml_escape(name),
            "    <Style><LineStyle><width>3</width></LineStyle></Style>"]
    if len(timed) == len(points) and len(points) > 1:
        out.append("    <gx:Track>")
        for ts, _la, _lo in points:
            out.append("      <when>%s</when>" % iso_utc(ts))
        for _ts, lat, lon in points:
            out.append("      <gx:coord>%.6f %.6f 0</gx:coord>" % (lon, lat))
        out.append("    </gx:Track>")
    else:
        coords = " ".join("%.6f,%.6f,0" % (lon, lat)
                          for _t, lat, lon in points)
        out.append(
            "    <LineString><tessellate>1</tessellate>\n"
            "      <coordinates>%s</coordinates>\n"
            "    </LineString>" % coords)
    out += ["  </Placemark>", "</Document>\n</kml>"]
    return "\n".join(out) + "\n"


def write_gpx(points, name):
    out = ['<?xml version="1.0" encoding="UTF-8"?>',
           '<gpx version="1.1" creator="ntfconvert.py" '
           'xmlns="http://www.topografix.com/GPX/1/1">',
           "  <trk><name>%s</name><trkseg>" % kml_escape(name)]
    for ts, lat, lon in points:
        t = "<time>%s</time>" % iso_utc(ts) if ts else ""
        out.append('    <trkpt lat="%.6f" lon="%.6f">%s</trkpt>'
                   % (lat, lon, t))
    out += ["  </trkseg></trk>", "</gpx>"]
    return "\n".join(out) + "\n"


# =================================================================== modes

def mode_report_files(entries):
    shapes = Counter()
    for msg in iter_msgs(entries):
        shapes[tuple(sorted({f for f, _ in msg.fields}))] += 1
    for shape, cnt in shapes.most_common():
        print("   %-20s x%d" % (" ".join(map(str, shape)), cnt))


def dump_msg(msg, indent=0, path="$"):
    pad = "  " * indent
    for fnum, val in msg.fields:
        p = "%s.%d" % (path, fnum)
        if isinstance(val, Msg):
            if not val.fields:
                print("%s%s: {}" % (pad, p))
            else:
                print("%s%s: message" % (pad, p))
                dump_msg(val, indent + 1, p)
        elif isinstance(val, bytes):
            try:
                s = val.decode("utf-8")
                printable = all(31 < ord(c) < 127 or c in "\n\r\t"
                                for c in s)
            except UnicodeDecodeError:
                s, printable = val.hex(), False
            print("%s%s: %s" % (pad, p, '"%s"' % s if printable else s))
        else:
            print("%s%s: %d" % (pad, p, val))


def scan_pairs(data, scale_k):
    hits = []
    i, n = 0, len(data)
    lat_lim, lon_lim = 90_000_000 * scale_k, 180_000_000 * scale_k
    while i < n - 1:
        if data[i] == 0x08:
            try:
                v1, j = read_varint(data, i + 1)
                if j < n and data[j] == 0x10:
                    v2, k = read_varint(data, j + 1)
                    z1, z2 = zigzag(v1), zigzag(v2)
                    if abs(z1) <= lat_lim and abs(z2) <= lon_lim:
                        hits.append((i, z1 / scale_k, z2 / scale_k))
                        i = k
                        continue
            except (IndexError, ValueError):
                pass
        i += 1
    return hits


# ---------------------------------------------------------------- conversion

def convert_pair(ntf_path, nts_path, fmt, animate, anchor_arg, scale):
    """Returns (points, desc) or raises."""
    _d, entries, layout = load(ntf_path)
    if layout != "frames":
        raise ValueError("not a framed .ntf track file")
    desc = parse_descriptor(nts_path, scale)
    points, anc = decode_track(entries, anchor_arg, scale)
    if len(points) < 2:
        raise ValueError("fewer than 2 track points decoded")
    if animate:
        points = synthesize_times(points, desc.start_ts, desc.end_ts)
    else:
        points = [(None, la, lo) for la, lo in points]
    return points, desc


def output_basename(desc, ntf_path):
    datestr = "unknown-date"
    if desc.start_ts:
        tz = local_tz(desc.tz) if desc.tz else datetime.timezone.utc
        datestr = datetime.datetime.fromtimestamp(
            desc.start_ts, tz).strftime("%Y-%m-%d_%H%M")
    base = safe_name(desc.name) if desc.name else "track"
    return "%s_%s" % (datestr, base)


def cmd_batch(args):
    src = args.paths[0]
    scale_k = args.scale
    if not os.path.isdir(src):
        sys.exit("not a directory: %s" % src)
    outdir = args.output or os.path.join(src, "converted")
    os.makedirs(outdir, exist_ok=True)
    ntf_files = sorted(f for f in os.listdir(src) if f.lower().endswith(".ntf"))
    if not ntf_files:
        sys.exit("no .ntf files found in %s" % src)
    rows = []
    report_lines = ["# Navionics batch conversion\n",
                    "| file | date | name | points | waypoints | output |",
                    "|---|---|---|---|---|---|"]
    ok = fail = 0
    for ntf in ntf_files:
        stem = os.path.splitext(ntf)[0]
        ntf_path = os.path.join(src, ntf)
        nts_path = os.path.join(src, stem + ".nts")
        try:
            points, desc = convert_pair(ntf_path, nts_path, args.fmt,
                                        args.animate, args.anchor, scale_k)
        except Exception as exc:
            rows.append((ntf, "-", "-", 0, 0, "FAILED: %s" % exc))
            fail += 1
            continue
        base = output_basename(desc, ntf_path)
        exts = {"kml": ["kml"], "gpx": ["gpx"], "both": ["kml", "gpx"]}[args.fmt]
        written = []
        for ext in exts:
            out_path = os.path.join(outdir, "%s.%s" % (base, ext))
            if os.path.exists(out_path) and not args.overwrite:
                written.append(os.path.basename(out_path) + " (exists, skipped)")
                continue
            se = None
            if desc.start_pt and desc.end_pt:
                se = [("Start", desc.start_pt), ("End", desc.end_pt)]
            if ext == "kml":
                text = write_kml(points, desc.name or base,
                                 desc.waypoints or None, se)
            else:
                text = write_gpx(points, desc.name or base)
            with open(out_path, "w", encoding="utf-8") as fh:
                fh.write(text)
            written.append(os.path.basename(out_path))
        name_str = desc.name or "(no name found)"
        name_str = name_str.replace("\r", " ").replace("\n", " ").strip()
        datestr = "unknown"
        if desc.start_ts:
            tz = local_tz(desc.tz) if desc.tz else datetime.timezone.utc
            datestr = datetime.datetime.fromtimestamp(
                desc.start_ts, tz).strftime("%Y-%m-%d %H:%M")
        rows.append((stem[:24], datestr, name_str, len(points),
                     len(desc.waypoints), ", ".join(written)))
        report_lines.append("| %s | %s | %s | %d | %d | %s |"
                            % (stem, datestr, name_str, len(points),
                               len(desc.waypoints), ", ".join(written)))
        if desc.candidate_names:
            report_lines.append("  - name candidates in %s: %s"
                                % (stem, desc.candidate_names[:6]))
        ok += 1
    # summary
    print("%-26s %-17s %-28s %6s %5s" %
          ("file", "start (local)", "name", "pts", "wpts"))
    print("-" * 92)
    for r in rows:
        if len(r) == 6 and isinstance(r[3], int) and r[3] > 0:
            print("%-26s %-17s %-28s %6d %5d  -> %s"
                  % (r[0], r[1], r[2][:28], r[3], r[4], r[5]))
        else:
            print("%-26s %s" % (r[0], r[5] if len(r) == 6 else r))
    print("-" * 92)
    print("%d converted, %d failed -> %s" % (ok, fail, outdir))
    if args.report:
        with open(args.report, "w", encoding="utf-8") as fh:
            fh.write("\n".join(report_lines) + "\n")
        print("report written to %s" % args.report)


def cmd_convert(args, mode):
    ntf_path = args.paths[0]
    stem = os.path.splitext(ntf_path)[0]
    nts_path = None
    for cand in (stem + ".nts",):
        if os.path.exists(cand):
            nts_path = cand
    points, desc = convert_pair(ntf_path, nts_path, mode, args.animate,
                                args.anchor, args.scale)
    if desc.start_pt and desc.end_pt:
        se = [("Start", desc.start_pt), ("End", desc.end_pt)]
    else:
        se = None
    name = desc.name or os.path.basename(stem)
    if mode == "kml":
        text = write_kml(points, name, desc.waypoints or None, se)
    else:
        text = write_gpx(points, name)
    out = args.output or (os.path.basename(output_basename(desc, ntf_path))
                          + "." + mode)
    with open(out, "w", encoding="utf-8") as fh:
        fh.write(text)
    print("wrote %d points (+%d waypoints) to %s"
          % (len(points), len(desc.waypoints), out))


# ====================================================================== main

def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["kml", "gpx", "batch", "dump", "fields",
                                     "survey", "series", "track", "scan"])
    ap.add_argument("paths", nargs="+")
    ap.add_argument("-o", "--output")
    ap.add_argument("--fmt", choices=["kml", "gpx", "both"], default="both")
    ap.add_argument("--overwrite", action="store_true")
    ap.add_argument("--report", help="batch: write markdown summary report")
    ap.add_argument("--animate", action="store_true",
                    help="add timestamps spread over the .nts-reported "
                         "duration (KML time slider / GPX <time>)")
    ap.add_argument("--anchor", nargs=2, type=float, metavar=("LAT", "LON"),
                    help="override auto-detected anchor position")
    ap.add_argument("--scale", type=float, default=1e6)
    ap.add_argument("--bbox", nargs=4, type=float,
                    metavar=("LAT", "LON", "LAT", "LON"))
    args = ap.parse_args()

    try:
        if args.mode == "batch":
            cmd_batch(args)
        elif args.mode in ("kml", "gpx"):
            cmd_convert(args, args.mode)
        elif args.mode in ("dump", "fields"):
            rng = None
            if len(args.paths) >= 3:
                rng = (int(args.paths[1]), int(args.paths[2]))
            for path in args.paths[:1]:
                _d, entries, layout = load(path)
                print("layout: %s, %d records" % (layout, len(entries)))
                for idx, msg in enumerate(iter_msgs(entries), 1):
                    if rng and not (rng[0] <= idx <= rng[1]):
                        continue
                    print("\n--- record #%d ---" % idx)
                    if args.mode == "dump":
                        dump_msg(msg)
                    else:
                        for p, m in walk_tree(msg):
                            vs = varints_of(m)
                            if len(vs) >= 2:
                                print("  %-16s zigzag: %s"
                                      % (p, ", ".join("%.0f" % v
                                                      for v in vs)))
        elif args.mode == "survey":
            _d, entries, _l = load(args.paths[0])
            shapes = Counter()
            fieldvals = defaultdict(list)
            for msg in iter_msgs(entries):
                shapes[tuple(sorted({f for f, _ in msg.fields}))] += 1
                for p, m in walk_tree(msg):
                    vs = varints_of(m)
                    if vs:
                        fieldvals[p].append(vs[0])
            for p, seqs in sorted(fieldvals.items()):
                print("%-14s n=%-5d min=%-12.0f max=%-12.0f first5=%s"
                      % (p, len(seqs), min(seqs), max(seqs),
                         [int(x) for x in seqs[:5]]))
        elif args.mode == "series":
            path, tgt = args.paths[0], (args.paths[1] if len(args.paths) > 1
                                        else "$.3.1")
            _d, entries, _l = load(path)
            idx_out = 0
            for i, msg in enumerate(iter_msgs(entries), 1):
                for p, m in walk_tree(msg):
                    if p == tgt:
                        vs = varints_of(m)
                        print("%5d  %s" % (i, " ".join("%.0f" % v
                                                       for v in vs)))
                        idx_out += 1
        elif args.mode == "track":
            _d, entries, layout = load(args.paths[0])
            points, anc = decode_track(entries, args.anchor, args.scale)
            print("%d fixes, anchor %s" % (len(points), anc))
            for i, (lat, lon) in enumerate(points[:args_limit()]):
                print("%5d  %12.6f  %12.6f" % (i, lat, lon))
        elif args.mode == "scan":
            for path in args.paths:
                with open(path, "rb") as fh:
                    data = fh.read()
                hits = scan_pairs(data, args.scale / 1e6)
                print("%s: %d candidates" % (path, len(hits)))
                for h in hits[:20]:
                    print("  @0x%X  %.6f %.6f" % h)
    except BrokenPipeError:
        pass


def args_limit():
    return 30


if __name__ == "__main__":
    main()
