# .nts/.ntf Boating Track Converter

Convert Navionics Boating app track files (`.nts` / `.ntf` pairs) into
`.kml` and `.gpx` for viewing in Google Earth, and batch-convert whole directories.

Written in pure Python 3 (standard library only — no dependencies).

## Background

The Navionics Boating app (now part of Garmin) stores tracks and routes as
binary files that are not publicly documented. This tool is the result of an
independent reverse-engineering effort, performed solely by analyzing the authors's own recorded files in a hex editor.
No Navionics or Garmin software, firmware, or documentation was used or decompiled.

**Key findings of the file format** (undocumented, may change between app
versions):

- Both files use Protocol Buffers (protobuf) wire format.
  - `.nts` (~2 KB): a single descriptor message holding start/end positions
    (field `3`), start/end timestamps (fields `4`/`5`, Unix seconds), timezone
    (IANA string, e.g. `Europe/Oslo`), per-leg records with optionally named
    waypoints (field `11`), and a double that appears to be the total distance
    in metres (field `7`).
  - `.ntf` (large): a stream of length-prefixed protobuf frames, one frame per
    recorded fix (~1 Hz). The first fix stores an **absolute** position; every
    following fix stores an **offset** from that anchor.
- Coordinates are **zigzag-encoded varints, scaled by 10⁶ degrees**
  (`degrees = zigzag(value) / 1_000_000`), stored as submessages
  `{field 1: lat, field 2: lon}`.
- Times are Unix epoch seconds; timezone offsets are seconds (e.g. `7200`).

## Installation

```bash
git clone <repository-url>
cd <repository-name>
```

Requires Python ≥ 3.9 (for `zoneinfo`; falls back to UTC otherwise).

## Usage

Convert a single track pair (the sidecar `.nts` is picked up automatically
for name/date/waypoint metadata):

```bash
python3 ntfconvert.py kml <TRACK-UUID>.ntf -o track.kml
python3 ntfconvert.py gpx <TRACK-UUID>.ntf -o track.gpx
```

Add `--animate` to include timestamps (spread over the duration reported by
the `.nts`), enabling Google Earth's time slider / GPX `<time>` elements.

Batch-convert a directory containing many `.ntf`/`.nts` pairs:

```bash
python3 ntfconvert.py batch ~/Boating -o converted --fmt both --report report.md
```

Options:

| option | effect |
|---|---|
| `--fmt kml\|gpx\|both` | output format for batch mode (default: both) |
| `--overwrite` | overwrite existing output files |
| `--report FILE` | write a markdown summary table (file, date, name, points, waypoints) |
| `--anchor LAT LON` | override the auto-detected absolute anchor position |
| `--scale FLOAT` | coordinate scale factor (default: `1e6`) |

Output files are named `<YYYY-MM-DD>_<HHMM>_<track-name>.kml/.gpx` using the
track's start time in its local timezone. Waypoints extracted from the
descriptor are included in KML output as named placemarks.

Diagnostic modes for exploring (new or changed) file variants:
`dump`, `fields`, `survey`, `series`, `track`, `scan` — see `--help`.

## How it works

The tool parses the protobuf wire format generically (no schema compiler
needed), then applies the empirically determined field semantics described
above. The anchor/offset scheme was validated by confirming that the decoded
track extents reproduce the start/end coordinates stored independently in the
`.nts` descriptor to within 2 units of 10⁻⁶ degrees.

## Limitations

- The format is undocumented and proprietary; it may change in future app
  versions. The diagnostic modes are provided to help adapt if it does.
- Per-fix timestamps in `.ntf` frames are not yet decoded; animated output
  interpolates between the start/end times from the `.nts`.
- User-given track names are not correctly detected.
- In a trial run on 62 tracks, 1 track could not be parsed completely. Therefore a 90% completeness threshold has been added to regard a file as successfully parsed. You might want to revert that. 

## Disclaimer

This is an unofficial community tool, not affiliated with or endorsed by
Navionics or Garmin. Use at your own risk. The author makes no claims about
the completeness or stability of the reverse-engineered format.

## Privacy note

`.ntf`/`.nts` files contain precise personal location history. Never commit
your own track files to a public repository. This tool reads them locally and
does not transmit any data.

## License

This project is licensed under the MIT License — see [LICENSE](LICENSE).

## Acknowledgements

The `.nts`/`.ntf` file format was reverse-engineered collaboratively by the
author and **Lumo**, Proton's privacy-first AI assistant
([lumo.proton.me](https://lumo.proton.me)), through iterative analysis of
hex dumps, protobuf wire-format decoding, and cross-validation between the
descriptor and track files.
