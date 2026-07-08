# Map Layers — Incremental Projection Architecture

Target architecture for **efficient, near-real-time, multi-layer** spatial maps (coverage,
motor-current heatmap, future sensors). Replaces the naive full-grid `coverage_feedback` raster.

## 0. The one idea
**Source of truth ≠ live map.**
- **Source** = recorded **track + sensor streams** (position-history + ESC current + …). Every layer
  is a pure function of this → fully **post-hoc recomputable**. Never on the hot path.
- **Live map** = a thin **incremental projection**: sparse-tiled, **delta-pushed**, layer-generic.

Cost is **O(robot movement)**, never O(lawn). A glitch point allocates one stray tile, not a monster.

---

## 1. Spatial model — sparse tiles

```
TILE_CELLS   = 128          // cells per tile edge
RES          = 0.05 m       // cell size (display layers may use 0.10–0.20)
TILE_SPAN    = TILE_CELLS * RES = 6.4 m
tile_key(x,y)= ( floor(x / TILE_SPAN), floor(y / TILE_SPAN) )   // integer pair
```

- Tiles allocated **on demand** into `unordered_map<TileKey, Tile>`.
- Memory/work scale with **mown area**, not lawn bounds.

```cpp
struct Tile {
  int16_t tx, ty;                                 // tile key
  std::array<LayerCells, NUM_LAYERS> layers;      // one cell array per active layer
  std::bitset<TILE_CELLS*TILE_CELLS> dirty;       // cells changed since last publish
};
// LayerCells = std::vector<CellValue> (lazy-allocated when a layer first writes the tile)
```

`CellValue` is layer-defined (uint8 for coverage, float/uint16 for heatmaps).

---

## 2. Layer model — generic

```cpp
struct LayerDef {
  std::string id;                       // "coverage", "mow_current", ...
  CellValue (*value_fn)(const Ctx&);    // returns the cell contribution, or NONE to skip
  CellValue (*accumulate)(CellValue old, CellValue v);  // OR / max / running-avg / EWMA
  Encoding  encoding;                   // u8 | u16 | f32 (wire size)
  // colormap lives app-side
};

struct Ctx {                            // everything a layer might need at a swept cell
  double x, y;                          // EKF pose
  bool   blade_on;
  double speed;                         // |measured_twist|
  double mow_current, left_current, right_current;  // ESC telemetry
  // ...extend freely
};
```

Examples:
| layer | `value_fn` | `accumulate` |
|---|---|---|
| `coverage` | `blade_on ? 255 : NONE` | `max` (sticky) |
| `mow_current` | `blade_on ? current/max(speed,ε) : NONE` | running-avg (biomass proxy) |
| `slope` | `imu/odom derived` | last / avg |

Adding a layer = register a `LayerDef` + an app colormap. **Nothing else.**

---

## 3. Projector node (`map_projector`, replaces coverage raster)

Subscribes: EKF pose (~50 Hz), low-level status (blade + ESC currents), measured_twist.
Keeps a latest-value `Ctx` updated by the sensor callbacks.

**Per pose** (only inside a mowing session; outlier-rejected):
```
swept = cells along the swath (disc r=tool/2 + segment from prev pose)   // ~50 cells
for cell in swept:
  tile = tiles[tile_key(cell)]            // allocate if new
  for layer in registry:
    v = layer.value_fn(ctx_at(cell))
    if v != NONE:
      tile.cell[layer] = layer.accumulate(tile.cell[layer], v)
      tile.dirty.set(local_index)
```
Microseconds. The 50 Hz stream never touches publish/gzip → **no dropped poses**.

**Per tick (1 Hz timer)** — drain dirty into deltas:
```
for tile in tiles where tile.dirty.any():
  for layer in registry:
    delta[layer].push({ tx, ty, [(local_index, value) for dirty cells] })
  tile.dirty.reset()
for layer in registry:
  if delta[layer] non-empty: publish_delta(layer, delta[layer])
```
O(dirty cells) ≈ **~56 cells/layer/tick → a few hundred bytes.**

---

## 4. Transport

| topic | type | retained | rate | purpose |
|---|---|---|---|---|
| `map_layers/<id>/delta` | binary LayerDelta | no | 1 Hz | incremental live |
| RPC `layers.snapshot(id, job?)` | binary LayerSnapshot | — | on demand | full state for a new/refreshed client |
| RPC `layers.list` | json | — | — | available layers + meta (res, encoding, colormap hint) |

**LayerDelta** (binary, BSON or packed):
```
{ id, res, tile_cells, encoding,
  tiles: [ { tx, ty, cells: [ {i: uint16, v: <encoding>}, ... ] }, ... ] }
```
**LayerSnapshot** = same shape but full (every non-empty cell of every tile for that layer/job).
New client flow: `layers.snapshot` → apply → then follow `…/delta`. (No giant retained grid — ever.)

---

## 5. App side

```
LayerStore(id): Map<TileKey, TileData>      // mirrors the node's sparse tiles
 on snapshot → replace; on delta → patch the listed cells
 render: per dirty tile, blit into a tiled <canvas>/WebGL texture; colormap(value)→rgba
 toggle: show/hide; opacity per layer
```
- Never receives a full map; only deltas after the initial snapshot.
- Tiled rendering → only redraw changed tiles. Comfortable at 1 Hz, scales to any lawn.
- Historical job = `layers.snapshot(id, job_id)` → render, no live.

---

## 6. Persistence & post-hoc

- **Live** needs no disk — tiles are an in-RAM projection.
- **Resume / history**: persist the sparse tiles per job (small) **or** recompute by replaying the
  recorded track + sensors through the same `LayerDef` registry.
- **New layer added later** → recompute over old jobs from the recorded source. Source = position-
  history (track+blade) + sensor bags/streams. Nothing is lost.

---

## 7. Coverage refill (unchanged behaviour)

At area-finish the gap-detector reads the **`coverage` layer's tiles** (the mown set) → rasterize the
area minus coverage → connected gaps → slic3r fill paths → refill. Same logic, sparse input.

---

## 8. Migration / phasing

1. **Projector core**: sparse tiles + `coverage` layer + delta transport. Drop the dense grid +
   per-tick full publish. Gap-detect reads tiles. (Replaces today's `coverage_feedback` raster.)
2. **App**: `LayerStore` + delta/snapshot apply + tiled render. Replaces `CoverageLayer`.
3. **Prove generic**: add `mow_current` heatmap (value=current/speed, running-avg) — should be ~a
   `LayerDef` + a colormap, no infra change.
4. **Persistence + post-hoc recompute** + historical snapshots.

## 9. Numbers (sanity)
- TILE 128² @ 5 cm = 6.4 m, 16 KB/layer/tile (u8).
- Per pose: ~50 cells × layers (µs). Per-tick delta: ~56 cells/layer ≈ few hundred B.
- 1000 m² mown ≈ ~25 tiles → ~400 KB/layer RAM. Lawn size irrelevant.
- vs. today: one 26M-cell, 24 MB/msg grid at 1 Hz.
