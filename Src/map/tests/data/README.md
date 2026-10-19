# Src/map/tests/data

Testing data for the `map` component tests. These files are **component-local
working copies**, not versioned repository fixtures: per the Wave 4 task
instructions they deliberately do not live under `tests/fixtures/v1/` (that
tree is frozen and owned by the integration owner). Promoting the example map
to a versioned fixture is a coordinator decision for M4/M5.

| File | Origin / provenance | Purpose |
| --- | --- | --- |
| `mapping_example.osm` | Copy of `lanelet2_maps/res/mapping_example.osm` from the locked Lanelet2 1.2.3 archive (`dependencies.lock.json`, sha256 `7617046cdb4aa8893df1f4c396051903a3f2d9783fd7dad434f5b084bfb47dac`); upstream license BSD-3-Clause (`Lanelet2-1.2.3/LICENSE`). Local copy sha256: see `src/map/tests/test_import.cpp`. | Official small example map for M1/M2 acceptance (import, projection round trip, queries, cropping). |
| `invalid_xml.osm` | Written for this repository (Wave 4 map agent). | Malformed XML import diagnostics. |
| `no_lanelets.osm` | Written for this repository (Wave 4 map agent). | Valid OSM without lanelets: import succeeds with a diagnostic warning. |
| `disconnected.osm` | Written for this repository (Wave 4 map agent). | Two isolated lanelets: routing no-path and empty-adjacency diagnostics. |

Notes:

- The local working copy of `mapping_example.osm` is byte-identical to the
  upstream file in the locked archive; `test_import.cpp` pins its SHA-256 and
  byte size so accidental edits fail the test suite.
- The map is located near Munich (49.0034 N, 8.4243 E). The fixed test origin
  used by the suite is the first node of the file: lat 49.00345654351,
  lon 8.42427590707.
