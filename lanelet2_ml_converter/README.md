# Lanelet2 ML Converter

Converter module to convert Lanelet2 maps into local instance labels for machine learning tasks.

**Note:** This module is experimental, so there may be breaking API changes in the future!

For a detailed description of how the labels are generated and what the data model looks like, see
[here](doc/InstanceLabels.md).

![](doc/summary_flowchart.png)

## Usage Examples

### Python

```python
import lanelet2
from lanelet2.core import BasicPoint2d
from lanelet2.ml_converter import MapDataInterface

pos = BasicPoint2d(10, 10)                              # set your local reference frame origin
yaw = 0                                                 # set your local reference frame yaw angle (heading)
mDataIf = MapDataInterface(ll2_map)                     # get the MapDataInterface object and pass the ll2 map
mDataIf.setCurrPosAndExtractSubmap2d(pos, yaw)          # extract the local submap
mData = mDataIf.mapData(True)                           # get the MapData local instance labels
tfData = mData.getTensorInstanceData(True, False)       # get the local instance labels as numpy arrays
```

### C++
```c++
#include "lanelet2_ml_converter/MapData.h"
#include "lanelet2_ml_converter/MapDataInterface.h"
#include "lanelet2_ml_converter/MapInstances.h"

using TensorData = lanelet::ml_converter::MapData::TensorInstanceData;

lanelet::BasicPoint2d pos(10, 10);                      // set your local reference frame origin
double yaw = 0;                                         // set your local reference frame yaw angle (heading)
MapDataInterface mDataIf(laneletMap);                   // get the MapDataInterface object and pass the ll2 map
mDataIf.setCurrPosAndExtractSubmap2d(pos, yaw);         // extract the local submap
lanelet::ml_converter::MapDataPtr mData = mDataIf.mapData(true);  // get the MapData local instance labels
TensorData tData = mData->getTensorInstanceData(true, false);     // get the labels as Eigen mats
```

## Features

- Access lanelet map information as numpy arrays from python, in a representation directly usable for machine learning tasks
- Compound labels for independence from map annotation artifacts
- Configurable label sets through type groupings
- Full traceability to the underlying map element for all instances, including the compound instances
- Real-time capable optimized C++ implementation (around 3 ms for one set of local instance labels)
- Save and load generated labels in both binary and human-readable XML format


## Components

### `MapDataInterface`

Main interface class that is used to generate `MapData` objects. Set a local reference frame pose with
`setCurrPosAndExtractSubmap*()`, then get the labels for it with `mapData()`. For more than one pose, the
`mapDataBatch*()` methods do the same without touching the current position.

Its `Configuration` holds the parameters of the extraction and processing, most importantly the extents of the
local reference frame, the number of points to resample to, and the type groupings (see below).

### `MapData`

`MapData` objects hold all local instance labels of a local reference frame pose. They are created in two
phases: `build()` extracts the instances from a local submap, `processAll()` brings them into the local
reference frame. `MapDataInterface` does both for you.

### `TensorInstanceData`

Nested class of `MapData` that holds all local instance labels in a convenient numpy array / Eigen matrix form,
grouped by type. Obtained via `MapData::getTensorInstanceData()`.

### `LaneLineStringInstance` / `CompoundLaneLineStringInstance` / `LaneletInstance` / `TEInstance` / ...

Each object of these classes holds a single instance label. You can access various information such as the map
element ID and the raw or partially processed LineStrings.

## The processing pipeline

Every instance is created from one map element and then brought into the local reference frame by cutting,
transforming and resampling its geometry. The result of each stage is kept and accessible:

| Stage | Accessor | Frame |
| --- | --- | --- |
| raw | `rawInstance` | map |
| cut | `cutInstance` | map |
| cut + transformed | `cutAndTransformedInstance` | local |
| cut + transformed + resampled | `cutTransformedAndResampledInstance` | local |

Each stage is a *list* of line strings, because clipping can split one line into several disjoint pieces. The
lists are index aligned per surviving piece.

Two flags tell you what happened to an instance:

- `wasCut`: the local bounding box removed some geometry, i.e. the instance reaches beyond it
- `valid`: enough of the instance survived to be usable. Invalid instances are what the `validXXX()` accessors
  of `MapData` filter out, and they are not part of the tensor data

Resampling is optional. Passing fewer than 2 points (or setting `resampleLanes` / `resampleTE` to `False`)
leaves the instances at their original point count, in which case
`cutTransformedAndResampledInstance` stays empty and the other accessors fall back to the transformed geometry.

## Type groupings

A `LineStringTypeGrouping` maps lists of `LineStringType`s to a representative type and does two things at once:

1. it decides which boundaries are chained into one compound instance — consecutive boundaries are chained as
   long as they stay within the same group
2. it decides the label of the result — a compound instance carries the representative type of its group

This is how the label set is adapted to what a model is supposed to predict. Four groupings ship with the
module:

| Grouping | Effect |
| --- | --- |
| `getDefaultLineStringTypeGrouping()` | every type is its own group, nothing is merged |
| `getRoadBorderMergedGrouping()` | merges road border, fence, curbstones, guard rail, building and wall into `RoadBorder` |
| `getMapTRDefaultSimpleGrouping()` | as above, plus all lane markings merged into `Divider` |
| `getM3TRDefaultGrouping()` | as `RoadBorderMerged`, plus solid variants merged into `Solid` and dashed ones into `Dashed` |

A grouping must assign a group to **every** type a lanelet boundary can have. Otherwise the ungrouped types
would all share the "not found" group and unrelated boundaries would be chained into a single compound
instance — `MapData::build()` therefore rejects such a grouping.

A `TETypeGrouping` works the same way for traffic elements, except that it only relabels: traffic elements are
never chained.

## Traffic elements and edges

Traffic elements are the labels that are not part of a lane. They are collected from line strings and polygons
of the map that carry the respective tags:

| Tag | Becomes |
| --- | --- |
| `type=stop_line` | `TEType::StopLine` (unless tagged `artificial`) |
| `type=arrow` | the `Arrow*` types |
| `type=symbol` | `BikeSymbol`, `BusSymbol`, `Symbol30/50/70` |
| `type=traffic_light`, `traffic_light_bikes`, `traffic_light_pedestrians` | the `TL*` types |
| `type=traffic_sign` | the `TS*` types, mapped from the German StVO code in the `subtype` tag |

Crosswalk lanelets are turned into compound instances tracing their closed perimeter, labelled
`ZebraCrossing` or `PedestrianCrossing` depending on how their borders are tagged.

Two kinds of edges connect them to the rest of the data:

- `teToCenterlineEdges`: a traffic element to the compound centerline of the lanelet it applies to, e.g. a stop
  line to the lane it stops, or an arrow to the lane it is painted on
- `teToTEEdges`: a traffic element to another one, e.g. a traffic light to its stop line

Both are also available in `TensorInstanceData` as index based edges, where the indices refer to the position
of an instance within its own type.

## Drivable space borders

The outer boundary of the space a vehicle may physically drive on is expected to be annotated explicitly, with the tag
`drivable_space_border=true` on a LineString. These LineStrings may (but do not have to) live next to the lanelets rather than on them:
they are not a lanelet boundary, they do not have to follow the lane structure, and their own `type` / `subtype`
tags are irrelevant here. Everything tagged this way becomes `LineStringType::DrivableArea`.

This is what makes the drivable space independent of how the lane boundaries happen to be tagged. 
Trying to infer the drivable area polygon from the lanelets is algorithmically difficult and very
ambiguous depending on map-specific tagging and definitions, therefore this explicit annotation is required.

The individual borders are chained into compound instances so that a model does not see the splits of the
annotation:

- two borders are considered connected if they **share an endpoint point**, i.e. the same `Point3d` id — not
  merely the same coordinates
- each connected group is walked into continuous chains, starting at an open end where there is one and
  inverting borders as needed so that a chain runs in one direction
- a junction where more than two borders meet ends a chain, so a branching drivable space yields several
  compound instances rather than one with a jump in it

Each border therefore shows up twice: once as an individual `DrivableArea` instance under its own map id, and
once as a member of the compound instance it was chained into. `associatedCpdLineStringsOfType(mapId,
LineStringType.DrivableArea)` gets you from the former to the latter.

## Important to know

- **Drivable space border tagging**: The extent of the drivable space is taken from the LineStrings tagged `drivable_space_border=true`, so a map that does not carry that tag produces no `DrivableArea` instances at all.
- **Point identity**: Drivable space borders are chained by shared point ids. Two borders that meet visually but end in two distinct points at the same position are *not* chained, so they end up as separate compound instances.
- **Python numpy arrays**: To make it possible to convert the underlying Eigen matrices to numpy arrays, they are copied for each access (such as calling `getTensorInstanceData`). This means that if you change a numpy array returned from a function like this, the underlying Eigen matrix or object will not be modified!
- **Traffic rules**: The traffic rules used for the routing graphs are currently fixed to Germany, for vehicles and bicycles.
- **Parametrization**: Only `ParametrizationType::LineString` is implemented so far, the Bezier variants throw.
