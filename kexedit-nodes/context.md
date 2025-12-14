# kexedit-nodes

Node schema and implementations for track construction system.

## Purpose

- Node schema (ports, properties, iteration configs)
- All 8 node type implementations
- Pure Rust track segment generation

## Layout

```
kexedit-nodes/
├── Cargo.toml
└── src/
    ├── lib.rs           # Schema (PortId, PropertyId, NodeType, NodeSchema)
    ├── force.rs         # ForceNode
    ├── geometric.rs     # GeometricNode
    ├── curved.rs        # CurvedNode
    ├── bridge.rs        # BridgeNode
    ├── copy_path.rs     # CopyPathNode
    ├── anchor.rs        # AnchorNode
    ├── reverse.rs       # ReverseNode
    ├── reverse_path.rs  # ReversePathNode
    ├── golden.rs        # Golden test data (test-only)
    └── point_comparer.rs # Point comparison (test-only)
```

## Scope

**Schema**:
- PortId (14 ports), PropertyId (10 properties), NodeType (8 types)
- NodeSchema (const fn for O(1) lookups)
- PropertyIndex (bidirectional mapping)

**Nodes** (8/8 complete):
- ForceNode: Force-based sections (normal/lateral)
- GeometricNode: Steering-based sections (pitch/yaw/roll)
- CurvedNode: Curved paths with lead-in/out dampening
- BridgeNode: Cubic Bezier between anchors
- CopyPathNode: Matrix-transformed path following
- AnchorNode: Initial state creation
- ReverseNode: Direction reversal
- ReversePathNode: Path order reversal

## Entrypoints

Each node exports `NodeName::build()` or `::create()`/`::reverse()`

## Dependencies

- kexedit-core (Point, Frame, Keyframe, Forces, Curvature, sim)
- approx (dev, for floating point assertions)
- serde/serde_json (dev, for golden test JSON parsing)
