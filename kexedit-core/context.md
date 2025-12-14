# kexedit-core (Rust)

Pure domain layer - physics and mathematics with no external dependencies.

## Purpose

High-performance Rust implementation of Core domain logic. No FFI - just pure business logic.

## Layout

```
kexedit-core/
├── src/
│   ├── math.rs          # Float3, Quaternion (with quat-quat multiplication)
│   ├── frame.rs         # Orthonormal frames, pitch/yaw/roll
│   ├── sim.rs           # Physics constants, energy functions
│   ├── curvature.rs     # Track curvature computation
│   ├── forces.rs        # G-force computation
│   ├── point.rs         # Complete track point state
│   ├── keyframe.rs      # Keyframes with Bezier evaluator
│   ├── frame_change.rs  # Frame transformations
│   ├── physics_params.rs # Physics parameters
│   └── lib.rs           # Public API exports
├── Cargo.toml
└── context.md
```

## Testing

```bash
# All tests (65 passing)
cargo test
```

## Dependencies

None. Test dependencies: `approx` for float comparisons.

## Consumers

- `kexedit-ffi` - FFI adapter for Unity C# interop
- Future: `kexedit-nodes`, `kexedit-wasm`
