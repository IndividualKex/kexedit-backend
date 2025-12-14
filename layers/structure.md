# Project Structure

Standalone Rust library for Force Vector Design (FVD) physics and roller coaster track generation

## Stack

- Language: Rust (edition 2021)
- Build: Cargo (Rust build system)
- Testing: Cargo test (unit and integration tests)
- FFI: C ABI via cdylib for external integration

## Commands

- Build: `cargo build --release` (Production build)
- Build FFI: `cargo build --release -p kexedit-ffi` (Build only FFI layer)
- Test: `cargo test` (Run all tests)
- Test package: `cargo test -p [package-name]` (Test specific package)
- Check: `cargo check` (Fast compile check)
- Clippy: `cargo clippy` (Linting)
- Format: `cargo fmt` (Code formatting)

## Layout

```
kexedit-backend/
├── CLAUDE.md  # Global context (Tier 0)
├── Cargo.toml  # Workspace manifest
├── Cargo.lock  # Dependency lock file
├── kexedit-core/  # Pure physics and math domain layer
│   ├── context.md  # Module context
│   ├── Cargo.toml  # Package manifest
│   ├── src/  # Source code
│   │   ├── lib.rs  # Library root
│   │   ├── frame.rs  # Reference frame types
│   │   ├── point.rs  # 3D points and track data
│   │   ├── math.rs  # Vector/quaternion math
│   │   ├── keyframe.rs  # Interpolation
│   │   ├── forces.rs  # Force calculations
│   │   ├── curvature.rs  # Curvature computations
│   │   ├── sim.rs  # Physics simulation
│   │   ├── frame_change.rs  # Frame transformations
│   │   └── physics_params.rs  # Physical constants
│   └── tests/  # Integration tests
├── kexedit-nodes/  # Node-based track construction layer
│   ├── context.md  # Module context
│   ├── Cargo.toml  # Package manifest
│   └── src/  # Source code
│       ├── lib.rs  # Library root
│       ├── anchor.rs  # Anchor nodes
│       ├── force.rs  # Force-based nodes
│       ├── geometric.rs  # Geometric primitives
│       ├── curved.rs  # Curved sections
│       ├── golden.rs  # Golden ratio banking
│       ├── bridge.rs  # Bridge connections
│       ├── reverse.rs  # Reverse direction
│       ├── reverse_path.rs  # Path reversal
│       ├── copy_path.rs  # Path copying
│       └── point_comparer.rs  # Point utilities
├── kexedit-ffi/  # C FFI adapter layer
│   ├── context.md  # Module context
│   ├── Cargo.toml  # Package manifest (cdylib)
│   └── src/
│       └── lib.rs  # C ABI exports
├── layers/
│   ├── structure.md  # Project-level context (Tier 1)
│   └── context-template.md  # Template for context files
└── target/  # Build artifacts (gitignored)
```

## Architecture

**Pattern**: Layered library architecture with pure domain core

- **kexedit-core**: Pure physics and math primitives (no external dependencies)
  - Frame calculations, vector math, physics simulation
  - Keyframe interpolation, force computation
  - No I/O, no FFI, fully portable

- **kexedit-nodes**: Node-based track construction (depends on core)
  - High-level track building primitives
  - Node types for different track elements
  - Pure Rust, no FFI dependencies

- **kexedit-ffi**: C FFI adapter (depends on core and nodes)
  - Exports selected functions via C ABI
  - Thin wrapper with null safety
  - Built as cdylib for external consumption

**Flow**: External consumer → FFI layer → Nodes layer → Core domain logic

## Entry points

- Core library: `kexedit-core/src/lib.rs` (Physics and math primitives)
- Nodes library: `kexedit-nodes/src/lib.rs` (Track construction)
- FFI library: `kexedit-ffi/src/lib.rs` (C ABI exports)

## Naming Conventions

- Files: snake_case for modules (e.g., `frame_change.rs`, `point_comparer.rs`)
- Directories: kebab-case for packages (e.g., `kexedit-core/`, `kexedit-ffi/`)
- Types: PascalCase (e.g., `Frame`, `Point`, `Quaternion`)
- Functions: snake_case (e.g., `evaluate()`, `rotate_around()`)
- Constants: SCREAMING_SNAKE_CASE (e.g., `DEFAULT_GRAVITY`)
- FFI exports: prefixed with `kexedit_` (e.g., `kexedit_frame_rotate_around`)

## Configuration

- Workspace: `Cargo.toml` (Workspace-level settings and shared dependencies)
- Package manifests: `*/Cargo.toml` (Individual package configuration)
- Dependency lock: `Cargo.lock` (Exact dependency versions)

## Where to add code

- Pure physics/math → `kexedit-core/src/`
- Track construction logic → `kexedit-nodes/src/`
- FFI bindings → `kexedit-ffi/src/lib.rs`
- Unit tests → `*/src/*.rs` (inline with `#[cfg(test)]`)
- Integration tests → `*/tests/`
