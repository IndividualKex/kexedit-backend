# Rust Workspace

Portable physics/math core implementation for KexEdit.

## Purpose

High-performance Rust implementation of Core domain logic, designed for portability (native, WASM) and FFI integration with Unity C#.

## Layout

```
rust-backend/
├── kexedit-core/    # Pure domain layer (physics/math primitives)
├── kexedit-nodes/   # Node schema + implementations
├── kexedit-ffi/     # FFI adapter for Unity C# interop
├── Cargo.toml       # Workspace manifest
├── build.sh         # Build script
└── context.md
```

## Commands

```bash
# Build all
cargo build --release

# Test all
cargo test

# Build FFI for Unity
cargo build --release -p kexedit-ffi
# Output: target/release/kexedit_core.{dll,dylib,so}
```

## Architecture

- **kexedit-core**: Pure Rust domain logic (76 tests)
- **kexedit-nodes**: Node schema + all 8 implementations (39 tests)
- **kexedit-ffi**: C FFI adapter (2 tests)

## Dependencies

None for runtime. Test dependencies: `approx`.
