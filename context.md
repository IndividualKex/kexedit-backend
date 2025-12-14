# Rust Backend (Submodule)

Git submodule → [kexedit-backend](https://github.com/IndividualKex/kexedit-backend)

## Purpose

Standalone Rust workspace for FVD physics computation. Consumed by KexEdit via C FFI.

## Layout

```
rust-backend/  # Git submodule
├── kexedit-core/    # Pure domain layer (physics/math)
├── kexedit-nodes/   # Node-based track construction
├── kexedit-ffi/     # C FFI adapter
└── Cargo.toml       # Workspace manifest
```

## Commands

```bash
# Test Rust workspace
cd rust-backend && cargo test

# Build for Unity (from KexEdit root)
./build-rust.sh
```

## Architecture

Layered library: Core (domain) → Nodes (construction) → FFI (C adapter)

See [kexedit-backend repository](https://github.com/IndividualKex/kexedit-backend) for full documentation.
