# kexedit-ffi

FFI adapter - exposes kexedit-core as C-compatible library for Unity.

## Purpose

Hexagonal "adapter" layer that bridges pure Rust domain logic to C# via FFI. Translates between Rust types and C ABI.

## Layout

```
kexedit-ffi/
├── src/
│   └── lib.rs     # C FFI exports
├── Cargo.toml
└── context.md
```

## Building

```bash
cargo build --release -p kexedit-ffi
# Output: target/release/kexedit_core.dll (Windows)
#         target/release/libkexedit_core.dylib (macOS)
#         target/release/libkexedit_core.so (Linux)
```

Copy DLL to `Assets/Runtime/Plugins/` for Unity integration.

## FFI Convention

All exports:
- Prefixed with `kexedit_`
- Use `#[no_mangle]`
- `unsafe extern "C"`
- Null-safe (check all pointers)

Example:
```rust
#[no_mangle]
pub unsafe extern "C" fn kexedit_frame_rotate_around(
    frame: *const Frame,
    axis: *const Float3,
    angle: f32,
    out: *mut Frame
) { /* ... */ }
```

## Dependencies

- `kexedit-core` - Pure domain logic
