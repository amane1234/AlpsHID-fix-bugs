# Protocol tests

`AlpsHID/AlpsProtocol.hpp` holds everything the driver does with untrusted device data: the T4/U1 report layouts, contact and trackstick decoding, register request packets, T4 reply validation, feature-reply coverage and sensor geometry. It has no IOKit dependency, so the kext and these host tests compile the same code.

## Unit tests

```bash
make -C Tests test
```

Runs under AddressSanitizer and UBSan. Where logic moved out of the kext, the original implementation (AlpsHID 36d1fce) is kept in the test as a reference and compared exhaustively or with a fixed pseudo-random sequence: checksum, T4 contact decoding and Y inversion, request packets, T4 reply checks, geometry and button decoding.

## Fuzzing

```bash
make -C Tests run-fuzz FUZZ_SECONDS=300
```

| Target | Covers |
|---|---|
| `fuzz_input_report` | The input report path of `handleInterruptReport`: report classification, the copy length rule, every contact slot and the trackstick deltas, with geometry from every T4 `CONFIG_3` value |
| `fuzz_feature_reply` | `writeBytes()` coverage tracking, the reply length check, T4 reply validation and request packets |

Inputs are copied into exact-size heap buffers, so a read past the data the driver validated is reported by ASan. Seeds are in `corpus/<target>`; new findings go to `build/corpus/<target>`. A crash writes `crash-*` in `Tests/`; reproduce it with `./build/<target> crash-...`.

libFuzzer needs clang with its runtime: on Linux `clang` and `libclang-rt-dev` (or the versioned package), on macOS Homebrew `llvm` (Apple's clang has no libFuzzer). The unit tests build with any C++17 compiler.

The GitHub Actions workflow `.github/workflows/tests.yml` runs both on every push.
