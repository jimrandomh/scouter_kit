# AGENTS.md

ScouterKit: an alternative firmware (g1v0) for the Even Realities G1 glasses, its protocol (an on-arm VM)
and host libraries (C/C++, Go, Python) for Linux + BlueZ, with demos, tools and an Android example app.
MIT (`LICENSE`), except `examples/ssh/`: GPL-3.0-or-later (its own `LICENSE`, `NOTICE`, `licenses/`).

## Rules
- `README.md` and `DESIGN.md` are the user's: never edit them. Don't create doc files. The other docs
  (below) were written by agents; keep them free of personal data and local paths.
- Never run history-changing git commands (init, commit, amend, rebase, merge, push). Leave changes
  uncommitted for the user.
- No environment-variable configuration in the libraries (`src/`, the `go/` package, `python/scouterkit/`):
  explicit config fields (`Connection`, `sk_g1_config`) or compile-time defines (`SCOUTER_G1_*`, src/README.md
  "Configuration"). Tools, scripts and examples may take flags; the scripts also read `G1_WORK` and friends.
- `g1_firmware/flash.py`, `tools/` and the hardware tests act on real glasses: only when asked.

## Publishing hygiene
- No Even binaries or decompiled Even code in the tree. Even's images are fetched into the work directory by
  `g1_firmware/scripts/fetch_stock.sh` (URLs and SHA-256 in `g1_firmware/stock_images.txt`). Built g1v0
  images embed Even's net-core image and touch table: never commit or publish build outputs.
- g1-fw (FreedomCoder-dev, the Renode models) has no licence: it is only fetched (`scripts/fetch_g1fw.sh`);
  the tree holds only this project's changed lines (`g1_firmware/emulator/models/patches/*.ed`).
- No personal data or secrets: MAC addresses (use `AA:BB:CC:DD:EE:FF`, `00:00:00:00:00:00` or `LEFT_ADDR`),
  serials (the emulator's test serial is `S110LTST000042`), bond keys and LTKs, IP addresses, hostnames,
  usernames, emails, absolute home paths, SSH keys, keystores, logs and captures (btsnoop). The tools write
  to `logs/` by default; `.gitignore` keeps it, build outputs, caches and keystores out.
- Third-party code and assets live in `third_party/<name>/` with the upstream licence file and a `NOTICE`
  (origin, version or commit, SHA-256, what was changed or left out). Files converted from them
  (`src/base/departure_mono.inc`, `go/examples/term/fonts/*.skf`, `go/examples/g1/icons/*.4bpp`) keep the
  source's licence and have a NOTICE saying how they were made. Binaries built with the G1 backend contain
  concurrentqueue and TinyCrypt (BSD: ship their licences), with `SCOUTER_G1_MIC` also liblc3 (Apache-2.0),
  and the Go and Python libraries Departure Mono (OFL).
- Every dependency is fetched by a script at a pinned version with its SHA-256, or pinned in a lock file with
  hashes (`examples/ssh/gradle/verification-metadata.xml`, `g1_firmware/scripts/ncs_requirements.txt`).
  Nothing may depend on one machine's paths.
- GPL code (mosh, from `examples/ssh/`) must not move into the MIT parts.

## Layout
| Path | |
|---|---|
| `src/base/` | `scouter_kit.hpp/.cpp`: the device-independent gray display, fonts; `departure_mono.inc` built-in font |
| `src/g1/` | `scouter_kit_g1.hpp/.cpp`: the G1 backend (BlueZ over sd-bus, or emulated arms), program builder, mic, video |
| `src/c/` | `scouter_kit_c.h/.cpp`: the C layer the Go and Python wrappers use |
| `src/vm/` | the VM core (C11), shared by the firmware, its emulator builds and the library's emulated arms |
| `tests/` | C/C++ tests: `g1_test`, `c_test`, `mic_test` (emulated arms), `vm_test` (VM core, fuzzing), `dk_smoke` and `glasses_check` (hardware) |
| `go/` | Go package `scouterkit` (cgo; compiles the library from `src/` itself); `examples/` demos |
| `python/` | Python package `scouterkit` (ctypes over two shared objects from `build.sh`), `examples/`, `tests/` |
| `font_converter/` | `.ttf`/`.otf` to the VM's font blob (`.skf` or a C array) |
| `g1_firmware/` | g1v0: `firmware/` (Zephyr app), `g1min/` (a minimal app whose build gives the emulator's replica of the G1's MCUboot), `emulator/` (Renode harness, models, tests), `scripts/`, `container/`, `flash.py` |
| `tools/` | Python hardware test tools, `dk_hci/` (an nRF5340 DK as a second BlueZ controller) |
| `third_party/` | concurrentqueue, Departure Mono (+ Nerd Font), liblc3, Pebble icons, TinyCrypt's SHA-256 |
| `examples/ssh/` | "SSH Scouter" Android app (Kotlin + JNI, mosh client) |

## Build and test
Library tests, from the root (outputs land in the root; git-ignored):

    VM="src/vm/vm.c src/vm/vm_gfx.c"
    g++ -std=c++20 -O2 -DSCOUTER_G1_EMULATOR tests/g1_test.cpp src/base/scouter_kit.cpp $VM -o g1_test -lpthread && ./g1_test quick
    gcc -std=c11 -DSCOUTER_G1_EMULATOR -c tests/c_test.c -o c_test.o && g++ -std=c++20 -DSCOUTER_G1_EMULATOR c_test.o \
        src/c/scouter_kit_c.cpp src/base/scouter_kit.cpp src/g1/scouter_kit_g1.cpp $VM -o c_test -lpthread && ./c_test
    (cd tests && cc -std=c11 -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all -I../src/vm \
        vm_test.c vm_v4_ref.c ../src/vm/vm.c ../src/vm/vm_gfx.c -o ../vm_test) && ./vm_test

`mic_test` needs liblc3's objects (src/README.md "Tests"). The BlueZ builds need `pkg-config` and
libsystemd's headers. The firmware's iPhone notification relay, against a scripted phone (Zephyr stubbed in
`tests/ancs_stubs/`):

    cc -std=c11 -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all -Itests/ancs_stubs -Ig1_firmware/firmware/src \
        tests/ancs_test.c -o ancs_test && ./ancs_test

- Go (1.22+, cgo): `cd go && go vet ./... && go test -tags emulator ./...` (a few minutes; tag `emulator`
  = two emulated arms, no Bluetooth). Go's build cache does not see the library sources (`src/`, `third_party/`):
  after changing them, run `go build -a ./...` in `go/`. It rewrites the shared cache, so later plain `go run`
  uses the new code. Other tags and test builds have their own entries: give them `-a` once too
  (`go test -a -tags emulator ./...`). Speech demos: tags `sherpa`/`whisper`, libraries and models from
  `go/examples/internal/speech/fetch_*.sh` (go/README.md).
- Python (3.10+): `bash python/build.sh` (gcc, g++, pkg-config, libsystemd) builds
  `python/scouterkit/*.so`; then in `python/`: `pip install -e '.[test]'` (or `PYTHONPATH=.`) and
  `python -m pytest`. `SCOUTERKIT_GLASSES=1` adds the test on the glasses.
- Tools and font converter: `pip install -r tools/requirements.txt` (bleak, Pillow, pyserial); run scripts
  as `python3 -I tools/NAME.py`. `font_converter.py` needs Pillow.
- Firmware (g1_firmware/README.md, docs/DEVELOPMENT.md). Host: Linux x86-64 with rootless podman
  (Debian: `podman uidmap passt fuse-overlayfs git curl python3`), network for the setup, about 3 GB for the
  work directory plus about 0.5 GB per built version. The work directory is `G1_WORK`, default
  `g1_firmware/work` (git-ignored; locally it may be a symlink to a directory elsewhere).
  - `scripts/setup.sh build`: the podman images (`container/`, base images pinned by digest), NCS v2.5.1 and
    the Zephyr SDK 0.16.5-1 (`scripts/ncs_fetch.sh`: Python packages from `ncs_requirements.txt` by hash, the
    NCS checkout checked against its git commits), Even's 1.6.6 images (`fetch_stock.sh`).
  - `scripts/setup.sh emulator`: Renode 1.16.1, g1-fw's models with this project's patches, the QSPI seeds,
    the MCUboot replica.
  - `scripts/build.sh VERSION [NEXT]`: four variants (G1, next-version, DK, emulator test) into
    `$G1_WORK/build/`, each in a network-less container. `scripts/gate.sh VERSION`: the emulator tests
    (`emulator/tests/v5_tests.py`). `tools/dk_hci/build.sh` builds the DK controller image.
- Android app (examples/ssh/README.md): Android SDK (platform 34, NDK 27.3.13750724, CMake 3.31.6), JDK 17+.
  `native/third_party/build.sh [OUT]` (mosh, OpenSSL, protobuf at pinned SHA-256), then
  `./gradlew assembleGlassesDebug` (`-PthirdPartyDir=OUT`, `-PbuildRoot=DIR` optional;
  `assembleEmulatedDebug` runs emulated arms in the app). Release signing comes only from
  `examples/ssh/keystore.properties` (git-ignored: storeFile, storePassword, keyAlias, keyPassword).
  Dependency changes need new entries in `gradle/verification-metadata.xml`. Host tests:
  `cmake -S native -B BUILD -G Ninja -DTHIRD_PARTY_DIR=OUT && cmake --build BUILD && ctest --test-dir BUILD`
  (needs `mosh-server`).

## Conventions
- C++20 for the library (2-space indent), C11 for the VM core and the firmware (tabs, Zephyr style); lines up
  to about 120 columns. Go: gofmt. Python: 4 spaces, scripts run with `python3 -I`. Kotlin: official style.
- Byte layouts in `g1_firmware/docs/VM.md` are the source of truth: the VM core, the firmware, the library,
  the emulator and the tests change together.
- Comments and docs are terse and factual; no personal data, no local paths.

## Docs
- `README.md`, `DESIGN.md`: the user's overview and design.
- `g1_firmware/README.md`: the firmware for owners (install, risks, recovery). `g1_firmware/docs/VM.md`:
  protocol 5 and the on-arm VM. `docs/PROTOCOL.md`: protocol v4 (g1v0 0.4.x). `docs/DEVELOPMENT.md`:
  firmware internals and change notes.
- `src/README.md`: C++ library API, configuration, tests, third-party code. `src/vm/README.md`: the VM core.
- `go/README.md`, `python/README.md`: the wrappers and demos. `tools/README.md`: the tools.
  `examples/ssh/README.md`: the Android app.
