# Building CH32 on Windows

This document explains how Rotorflight selects and verifies the WCH RISC-V toolchain for CH32 targets.

## 1. The two possible toolchains

A CH32 build can use either of these:

- The Rotorflight-local SDK:

  ```text
  tools/riscv-wch-elf-12.2.0/bin/riscv-wch-elf-gcc.exe
  ```

- A system-wide compiler already present in `PATH`, for example the compiler installed by MounRiver Studio.

Rotorflight prefers the local SDK. If the local SDK is missing, it falls back to `PATH`.

This means that a successful build does not automatically prove that the Rotorflight SDK was used. The machine might have used MounRiver's compiler instead.

## 2. Install the local CH32 SDK

From the repository root, run:

```bash
make ch32_sdk_install
```

On Windows, Rotorflight uses this archive:

```text
https://github.com/TianpeiLee/riscv-gun-toolchain-12.2.0-x86_64-w64-mingw32-riscv-wch-elf/archive/refs/tags/12.2.0.tar.gz
```

The installer downloads the archive to:

```text
downloads/12.2.0.tar.gz
```

It extracts the compiler to:

```text
tools/riscv-wch-elf-12.2.0
```

The `.installed` file records that extraction has completed:

```text
tools/riscv-wch-elf-12.2.0/.installed
```

The archive contains an extra top-level directory. This command removes that extra level during extraction:

```make
 tar --strip-components=1
```

## 3. Verify the local SDK directly

Check that the compiler exists:

```bash
ls tools/riscv-wch-elf-12.2.0/bin/riscv-wch-elf-gcc.exe
```

Run the compiler directly:

```bash
./tools/riscv-wch-elf-12.2.0/bin/riscv-wch-elf-gcc.exe --version
```

The output should identify the WCH RISC-V GCC compiler and version 12.2.0.

The `.exe` suffix is important on Windows. The makefile checks for this suffix when deciding whether the local SDK is installed.

## 4. Find the compiler from PATH

To see whether Windows or MounRiver provides another compiler, run:

```bash
command -v riscv-wch-elf-gcc
```

On Windows Command Prompt, use:

```bat
where riscv-wch-elf-gcc
```

In PowerShell, use:

```powershell
Get-Command riscv-wch-elf-gcc
```

For example, a MounRiver result may look like:

```text
C:/MounRiver/MounRiver_Studio2/resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC12/bin/riscv-wch-elf-gcc
```

This is the fallback compiler. It is not the Rotorflight-local SDK.

## 5. Verify which compiler Make will use

Run:

```bash
make -pn TARGET=CH32H417 | grep -E '^(CH32_SDK_GCC|CH32_SDK_PREFIX|CH32_SDK_CC|ARM_SDK_PREFIX|CROSS_CC)[[:space:]]*[:?+]?='
```

When the local SDK is selected, the important output should contain paths similar to:

```text
CH32_SDK_PREFIX := ./tools/riscv-wch-elf-12.2.0/bin/riscv-wch-elf-
ARM_SDK_PREFIX := ./tools/riscv-wch-elf-12.2.0/bin/riscv-wch-elf-
CROSS_CC := ./tools/riscv-wch-elf-12.2.0/bin/riscv-wch-elf-gcc
```

If the output only contains:

```text
riscv-wch-elf-gcc
```

then the local SDK was not detected and Make is using a compiler from `PATH`.

## 6. Inspect the generated build command

Use a dry run so no files are compiled:

```bash
make -n V=1 TARGET=CH32H417 > build-command.txt
```

Search the generated commands:

```bash
 grep -m 1 riscv-wch-elf-gcc build-command.txt
```

When the local SDK is active, the command should contain:

```text
./tools/riscv-wch-elf-12.2.0/bin/riscv-wch-elf-gcc
```

Do not pipe the make command directly into `head` or `grep` while diagnosing it. Make may receive a broken-pipe error when the reader exits early. Redirect the output to a file first, as shown above.

## 7. Perform a real clean build

To verify compilation and linking with the selected SDK:

```bash
make clean TARGET=CH32H417
make TARGET=CH32H417
```

A successful build should produce files such as:

```text
obj/rotorflight_<version>_CH32H417.hex
obj/main/rotorflight_CH32H417.elf
```

The build output should show the local compiler path when the SDK is being used.

## 8. Why the build can work without installing the SDK

The makefile intentionally has a fallback:

```make
ifneq ($(wildcard $(CH32_SDK_GCC)),)
  CH32_SDK_PREFIX := $(CH32_SDK_BIN)/riscv-wch-elf-
else
  CH32_SDK_PREFIX := riscv-wch-elf-
endif
```

The first branch is used when the Rotorflight-local compiler exists.

The second branch tells Make to search `PATH`. Since MounRiver had already added its compiler directory to `PATH`, this command worked:

```text
riscv-wch-elf-gcc
```

The earlier successful build therefore used MounRiver. It did not use the Rotorflight SDK.

## 9. How the compiler prefix is used

The build does not store every compiler name separately. It stores one prefix:

```text
riscv-wch-elf-
```

The Makefile appends tool names to that prefix:

```make
CROSS_CC := $(ARM_SDK_PREFIX)gcc
OBJCOPY  := $(ARM_SDK_PREFIX)objcopy
SIZE     := $(ARM_SDK_PREFIX)size
```

For a local SDK, the resulting commands are:

```text
./tools/riscv-wch-elf-12.2.0/bin/riscv-wch-elf-gcc
./tools/riscv-wch-elf-12.2.0/bin/riscv-wch-elf-objcopy
./tools/riscv-wch-elf-12.2.0/bin/riscv-wch-elf-size
```

For a PATH installation, the resulting commands are:

```text
riscv-wch-elf-gcc
riscv-wch-elf-objcopy
riscv-wch-elf-size
```

`ARM_SDK_PREFIX` is an old shared variable name in this project. It is used for both ARM and RISC-V cross-compilers.

## 10. Remove and reinstall the SDK

Remove the local CH32 SDK and its downloaded archive:

```bash
make ch32_sdk_clean
```

After cleaning, a CH32 build may still work if MounRiver is in `PATH`. Verify the compiler source with the commands in sections 4 and 5.

Reinstall the local SDK with:

```bash
make ch32_sdk_install
```

## 11. Adding another toolchain later

A new toolchain integration normally needs these parts:

1. A version variable.
2. An OS-specific download URL.
3. A download filename.
4. An installation directory.
5. A compiler executable used as an installation marker.
6. An install target.
7. A clean target.
8. A local-versus-`PATH` selection rule.
9. A compiler existence check.
10. Documentation and a dry-run verification command.

The general pattern is:

```make
SDK_VERSION := 1.0.0
SDK_URL := <archive URL>
SDK_FILE := $(notdir $(SDK_URL))
SDK_DIR := $(TOOLS_DIR)/my-toolchain-$(SDK_VERSION)
SDK_BIN := $(SDK_DIR)/bin
SDK_CC := $(SDK_BIN)/my-target-gcc
SDK_PREFIX := $(SDK_BIN)/my-target-

.PHONY: my_sdk_install
my_sdk_install: $(SDK_DIR)/.installed

$(SDK_DIR)/.installed: $(DL_DIR)/$(SDK_FILE)
	mkdir -p $(SDK_DIR)
	tar -C $(SDK_DIR) --strip-components=1 -xzf "$<"
	touch "$@"
```

The most important rule is to keep toolchain installation and toolchain selection in `make/tools.mk`. MCU files should define architecture flags, source files, and linker scripts, but should not override the compiler path.
