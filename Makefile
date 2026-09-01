.PHONY: setup-jaka run-jaka build build-linux build-win

IP ?= 10.5.5.100

UNAME_S := $(shell uname -s)
MINGW := $(shell command -v x86_64-w64-mingw32-gcc 2>/dev/null)

# Current-platform release build, output to build/jaka-cli
build:
	cargo build --release
	mkdir -p build
	cp target/release/jaka-cli build/jaka-cli
	@echo "Binary copied to build/jaka-cli"

# Linux release build. Cross-compiles from other systems is not supported,
# use a Linux machine or WSL
build-linux:
ifeq ($(UNAME_S),Linux)
	cargo build --release
	mkdir -p build/linux
	cp target/release/jaka-cli build/linux/jaka-cli
	@echo "Linux binary copied to build/linux/jaka-cli"
else
	@echo "Cross-compiling to Linux from $(UNAME_S) is not supported"
	@echo "Build on a Linux machine or inside WSL instead"
endif

# Windows release build. Native Windows builds with the default toolchain,
# Linux cross-compiles with mingw-w64 (the gnu target)
build-win:
ifeq ($(UNAME_S),Linux)
	@if [ -z "$(MINGW)" ]; then \
		echo "mingw-w64 not found, install it with: sudo pacman -S mingw-w64-gcc"; \
		exit 1; \
	fi
	cargo build --release --target x86_64-pc-windows-gnu
	mkdir -p build/win
	cp target/x86_64-pc-windows-gnu/release/jaka-cli.exe build/win/
	cp target/x86_64-pc-windows-gnu/release/jakaAPI.dll build/win/
	@echo "Windows binaries copied to build/win/"
else ifneq (,$(findstring MINGW,$(UNAME_S)))
	cargo build --release
	mkdir -p build/win
	cp target/release/jaka-cli.exe build/win/
	cp target/release/jakaAPI.dll build/win/
	@echo "Windows binaries copied to build/win/"
else
	@echo "Cross-compiling to Windows from $(UNAME_S) is not supported"
	@echo "Build on Windows or Linux with mingw-w64 instead"
endif

run-jaka:
	cargo run -- --ip $(IP) rot

setup-jaka:
	@echo "Downloading SDK from sdk/source.txt..."
	@wget -O sdk.zip $$(cat sdk/source.txt)
	@echo "Unzipping SDK..."
	@unzip -o sdk.zip -d sdk/
	@echo "SDK setup complete."
