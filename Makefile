.PHONY: clean setup-jaka run-jaka build build-linux build-win

IP ?= 10.5.5.100

UNAME_S := $(shell uname -s)
MINGW := $(shell command -v x86_64-w64-mingw32-gcc 2>/dev/null)

# Copy every jaka-* binary of the current platform out of the target dir.
# The .d files are rustc dependency files, not binaries
define copy_bins
	@for bin in $(1)/jaka-*; do \
		case "$$bin" in \
			*.d) ;; \
			*) cp "$$bin" $(2)/; \
						echo "Copied $$(basename $$bin) to $(2)/";; \
		esac; \
	done
endef

# Remove the build directory and clean cargo artifacts
clean:
	rm -rf build
	cargo clean --workspace

# Current-platform release build, output to build/
build:
	cargo build --release --workspace
	mkdir -p build
	$(call copy_bins,target/release,build)

# Linux release build. Cross-compiles from other systems is not supported,
# use a Linux machine or WSL
build-linux:
ifeq ($(UNAME_S),Linux)
	cargo build --release --workspace
	mkdir -p build/linux
	$(call copy_bins,target/release,build/linux)
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
	cargo build --release --target x86_64-pc-windows-gnu --workspace
	mkdir -p build/win
	$(call copy_bins,target/x86_64-pc-windows-gnu/release,build/win)
	cp target/x86_64-pc-windows-gnu/release/jakaAPI.dll build/win/
	@echo "Copied jakaAPI.dll to build/win/"
else ifneq (,$(findstring MINGW,$(UNAME_S)))
	cargo build --release --workspace
	mkdir -p build/win
	$(call copy_bins,target/release,build/win)
	cp target/release/jakaAPI.dll build/win/
	@echo "Copied jakaAPI.dll to build/win/"
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
