.PHONY: setup-jaka run-jaka build

IP ?= 10.5.5.100

build:
	cargo build --release
	mkdir -p build
	cp target/release/jaka-cli build/jaka-cli
	@echo "Binary copied to build/jaka-cli"

setup-jaka:
	@echo "Downloading SDK from sdk/source.txt..."
	@wget -O sdk.zip $$(cat sdk/source.txt)
	@echo "Unzipping SDK..."
	@unzip -o sdk.zip -d sdk/
	@echo "SDK setup complete."
