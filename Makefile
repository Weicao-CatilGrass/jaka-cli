.PHONY: setup-jaka run-jaka

IP ?= 10.5.5.100

run-jaka:
	cargo run -- --ip $(IP) rot

setup-jaka:
	@echo "Downloading SDK from sdk/source.txt..."
	@wget -O sdk.zip $$(cat sdk/source.txt)
	@echo "Unzipping SDK..."
	@unzip -o sdk.zip -d sdk/
	@echo "SDK setup complete."
