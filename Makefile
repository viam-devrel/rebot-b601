PYTHON ?= .venv/bin/python
UV ?= uv

.PHONY: venv test lint format module assets clean check-bootstrap

venv:
	$(UV) venv .venv
	$(UV) pip install -p $(PYTHON) -r requirements.txt -r requirements-dev.txt

test:
	$(PYTHON) -m pytest

lint:
	$(PYTHON) -m ruff check src tests tools
	$(PYTHON) -m ruff format --check src tests tools

format:
	$(PYTHON) -m ruff format src tests tools
	$(PYTHON) -m ruff check --fix src tests tools

# Registry tarball. Excludes bytecode (0.1.0 shipped stray .pyc files).
module:
	rm -f module.tar.gz
	tar --exclude='__pycache__' --exclude='*.pyc' -czf module.tar.gz \
		meta.json README.md CHANGELOG.md run.sh requirements.txt src

assets:
	$(PYTHON) tools/build_assets.py

# Simulate a clean install: copy the tree without .venv and run the entrypoint's self-check.
check-bootstrap:
	rm -rf /tmp/rebot-b601-bootstrap && mkdir -p /tmp/rebot-b601-bootstrap
	tar --exclude='.venv' --exclude='.git' --exclude='__pycache__' -cf - . | tar -xf - -C /tmp/rebot-b601-bootstrap
	cd /tmp/rebot-b601-bootstrap && ./run.sh --check

clean:
	rm -rf module.tar.gz .pytest_cache .ruff_cache
	find . -name __pycache__ -type d -prune -exec rm -rf {} +
