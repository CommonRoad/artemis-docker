IMAGE ?= commonroad-competition
TAG ?= 2026
PLATFORM ?= linux/amd64
ARGS ?= --scenarios planner/assets/scenarios/public/DEU_AugsburgCRC26P-23_16300_I-1.xml

.PHONY: build build-dev run test

build:
	docker build --platform "$(PLATFORM)" --target submission -t "$(IMAGE):$(TAG)" .

build-dev:
	docker build --platform "$(PLATFORM)" --target development -t "$(IMAGE):$(TAG)-dev" .

run:
	mkdir -p solutions
	docker run --rm --platform "$(PLATFORM)" --cpus=1 --memory=4g --memory-swap=4g \
		--network none \
		--mount "type=bind,source=$(CURDIR)/my_submission,target=/assignment,readonly" \
		--mount "type=bind,source=$(CURDIR)/solutions,target=/commonroad/solutions" \
		"$(IMAGE):$(TAG)" $(ARGS) --submission /assignment --solutions /commonroad/solutions

# Slim checkouts omit tests: run "python -m planner.fetch_data --tests"
# before make build-dev when they are missing.
test:
	docker run --rm --platform "$(PLATFORM)" --cpus=1 --memory=4g --memory-swap=4g \
		--network none --entrypoint python "$(IMAGE):$(TAG)-dev" -m pytest tests -q
