# Local dev shortcuts around the CMake build in ./build.
# Variables can be overridden: make build BUILD_TYPE=RelWithDebInfo ASAN=OFF

BUILD_DIR  ?= build
BUILD_TYPE ?= Debug
ASAN       ?= ON
JOBS       ?= $(shell nproc)

# make test T=gurka_truck F='*Fuel*'   |   make test T=loki_service
T ?= gurka_truck
F ?= *
# make logcheck FILE=src/sif/truckcost.cc
FILE ?= src/sif/truckcost.cc
# make run ACTION=route REQ=req.json CONFIG=valhalla.json
CONFIG ?= valhalla.json
ACTION ?= route
REQ    ?=

# npm package: separate Release build without ASan, everything it writes stays under NODE_BUILD_DIR
NODE_BUILD_DIR ?= build-node
NODE_ARCH      := $(if $(filter aarch64 arm64,$(shell uname -m)),arm64,x64)
NODE_OS        := $(if $(filter Darwin,$(shell uname -s)),darwin,linux)
NODE_SRC       := src/bindings/nodejs
NPM_VENV       := $(NODE_BUILD_DIR)/venv
NPM_DIST       := $(NODE_BUILD_DIR)/nodejs_dist
NPM_PKG        := $(NODE_BUILD_DIR)/valhalla-npm-package
NPM_EXES       := valhalla_build_tiles valhalla_build_admins valhalla_add_predicted_traffic \
                  valhalla_add_landmarks valhalla_ingest_transit valhalla_convert_transit
NPM_SCRIPTS    := valhalla_build_config valhalla_build_elevation valhalla_build_extract \
                  valhalla_build_timezones valhalla_get_elevation
NPM_VERSION    ?= $(shell sed -n 's/^\#define VALHALLA_VERSION_\(MAJOR\|MINOR\|PATCH\) \([0-9]*\)/\2/p' \
                    valhalla/valhalla.h | paste -sd.)

CMAKE_BUILD = cmake --build $(BUILD_DIR) -j$(JOBS)
TEST_BIN = $(if $(filter gurka_%,$(T)),test/gurka/$(T),test/$(T))

.DEFAULT_GOAL := help
.PHONY: help configure reconfigure build lib service tools test truck fuel loki gurka \
        tests_all format logcheck run clean npm-build npm-bundle npm

help: ## Show this help
	@grep -hE '^[a-zA-Z_-]+:.*## ' $(MAKEFILE_LIST) | \
	  awk 'BEGIN {FS = ":.*## "}; {printf "  \033[36m%-12s\033[0m %s\n", $$1, $$2}'
	@echo
	@echo "  Vars: BUILD_TYPE=$(BUILD_TYPE) ASAN=$(ASAN) JOBS=$(JOBS) T=$(T) F='$(F)'"

configure: ## Run cmake configure (BUILD_TYPE, ASAN)
	cmake -S . -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
	  -DENABLE_ADDRESS_SANITIZER=$(ASAN) -DENABLE_CCACHE=ON \
	  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

reconfigure: ## Wipe CMakeCache and configure again
	rm -f $(BUILD_DIR)/CMakeCache.txt
	$(MAKE) configure

build: ## Build everything
	$(CMAKE_BUILD)

lib: ## Build only libvalhalla
	$(CMAKE_BUILD) --target valhalla

service: ## Build valhalla_service
	$(CMAKE_BUILD) --target valhalla_service

tools: ## Build tile-building tools (build_tiles, build_admins, ...)
	$(CMAKE_BUILD) --target valhalla_build_tiles valhalla_build_admins \
	  valhalla_build_timezones valhalla_build_extract valhalla_service

test: ## Build and run one test: T=<target> F=<gtest filter>
	$(CMAKE_BUILD) --target $(T)
	cd $(BUILD_DIR) && ./$(TEST_BIN) --gtest_filter='$(F)'

truck: ## gurka_truck, all tests
	$(MAKE) test T=gurka_truck

fuel: ## gurka_truck, only fuel tests
	$(MAKE) test T=gurka_truck F='*Fuel*'

loki: ## loki_service unit test
	$(MAKE) test T=loki_service

gurka: ## All gurka integration tests (slow)
	$(CMAKE_BUILD) --target run-gurka

tests_all: ## All unit + gurka tests, keeps going past failures (very slow)
	$(CMAKE_BUILD) --target check run-gurka -- -k

format: ## clang-format + ruff over the repo
	./scripts/format.sh

logcheck: ## Compile FILE with LOGGING_LEVEL_ALL to type-check LOG_DEBUG/TRACE
	@python3 -c 'import json,os,shlex,subprocess,sys; \
	  f=os.path.abspath(sys.argv[2]); \
	  e=[c for c in json.load(open(sys.argv[1])) if os.path.abspath(c["file"])==f]; \
	  e or sys.exit("not in compile_commands: "+f); \
	  a=shlex.split(e[0]["command"]); i=a.index("-o"); \
	  a[i:i+2]=["-DLOGGING_LEVEL_ALL","-o","/dev/null"]; \
	  sys.exit(subprocess.call(a,cwd=e[0]["directory"]))' \
	  $(BUILD_DIR)/compile_commands.json $(FILE) && echo "logcheck OK: $(FILE)"

run: ## One-shot valhalla_service: ACTION=route REQ=<json or file> CONFIG=<config>
	$(BUILD_DIR)/valhalla_service $(CONFIG) $(ACTION) '$(REQ)'

npm-build: ## Build Node.js addon + bundled tools in NODE_BUILD_DIR (Release, no ASan)
	cmake -S . -B $(NODE_BUILD_DIR) -DCMAKE_BUILD_TYPE=Release \
	  -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DENABLE_NODE_BINDINGS=ON \
	  -DENABLE_SERVICES=OFF -DENABLE_TESTS=OFF -DENABLE_SINGLE_FILES_WERROR=OFF \
	  -DENABLE_CCACHE=ON
	cmake --build $(NODE_BUILD_DIR) -j$(JOBS) --target valhalla_node $(NPM_EXES)

npm-bundle: npm-build ## Copy addon + tools with their shared libs into NPM_DIST
	test -x $(NPM_VENV)/bin/python || python3 -m venv $(NPM_VENV)
	$(NPM_VENV)/bin/pip install -q macholib==1.16.3
	rm -rf $(NPM_DIST)
	$(NPM_VENV)/bin/python $(NODE_SRC)/scripts/bundle.py \
	  $(NODE_BUILD_DIR)/$(NODE_SRC)/lib/valhalla_node.node \
	  $(addprefix $(NODE_BUILD_DIR)/,$(NPM_EXES)) $(NPM_DIST)

npm: npm-bundle ## Build the npm .tgz for the current platform into NPM_PKG
	rm -rf $(NPM_PKG)
	mkdir -p $(NPM_PKG)/$(NODE_OS)/$(NODE_ARCH)
	cp $(addprefix $(NODE_SRC)/,index.js index.mjs index.d.ts package.json README.md .npmignore) $(NPM_PKG)/
	cp -rp $(NODE_SRC)/lib $(NODE_SRC)/bin proto $(NPM_PKG)/
	cp -p $(addprefix scripts/,$(NPM_SCRIPTS)) $(NPM_PKG)/
	cp -rp $(NPM_DIST)/. $(NPM_PKG)/$(NODE_OS)/$(NODE_ARCH)/
	cd $(NPM_PKG) && npm version $(NPM_VERSION) --no-git-tag-version --allow-same-version && npm pack
	@echo "npm package: $$(ls $(NPM_PKG)/*.tgz)"

clean: ## cmake clean of BUILD_DIR + remove NODE_BUILD_DIR (npm build, bundle, package)
	-$(CMAKE_BUILD) --target clean
	rm -rf $(NODE_BUILD_DIR)
