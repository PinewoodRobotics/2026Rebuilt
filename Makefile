export PYTHONPATH := $(shell pwd)
ARGS ?=

VENV_PYTHON := .venv/bin/python

THRIFT_DIR = ThriftTsConfig/schema
THRIFT_ROOT_FILE = $(THRIFT_DIR)/config.thrift
PROTO_DIR = src/proto

GEN_DIR = src/backend/generated
PROTO_GEN_DIR = $(GEN_DIR)/proto
THRIFT_GEN_DIR = $(GEN_DIR)/thrift

THRIFT_TS_SCHEMA_GEN_DIR = $(THRIFT_GEN_DIR)/ts_schema
PROTO_PY_GEN_DIR = $(PROTO_GEN_DIR)/python

initialize:
	python3 -m venv .venv
	.venv/bin/pip install -r requirements.txt
	make test

test:
	PYTHONPATH=src pytest

generate-proto-python:
	mkdir -p $(PROTO_PY_GEN_DIR)
	protoc -I=$(PROTO_DIR) \
		--python_out=$(PROTO_PY_GEN_DIR) \
		--pyi_out=$(PROTO_PY_GEN_DIR) \
		$(shell find $(PROTO_DIR) -name "*.proto")
	.venv/bin/fix-protobuf-imports $(PROTO_PY_GEN_DIR)

thrift-to-py:
	mkdir -p $(THRIFT_GEN_DIR)
	thrift -r --gen py:type_hints,enum,package_prefix=backend.generated.thrift. \
		-I $(THRIFT_DIR) \
		-out $(THRIFT_GEN_DIR) \
		$(THRIFT_ROOT_FILE);

generate: generate-proto-python thrift-to-py

deploy-backend:
	PYTHONPATH="$(PWD)/src" $(VENV_PYTHON) -m backend.deploy