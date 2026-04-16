#!/bin/bash

# Build script for FNR26FACTORY Docker images with layer sharing
# This script builds base images first so all services can reuse their layers

set -e  # Exit on error

echo "======================================"
echo "Building FNR26FACTORY Docker Images"
echo "======================================"

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Build base image first (used by all services)
echo -e "${BLUE}[1/3] Building base image...${NC}"
docker build -f Dockerfile.base -t fnr26factory-base:latest .
echo -e "${GREEN}✓ Base image built${NC}\n"

# Build builder image (used for rf2o with compilation)
echo -e "${BLUE}[2/3] Building builder image...${NC}"
docker build -f Dockerfile.builder -t fnr26factory-builder:latest .
echo -e "${GREEN}✓ Builder image built${NC}\n"

# Build service images (all inherit from base/builder, so they're minimal)
echo -e "${BLUE}[3/3] Building service images...${NC}"
docker compose build
echo -e "${GREEN}✓ Service images built${NC}\n"

echo "======================================"
echo -e "${GREEN}All images built successfully!${NC}"
echo "======================================"
echo ""
echo "Image sizes summary:"
docker images | grep "fnr26factory"
