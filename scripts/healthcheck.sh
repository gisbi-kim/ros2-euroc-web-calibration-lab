#!/usr/bin/env bash
set -euo pipefail
curl -fsS "http://127.0.0.1:${WEB_PORT:-8080}/health" >/dev/null
