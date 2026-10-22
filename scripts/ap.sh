#!/usr/bin/env bash
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail
exec /bin/bash "$(dirname "${BASH_SOURCE[0]}")/start-go2.sh" --switch ap "$@"
