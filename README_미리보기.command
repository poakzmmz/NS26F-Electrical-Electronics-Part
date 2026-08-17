#!/bin/zsh

set -e
project_dir="${0:A:h}"
cd "$project_dir"
python3 tools/readme_preview.py

