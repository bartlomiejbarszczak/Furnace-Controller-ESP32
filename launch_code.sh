#!/bin/bash
set -e

MODEL="minimax-m2.5:cloud"
#MODEL="gemma4:31b-cloud"

while getopts "m:" opt; do
  case $opt in
    m) MODEL="$OPTARG" ;;
    \?) echo "Usage: $0 -m <model>" >&2; exit 1 ;;
  esac
done

ANTHROPIC_AUTH_TOKEN=ollama ANTHROPIC_API_KEY="" ANTHROPIC_BASE_URL=http://localhost:11434 claude --model "$MODEL"