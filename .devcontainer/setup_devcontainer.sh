#!/usr/bin/env bash
set -euo pipefail

# ---------- CONFIG ----------
INPUT_FILE=".devcontainer/devcontainer.base.json"
OUTPUT_FILE=".devcontainer/devcontainer.json"
# --------------------------------

# Ensure jq is installed
if ! command -v jq >/dev/null 2>&1; then
    echo "jq is required but not installed. Installing it now..."
    sudo apt-get install -y jq
fi

# Detect CPU architecture (e.g., x86_64 or aarch64)
detect_cpu_arch() {
    local res=$(uname -m)
    echo "$res"
}

# Detect presence of an NVIDIA GPU
has_nvidia_gpu() {
    # TODO: run nvidia-smi and check exit status / output
    # Return 0 if GPU found, 1 otherwise
    if command -v nvidia-smi >/dev/null 2>&1 ; then
        echo "true"
    else
        echo "false"
    fi
}

# Adjust the devcontainer JSON based on arch and GPU presence
adjust_devcontainer() {
    local arch="$1"
    local gpu="$2"

    # Dockerfile mapping
    declare -A dockerfiles=(
        ["x86_64"]="../Dockerfile.amd64"
        ["aarch64"]="../Dockerfile.arm64"
    )

    # Verify we have a mapping for the detected architecture
    if [[ -z "${dockerfiles[$arch]:-}" ]]; then
        echo "Unsupported architecture: $arch"
        exit 1
    fi

    # Build the jq filter
    jq \
        --arg dockerfile "${dockerfiles[$arch]}" \
        --argjson gpu "$gpu" \
        '
        .build.dockerfile = $dockerfile
        |
        if $gpu then
            .runArgs = ( .runArgs // [] ) + ["--gpus=all"]
        else
            .
        end
        ' "$INPUT_FILE" > "$OUTPUT_FILE"
}


main() {
    echo $(pwd)
    if [[ ! -f "$INPUT_FILE" ]]; then
        echo "Input file $INPUT_FILE not found."
        exit 1
    fi

    arch=$(detect_cpu_arch)
    if [[ -z "$arch" ]]; then
        echo "Failed to detect CPU architecture."
        exit 1
    fi

    gpu=$(has_nvidia_gpu)
    echo "$gpu"
    adjust_devcontainer "$arch" "$gpu"

    echo "Generated $OUTPUT_FILE (arch=$arch, gpu=$gpu)"
}

main "$@"
