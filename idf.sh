#!/bin/bash
set -e
activation_script="$HOME/.espressif/tools/activate_idf_v6.1.sh"
project_dir="$(cd -P -- "$(dirname -- "$0")" && pwd)"
exec /bin/bash -c '
    set -e
    source "$1" >/dev/null
    cd -- "$2"
    shift 2
    exec "$IDF_PYTHON_ENV_PATH/bin/python" "$IDF_PATH/tools/idf.py" "$@"
' bash "$activation_script" "$project_dir" "$@"
