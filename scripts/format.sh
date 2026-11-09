#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 || ( "$1" != --check && "$1" != --write ) ]]; then
  echo "Usage: $0 --check|--write" >&2
  exit 2
fi
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
formatter=clang-format-14
if ! command -v "$formatter" >/dev/null 2>&1; then
  formatter=clang-format
fi
if ! command -v "$formatter" >/dev/null 2>&1 ||
   [[ ! $("$formatter" --version) =~ version[[:space:]]14\. ]]; then
  echo "clang-format 14 is required: sudo apt install clang-format-14" >&2
  exit 1
fi

files=()
# NUL-delimited paths preserve spaces and other special characters.
while IFS= read -r -d '' file; do
  [[ -f "$file" ]] || continue
  case "$file" in
    io/hikrobot/include/*|io/mindvision/include/*|io/serial/*|tasks/auto_aim/planner/tinympc/*)
      continue ;;
  esac
  case "$file" in
    src/*|calibration/*|tasks/auto_aim/*|tasks/omniperception/*|io/*|tools/*|tests/*|visualization/*)
      case "$file" in
        *.cpp|*.hpp|*.h|*.cc|*.c) files+=("$file") ;;
      esac ;;
  esac
done < <(git ls-files --cached --others --exclude-standard -z)

if [[ ${#files[@]} -eq 0 ]]; then
  echo "No maintained C/C++ files to format."
  exit 0
fi
if [[ "$1" == --check ]]; then
  "$formatter" --style=file --dry-run --Werror "${files[@]}"
else
  "$formatter" --style=file -i "${files[@]}"
fi
printf 'Format %s passed (%d files).\n' "$1" "${#files[@]}"
