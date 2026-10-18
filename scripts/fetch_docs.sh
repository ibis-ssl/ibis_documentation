#!/usr/bin/env bash
# ibis-ssl の各リポジトリから文書を取得して docs/ 配下に置く（CI とローカルプレビューで共用）
set -euo pipefail

cd "$(dirname "$0")/.."

# <リポジトリ> <ブランチ> <取得するディレクトリ（. はリポジトリ全体）> <配置先>
SOURCES='
specs                     main    .     docs/specs
crane                     develop docs  docs/others/crane
G474_Orion_main           main    doc   docs/others/G474_Orion_main
Orion_CM4                 main    doc   docs/others/Orion_CM4
4WS_MainFW                main    doc   docs/others/4WS_MainFW
foxglove_crane_visualizer main    docs  docs/others/foxglove_crane_visualizer
F303_boost                main    doc   docs/others/F303_boost
Orion_F303_sub            main    doc   docs/others/Orion_F303_sub
Orion_F303_BLDC           main    doc   docs/others/Orion_F303_BLDC
SwerveControllerF303      main    doc   docs/others/SwerveControllerF303
'

tmp_root=$(mktemp -d)
trap 'rm -rf "$tmp_root"' EXIT

while read -r repo branch src dest; do
  [ -z "$repo" ] && continue
  echo "fetch: ibis-ssl/$repo ($branch) $src -> $dest"
  clone="$tmp_root/$repo"
  if [ "$src" = "." ]; then
    git clone --quiet --depth 1 --branch "$branch" "https://github.com/ibis-ssl/$repo" "$clone"
    rm -rf "$clone/.git" "$clone/.github"
    src_dir="$clone"
  else
    git clone --quiet --depth 1 --branch "$branch" --filter=blob:none --sparse "https://github.com/ibis-ssl/$repo" "$clone"
    git -C "$clone" sparse-checkout set "$src"
    src_dir="$clone/$src"
  fi
  rm -rf "$dest"
  mkdir -p "$(dirname "$dest")"
  mv "$src_dir" "$dest"
done <<< "$SOURCES"
