#!/bin/bash
# push.sh — push nest repo to GitHub, then update it as a submodule
# in compas_nest, nest_pinvoke, and OpenNest.
#
# Usage:
#   ./push.sh                          # uses default commit message
#   ./push.sh "my commit message"      # custom commit message

set -e

NEST_DIR="C:/pc/3_code/opennest/nest"
COMPAS_NEST_DIR="C:/pc/3_code/opennest/compas_nest"
NEST_PINVOKE_DIR="C:/pc/3_code/opennest/nest_pinvoke"
OPENNEST_DIR="C:/pc/3_code/opennest/OpenNest"

NEST_REMOTE="https://github.com/petrasvestartas/nest.git"
SUBMODULE_PATH="external/nest"

COMMIT_MSG="${1:-Update nest solver}"

# ── 1. Push the nest repo ─────────────────────────────────────────────────────
echo "==> [nest] Staging and pushing..."
cd "$NEST_DIR"

git add -A

if git diff --cached --quiet; then
  echo "    Nothing to commit in nest — skipping commit."
else
  git commit -m "$COMMIT_MSG"
fi

git push

NEST_COMMIT=$(git rev-parse HEAD)
echo "    HEAD: $NEST_COMMIT"

# ── 2. Helper: add or update submodule in a parent repo ──────────────────────
update_submodule() {
  local REPO_DIR="$1"
  echo ""
  echo "==> [$REPO_DIR] Updating submodule..."
  cd "$REPO_DIR"

  if [ -f ".gitmodules" ] && grep -q "$SUBMODULE_PATH" .gitmodules 2>/dev/null; then
    # Already registered — init if needed, then pull latest remote commit
    git submodule update --init --remote "$SUBMODULE_PATH"
    git add "$SUBMODULE_PATH"
  else
    # First time — register and initialise
    git submodule add "$NEST_REMOTE" "$SUBMODULE_PATH"
    git add "$SUBMODULE_PATH" .gitmodules
  fi

  if git diff --cached --quiet; then
    echo "    Submodule already up to date."
  else
    git commit -m "Update nest submodule to ${NEST_COMMIT:0:7}"
    git push
  fi
}

# ── 3. Update each parent repo ───────────────────────────────────────────────
update_submodule "$COMPAS_NEST_DIR"
update_submodule "$NEST_PINVOKE_DIR"
update_submodule "$OPENNEST_DIR"

echo ""
echo "All done."
