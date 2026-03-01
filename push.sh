#!/bin/bash
# Usage:
#   bash push.sh                        # default commit message
#   bash push.sh "my commit message"    # custom message

set -e

NEST_DIR="C:/pc/3_code/opennest/nest"
COMPAS_NEST_DIR="C:/pc/3_code/opennest/compas_nest"
NEST_PINVOKE_DIR="C:/pc/3_code/opennest/nest_pinvoke"
OPENNEST_DIR="C:/pc/3_code/opennest/OpenNest"

NEST_REMOTE="https://github.com/petrasvestartas/nest.git"
SUBMODULE_PATH="external/nest"
COMMIT_MSG="${1:-Update nest solver}"

# ── 1. Push nest ──────────────────────────────────────────────────────────────
echo "==> [nest] pushing..."
cd "$NEST_DIR"
git add -A
if ! git diff --cached --quiet; then
  git commit -m "$COMMIT_MSG"
fi
git push
NEST_COMMIT=$(git rev-parse --short HEAD)
echo "    HEAD: $NEST_COMMIT"

# ── 2. Update submodule in a parent repo ──────────────────────────────────────
update_submodule() {
  local REPO_DIR="$1"
  echo ""
  echo "==> [$REPO_DIR] updating submodule..."
  cd "$REPO_DIR"

  if [ -d "$SUBMODULE_PATH" ]; then
    # Already cloned — update pointer to latest remote commit
    git submodule update --remote "$SUBMODULE_PATH"
  else
    # Not cloned yet — add (handles both fresh and stale .gitmodules entries)
    git submodule add --force "$NEST_REMOTE" "$SUBMODULE_PATH"
  fi

  git add "$SUBMODULE_PATH" .gitmodules
  if ! git diff --cached --quiet; then
    git commit -m "Update nest submodule to $NEST_COMMIT"
    git push
  else
    echo "    Already up to date."
  fi
}

# ── 3. Update each parent repo ────────────────────────────────────────────────
update_submodule "$COMPAS_NEST_DIR"
update_submodule "$NEST_PINVOKE_DIR"
update_submodule "$OPENNEST_DIR"

echo ""
echo "Done."
