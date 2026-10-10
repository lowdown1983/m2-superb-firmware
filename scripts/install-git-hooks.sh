#!/usr/bin/env bash
# Install the repo's tracked git hooks into .git/hooks/.
# Re-run after a fresh clone: .git/hooks/ is not tracked by git, so the
# sim pre-commit gate would otherwise be silently absent.
# Usage: bash scripts/install-git-hooks.sh
set -euo pipefail

REPO_ROOT="$(git rev-parse --show-toplevel 2>/dev/null || true)"
if [[ -z "$REPO_ROOT" ]]; then
  echo "error: not inside a git repo" >&2
  exit 1
fi

HOOK_SRC="$REPO_ROOT/scripts/hooks/pre-commit"
HOOK_DST="$REPO_ROOT/.git/hooks/pre-commit"

if [[ ! -f "$HOOK_SRC" ]]; then
  echo "error: tracked hook not found: $HOOK_SRC" >&2
  exit 1
fi

ln -sf "$HOOK_SRC" "$HOOK_DST"
chmod +x "$HOOK_SRC"
echo "installed: $HOOK_DST -> $HOOK_SRC"

# Sanity: the installed hook must delegate to sim/ci.sh
if ! grep -q 'sim/ci.sh' "$HOOK_DST"; then
  echo "warning: installed hook does not reference sim/ci.sh" >&2
fi
