"""Mesh Flow Phase 8 — rename rollout mechanism (§2, decision #6).

The mechanism is buildable now; the actual STRING replacement at the rename-
audit's file:line locations (convertmdfiles/MeshFlow_Phase0_Rename_Audit.md)
is deliberately NOT applied yet — see the note at the bottom of this module.

Safe default: stored/loaded identifiers never change under LABEL_ONLY, so this
module introduces zero risk of breaking an existing saved project just by
existing, regardless of whether decision #6 has been answered.
"""
from __future__ import annotations
from enum import Enum

from app.meshflow.open_decisions import OpenDecisionBlocked


class RenamePolicy(Enum):
    LABEL_ONLY = 'label_only'       # SAFE DEFAULT, needs no decision: stored/loaded identifiers
                                      # never change; only the UI-displayed label would change.
    MIGRATE = 'migrate'             # rewrite old identifiers to new on load — requires decision #6
    DUAL_WRITE = 'dual_write'       # new code writes both old and new strings — requires decision #6
    VERSION_FLAG = 'version_flag'   # saved files carry a schema version — requires decision #6


ACTIVE_RENAME_POLICY: RenamePolicy = RenamePolicy.LABEL_ONLY   # safe until a human changes it

_LEGACY_PATH_MODE_ALIASES: dict[str, str] = {'Mesh Surface': 'Mesh Flow'}
_CANONICAL_PATH_MODE_ALIASES: dict[str, str] = {v: k for k, v in _LEGACY_PATH_MODE_ALIASES.items()}


def resolve_path_mode_for_load(stored_value: str) -> str:
    """Used when loading a saved project / CSV / JSON export's path_mode field
    back in. ALWAYS succeeds for any recognized current identifier ('Mesh
    Surface' or 'Mesh Flow') — under the default LABEL_ONLY policy this is a
    pure passthrough, so no old project is ever broken by this module
    existing. OpenDecisionBlocked(6, ...) is raised ONLY if ACTIVE_RENAME_POLICY
    is explicitly set to MIGRATE/DUAL_WRITE/VERSION_FLAG by a human (decision
    #6 actively answered) AND that policy's specific mechanism isn't finished
    yet — the exception protects an in-progress migration a human chose to
    start, it never protects against the decision simply not having been made.
    """
    if ACTIVE_RENAME_POLICY is RenamePolicy.LABEL_ONLY:
        return stored_value
    if ACTIVE_RENAME_POLICY is RenamePolicy.MIGRATE:
        return _LEGACY_PATH_MODE_ALIASES.get(stored_value, stored_value)
    if ACTIVE_RENAME_POLICY is RenamePolicy.DUAL_WRITE:
        return stored_value   # both strings accepted as-is; writer side duplicates fields
    if ACTIVE_RENAME_POLICY is RenamePolicy.VERSION_FLAG:
        raise OpenDecisionBlocked(6, "VERSION_FLAG policy requires a schema_version field design")
    raise AssertionError(f"unhandled RenamePolicy: {ACTIVE_RENAME_POLICY}")


def display_label_for(internal_mode: str) -> str:
    """The ONLY function the rename-audit's call sites should migrate to
    calling, one at a time, instead of hardcoding 'Mesh Surface' literals.
    Works under every policy, including the default.

    NOT YET WIRED UP: see the note at the bottom of this module for why the
    rename-audit's actual call sites are not migrated to this function yet.
    """
    return internal_mode


# ---------------------------------------------------------------------------
# Why the rename-audit's string replacements are NOT applied in this phase,
# even though the mechanism above is complete and safe to use:
#
# Mesh Flow (Phases 3-7) is still a separate, beta, opt-in mode that COEXISTS
# with Mesh Surface — it is not yet the production replacement the spec's
# rename describes. Applying step 1-2 of the rollout mechanics now (swapping
# every hardcoded 'Mesh Surface' label to show 'Mesh Flow') would make
# display_label_for('Mesh Surface') == 'Mesh Flow', which would put TWO
# UI entries both reading "Mesh Flow" — the stable mode (mislabeled) and the
# new beta radio (correctly labeled 'Mesh Flow (beta)') — actively confusing
# users rather than clarifying anything. The rename is only correct to apply
# once Mesh Flow is mature enough to actually take over Mesh Surface's
# identity (roadmap Phases 3-7 validated, decision #6 answered) — until then,
# display_label_for is an identity function and the rename-audit's call sites
# keep their literal 'Mesh Surface' strings unchanged.
# ---------------------------------------------------------------------------
