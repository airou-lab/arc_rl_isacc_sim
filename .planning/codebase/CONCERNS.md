# Codebase Concerns

**Analysis Date:** 2026-07-28

## Tech Debt

**Agent Node Monolith:**
- Issue: The main agent logic is extremely large (1,148 lines) and likely handles multiple responsibilities.
- Files: `arcproLab/policy_stack/agent/agent_node.py`
- Impact: Hard to maintain, test, and understand. Increases cognitive load and risk of regressions during modifications.
- Fix approach: Refactor into smaller, focused modules (e.g., separate perception, decision making, and control).

**Silent Failures / Empty Returns:**
- Issue: Several methods return `None` silently on failure or invalid input without logging or raising exceptions.
- Files: `arcproLab/mdp/track_manager.py`, `arcproLab/scripts/monitor_2m.py`, `arcproLab/scripts/watchdog.py`
- Impact: Difficult to debug issues when a track fails to load or a script fails, as the error is swallowed and propagates as a `NoneType` error downstream.
- Fix approach: Implement proper error logging or raise descriptive exceptions instead of returning `None`.

**Legacy Code Retention:**
- Issue: Large legacy files are still present in the repository, such as a 1,381-line test script.
- Files: `archive/sb3_legacy/policy_stack/test_all_so_far.py`, `archive/sb3_legacy/policy_stack/policies/hierarchical_policy.py`
- Impact: Clutters the codebase and may confuse developers about current vs. deprecated patterns.
- Fix approach: If no longer needed, remove the `archive/` directory entirely, relying on version control for history.

## Known Bugs

**Not detected**

## Security Considerations

**Not detected**

## Performance Bottlenecks

**Large Data Collection Scripts:**
- Problem: The data collection script is large and likely complex (933 lines), which might indicate a monolithic approach to simulation rollouts.
- Files: `arcproLab/policy_stack/baselines/dave2/collect.py`
- Cause: Handling too many aspects of the simulation and data recording in one file.
- Improvement path: Modularize data collection and use asynchronous or batched writes if performance is an issue.

## Fragile Areas

**Massive Test Files:**
- Files: `arcproLab/policy_stack/tests/test_intersection_flow.py`
- Why fragile: At 1,050 lines, this test file is likely overly complex, testing too many things or using excessive boilerplate.
- Safe modification: Difficult to modify without breaking unrelated tests if they share state or complex fixtures.
- Test coverage: Unclear, but large test files often indicate integration tests masquerading as unit tests.

## Scaling Limits

**Not detected**

## Dependencies at Risk

**Not detected**

## Missing Critical Features

**Not detected**

## Test Coverage Gaps

**Not detected**
