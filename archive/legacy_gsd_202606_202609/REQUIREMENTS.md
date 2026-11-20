# Requirements

This file is the explicit capability and coverage contract for the project.

## Active

### R001 — Successful 5M step training run with zero target offset.
- Class: quality-attribute
- Status: active
- Description: Successful 5M step training run with zero target offset.
- Why it matters: Ensures the model doesn't swerve or behave poorly under full metric scaling.
- Source: .planning/REQUIREMENTS.md
- Primary owning slice: S01

### R002 — Achieve 6000+ steps per episode (full lap) with zero boundary resets.
- Class: core-capability
- Status: active
- Description: Achieve 6000+ steps per episode (full lap) with zero boundary resets.
- Why it matters: Proves the vision backbone is capable of mastering long-horizon navigation without failures.
- Source: .planning/REQUIREMENTS.md
- Primary owning slice: S01

## Validated

## Deferred

## Out of Scope

## Traceability

| ID | Class | Status | Primary owner | Supporting | Proof |
|---|---|---|---|---|---|
| R001 | quality-attribute | active | S01 | none | unmapped |
| R002 | core-capability | active | S01 | none | unmapped |

## Coverage Summary

- Active requirements: 2
- Mapped to slices: 2
- Validated: 0
- Unmapped active requirements: 0
