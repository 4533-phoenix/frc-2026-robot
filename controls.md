# Controls Reference

This document describes how to operate the robot during a match.
All controls are on the **driver Xbox controller** (port 0).
There is currently no operator controller binding.

## Match Mode Toggle

A **"Match Mode"** boolean toggle is available on the SmartDashboard / Shuffleboard. It defaults to **off**.

- **Off (default)** — The robot stays idle until the driver explicitly holds LT. No automatic flywheel spin-up or tracking. Scoring still works normally via LT/RT. Use this for practice, pit testing, and demos.
- **On** — Full automatic behavior is enabled: flywheel pre-spin, automatic tracking when in a scoring zone, and outpost approach warming. Turn this on for real matches.

## Controller Layout

### Left Stick — Drive

- **Up/Down** — Forward / Backward (field-relative)
- **Left/Right** — Strafe left / right (field-relative)

### Right Stick — Rotate

- **Left/Right** — Rotate the robot (overridden during auto-aim)

### Triggers — Scoring

- **Left Trigger (LT, hold)** — Aim at the target. In the shooting zone (with outpost active), the robot aims at the outpost. In the lobbing zone, it aims at the nearest lobbing target. The robot locks its rotation toward the target while you keep moving with the left stick.
- **Right Trigger (RT, hold while aiming)** — Fire. Only takes effect when LT is also held and the robot reports ready (heading aligned, flywheel at speed). The indexer feeds the game piece.

The typical scoring sequence is the same for both outpost shots and lobs:

1. Drive into the shooting zone or lobbing zone.
2. Hold **LT** to aim.
3. Feel the right-side rumble (robot is aligned and ready).
4. Hold **RT** to fire.
5. Release both triggers when done.

Lobbing uses the same controls as outpost scoring. The robot automatically detects which zone you are in and adjusts the target, flywheel speed, and hood angle accordingly. Lobbing is always available in the lobbing zone regardless of outpost state.

### Bumpers — Intake Spinners

- **Left Bumper (LB, hold)** — Run intake spinners inward to collect game pieces.
- **Right Bumper (RB, hold)** — Run intake spinners outward to eject game pieces.

The intake arm stays deployed for the entire match. These buttons only control the rollers.

### D-Pad — Climb

- **D-Pad Up** — Raise the climb mechanism.
- **D-Pad Down** — Lower the climb mechanism.

Pressing either climb button is a one-way transition. Once climb is initiated, the intake arm retracts and all scoring mechanisms shut off for the remainder of the match.

### Face Buttons

- **B** — Reset gyro heading to zero (works even while disabled). Use this when the robot's forward direction drifts or after repositioning.

## Automatic Behavior (Match Mode Only)

The following automatic behaviors only activate when **Match Mode is on**. When match mode is off, the robot stays idle until you hold LT.

- **Flywheel pre-spin** — The flywheel spins at an idle speed when the outpost is about to activate (approximately 5 seconds before), reducing spin-up time.
- **Automatic tracking** — While in the shooting zone with the outpost active, the flywheel speed and hood angle continuously track the target even before LT is held.
## Always-On Behavior

These features work regardless of match mode:

- **Lobbing** — When in the lobbing zone, the robot automatically switches to lobbing mode. It aims at the nearest of two lobbing targets using a fixed flywheel speed and hood angle tuned for a high-arc trajectory. Lobbing is always available regardless of outpost state.
- **Lead compensation** — When aiming at the outpost, the target is offset based on the robot's current velocity so shots land correctly while moving. Lead compensation is not applied to lobs.
- **Intake arm position** — The arm is held deployed during normal play and retracts automatically when climbing is initiated.

## Rumble Feedback

The controller vibrates to communicate robot state:

| Feedback | Meaning |
|---|---|
| Brief left rumble | The outpost just became active (you can score now). |
| Sustained right rumble | Robot is aimed and ready to fire (pull RT). |
| Strong full rumble (0.5s) | Endgame — 30 seconds remaining in teleop. |

## Match Flow Summary

1. **Auto** — Runs the selected autonomous routine from the dashboard. The outpost is always active during auto.
2. **Teleop start** — Enable **Match Mode** on the dashboard. Drive around, collect game pieces with LB. The flywheel will pre-spin automatically when your outpost window is approaching.
3. **Scoring (outpost)** — Enter the shooting zone, hold LT to aim, wait for right rumble, hold RT to fire.
4. **Scoring (lobbing)** — Enter the lobbing zone, hold LT to aim at the nearest lob target, wait for right rumble, hold RT to lob.
5. **Endgame** — When you feel the full-controller rumble, prepare to climb. Press D-Pad Up/Down to climb. This is irreversible.

> **Tip:** For practice or pit demos, leave Match Mode off. You can still score by holding LT — the only difference is no automatic flywheel spin-up or tracking.
