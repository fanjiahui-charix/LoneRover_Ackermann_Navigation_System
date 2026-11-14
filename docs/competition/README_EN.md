# Competition task chain

The vehicle navigates through QR task selection, reverse entry, channel tracking and return. [CompetitionSupervisor](../../src/hobot_navigation/hobot_nav/src/competition_supervisor.cpp) owns the mission state machine. [MotionActionArbiter](../../src/hobot_navigation/hobot_nav/src/motion_action_arbiter.cpp) coordinates Nav2 Action submission, cancellation and handover.

See the [competition guide](../../src/hobot_navigation/hobot_nav/README_COMPETITION_EN.md) for the launch command and [DEPENDENCIES.md](../DEPENDENCIES.md) for installation and build options. Use the organizer's competition documents for timing and penalty rules.

[中文](README.md)
