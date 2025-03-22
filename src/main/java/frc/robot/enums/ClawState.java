package frc.robot.enums;

import frc.robot.statemachine.IState;

public enum ClawState implements IState {
    IDLE,
    IDLE_WITH_ALGAE,
    FLOOR_INTAKE,
    DEALGIFY,
    PREP_NET,
    PREP_PROCESSOR,
    SCORE,
    TARGET,
    PREP_CLIMB,
    NONE
}
