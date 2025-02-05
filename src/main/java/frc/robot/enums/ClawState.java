package frc.robot.enums;

import frc.robot.statemachine.IState;

public enum ClawState implements IState {
    IDLE,
    FLOOR_INTAKE,
    DEALGIFY,
    PREP_NET,
    PREP_PROCESSOR,
    SCORE
}
