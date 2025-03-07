package frc.robot.enums;

import frc.robot.statemachine.IState;

public enum DrivetrainState implements IState  {
    IDLE,
    OPEN_LOOP,
    TRAJECTORY_FOLLOW,
    TARGET_FOLLOW,
    AUTO,
    NONE
}
