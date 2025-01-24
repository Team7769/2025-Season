package frc.robot.enums;

import frc.robot.statemachine.IState;

public enum DrivetrainState implements IState  {
    IDLE,
    OPEN_LOOP,
    TRAJECTORY_FOLLOW,
    POINT_FOLLOW,
    TARGET_FOLLOW,
    FACE_FOWARD
}
