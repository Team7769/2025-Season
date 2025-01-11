package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.enums.DrivetrainState;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.utilities.GeometryUtil;
import frc.robot.utilities.OneDimensionalLookup;
import frc.robot.utilities.VisionMeasurement;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class Drivetrain implements IDrivetrain {
    private ChassisSpeeds followChassisSpeeds = new ChassisSpeeds(0, 0, 0);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public final SwerveRequest idle = new SwerveRequest.Idle();
    // this is used to drive with chassis speeds see an example of it in
    // setTrajectoryFollowModuleTargets
    public final SwerveRequest.ApplyRobotSpeeds chassisDrive = new SwerveRequest.ApplyRobotSpeeds();

    private final Field2d m_field = new Field2d();
    private final PeriodicIO periodicIO = new PeriodicIO();
    private boolean hasAppliedOperatorPerspective = false;
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

    private DrivetrainState _currentState = DrivetrainState.OPEN_LOOP;
    private DrivetrainState _previousState = DrivetrainState.IDLE;
    private Translation2d _target = new Translation2d();
    private boolean _isFollowingFront = false;
    private double targetRotation;

    private static class PeriodicIO {
        double VxCmd;
        double VyCmd;
        double WzCmd;
    }

    private final CommandXboxController _driverController;
    // private final Vision _vision;

    public Drivetrain(CommandXboxController driveController/* , Vision vision */) {
        // super(DrivetrainConstants.SwerveConstants, DrivetrainConstants.modules);

        // AutoBuilder.configureHolonomic(this::getPose, this::setStartingPose,
        // this::getSpeeds, (speeds) ->
        // this.setControl(chassisDrive.withSpeeds(speeds)),
        // Constants.DrivetrainConstants.pathFollowerConfig,
        // GeometryUtil::isRedAlliance, this);

        PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));

        _driverController = driveController;
        // _vision = vision;
    }

    private void setChassisSpeeds(ChassisSpeeds newSpeeds) {
        followChassisSpeeds = newSpeeds;
    }

    // private void setStartingPose(Pose2d startingPose) {
    // System.out.println("setStartingPose");
    // this.seedFieldRelative(startingPose);
    // }

    // private double getAngleToTarget(Translation2d translation) {
    //     return GeometryUtil.getAngleToTarget(
    //         translation, this::getPose, _isFollowingFront
    //     );
    // }

    // private double getDistanceToTarget(Translation2d translation) {
    //     return GeometryUtil.getDistanceToTarget(
    //         translation, this::getPose
    //     );
    // }

    // public Pose2d getPose()
    // return this.m_odometry.getEstimatedPosition();
    // }

    // public InstantCommand resetGyro() {
    // return new InstantCommand(() -> m_pigeon2.setYaw(0));
    // }

    private String getCurrentState() {
        return _currentState.name();
    }

    private String getPreviousState() {
        return _previousState.name();
    }

    private double getVxCmd() {
        return this.periodicIO.VxCmd;
    }

    private double getVyCmd() {
        return this.periodicIO.VyCmd;
    }

    private double getWzCmd() {
        return this.periodicIO.WzCmd;
    }

    // private double getDegrees() {
        // return this.m_pigeon2.getAngle();
    // }

    private void updateOdometry() {
        // m_field.setRobotPose(getPose());
    }

    public void periodic() {
    // if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
        // DriverStation.getAlliance().ifPresent((allianceColor) ->
        // {this.setOperatorPerspectiveForward(allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation: BlueAlliancePerspectiveRotation);
        // hasAppliedOperatorPerspective = true;
        // });
    // }

        this.periodicIO.VxCmd = -OneDimensionalLookup.interpLinear(
                Constants.DrivetrainConstants.XY_Axis_inputBreakpoints,
                Constants.DrivetrainConstants.XY_Axis_outputTable, _driverController.getLeftY());

        // The Y translation will be the horizontal value of the left driver joystick
        this.periodicIO.VyCmd = -OneDimensionalLookup.interpLinear(
                Constants.DrivetrainConstants.XY_Axis_inputBreakpoints,
                Constants.DrivetrainConstants.XY_Axis_outputTable, _driverController.getLeftX());

        // The rotation will be the horizontal value of the right driver joystick
        this.periodicIO.WzCmd = -OneDimensionalLookup.interpLinear(
                Constants.DrivetrainConstants.RotAxis_inputBreakpoints,
                Constants.DrivetrainConstants.RotAxis_outputTable, _driverController.getRightX());
        // targetRotation = GeometryUtil.getAngleToTarget(_target, this::getPose, _isFollowingFront) / 50;

        updateOdometry();
        handleCurrentState().schedule();
    }

    private Command handleCurrentState() {
        switch (_currentState) {
            // case IDLE:
                // return applyRequest(() -> idle);
            // case OPEN_LOOP:
                // return applyRequest(() -> drive.withVelocityX(this.periodicIO.VxCmd *
                // DrivetrainConstants.kSpeedAt12VoltsMps).withVelocityY(this.periodicIO.VyCmd *
                // DrivetrainConstants.kSpeedAt12VoltsMps).withRotationalRate(this.periodicIO.WzCmd *
                // DrivetrainConstants.MaxAngularRate));
            // case TARGET_FOLLOW:
                // return applyRequest(() -> drive.withVelocityX(this.periodicIO.VxCmd *
                // DrivetrainConstants.kSpeedAt12VoltsMps).withVelocityY(this.periodicIO.VyCmd *
                // DrivetrainConstants.kSpeedAt12VoltsMps).withRotationalRate(targetRotation * 
                // DrivetrainConstants.MaxAngularRate));
            // case TRAJECTORY_FOLLOW:
                // return applyRequest(() -> chassisDrive.withSpeeds(followChassisSpeeds));
            default:
                return new InstantCommand(() -> setWantedState(_currentState));
                // return applyRequesy(() -> idle);
        }
    }

    @Override
    public InstantCommand setWantedState(DrivetrainState state) {
        return new InstantCommand(() -> {
            System.out.print(state.name());
            if (state != _currentState) {
                _previousState = _currentState;
                _currentState = state;
            }
        }, this);
    }
}
