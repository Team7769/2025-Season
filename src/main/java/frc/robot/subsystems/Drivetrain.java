package frc.robot.subsystems;

import java.util.function.Supplier;

import javax.xml.datatype.XMLGregorianCalendar;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.enums.DrivetrainState;
import frc.robot.enums.LocationTarget;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
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
import frc.robot.TunerConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.FieldConstants;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.utilities.GeometryUtil;
import frc.robot.utilities.OneDimensionalLookup;
import frc.robot.utilities.VisionMeasurement;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class Drivetrain extends CommandSwerveDrivetrain implements IDrivetrain {
    private ChassisSpeeds followChassisSpeeds = new ChassisSpeeds(0, 0, 0);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(DrivetrainConstants.kSpeedAt12VoltsMps * 0.05).withRotationalDeadband(DrivetrainConstants.MaxAngularRate * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle toPoint = new SwerveRequest.FieldCentricFacingAngle();
    public final SwerveRequest idle = new SwerveRequest.Idle();
    
    public final SwerveRequest.ApplyRobotSpeeds chassisDrive = new SwerveRequest.ApplyRobotSpeeds();

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("Robot Pose", Pose2d.struct).publish();
    private final Field2d m_field;

    private final PeriodicIO periodicIO = new PeriodicIO();
    private boolean hasAppliedOperatorPerspective = false;
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

    private double followP = 4;
    // 0 = right 1 = algae 2 = left
    private int reefTarget = 0;
    private int targetReefFace = 0;

    private DrivetrainState _currentState = DrivetrainState.OPEN_LOOP;
    private DrivetrainState _previousState = DrivetrainState.IDLE;
    private LocationTarget _currentTarget = LocationTarget.NONE;
    private Pose2d _target = new Pose2d();
    private boolean _isFollowingFront = false;
    private double targetRotation;
    private double xFollow;
    private double yFollow;

    private static class PeriodicIO {
        double VxCmd;
        double VyCmd;
        double WzCmd;
    }

    private final CommandXboxController _driverController;
    private final Vision _vision;

    public Drivetrain(CommandXboxController driveController , Vision vision ) {
        super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);

        // AutoBuilder.configureHolonomic(this::getPose, this::setStartingPose,
        // this::getSpeeds, (speeds) ->
        // this.setControl(chassisDrive.withSpeeds(speeds)),
        // Constants.DrivetrainConstants.pathFollowerConfig,
        // GeometryUtil::isRedAlliance, this);

        _driverController = driveController;
        _vision = vision;
        m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);
    }

    private void setChassisSpeeds(ChassisSpeeds newSpeeds) {
        followChassisSpeeds = newSpeeds;
    }

    private void setStartingPose(Pose2d startingPose) {
        this.resetPose(startingPose);
    }

    private double getAngleToTarget(Translation2d translation) {
        return GeometryUtil.getAngleToTarget(
            translation, this::getPose, _isFollowingFront
        );
    }

    private double getDistanceToTarget(Translation2d translation) {
        return GeometryUtil.getDistanceToTarget(
            translation, this::getPose
        );
    }

    public Pose2d getPose() {
        return getStateCopy().Pose;
    }

    public InstantCommand resetGyro() {
        return new InstantCommand(() -> {
            getPigeon2().setYaw(0);
            resetRotation(new Rotation2d());
        });
    }

    public double getPoseX() {
        return getPose().getTranslation().getX();
    }

    public double getPoseY() {
        return getPose().getTranslation().getY();
    }

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

    private double getDegrees() {
        return getPigeon2().getRotation2d().getDegrees();
    }

    public LocationTarget getCurrentTarget() {
        return _currentTarget;
    }

    public int targetedPole() {
        return reefTarget;
    }

    public void setReefTargetSideRight(int target) {
        reefTarget = target;
    }

    public void setReefTargetFace(int face) {
        targetReefFace = face;
    }

    public void targetNextReefFace() {
        if (targetReefFace < 5) {
            targetReefFace++;
        } else {
            targetReefFace = 0;
        }
    }
    
    private void updateOdometry() {
        ArrayList<VisionMeasurement> visionMeasurements = _vision
            .getVisionMeasurements(
            getPigeon2().getRotation2d()
        );

        for (VisionMeasurement visionMeasurement : visionMeasurements) {
            this.addVisionMeasurement(
                visionMeasurement.pose, Utils.fpgaToCurrentTime(visionMeasurement.timestamp)
            );
        }
        publisher.set(getPose());
        m_field.setRobotPose(getPose());        
        SmartDashboard.putNumber("target rotation", targetRotation);
        SmartDashboard.putNumber("pose x", getPoseX());
        SmartDashboard.putNumber("pose y", getPoseY());
        SmartDashboard.putNumber("angle", getDegrees());
        SmartDashboard.putNumber("followP", followP);
        SmartDashboard.putNumber("reefTarget", reefTarget);
        SmartDashboard.putNumber("reefFace", targetReefFace);
        SmartDashboard.putNumber("target angle", _target.getRotation().getDegrees());
        SmartDashboard.putNumber("Target X", _target.getX());
        SmartDashboard.putNumber("Target Y", _target.getY());
    }

    //#region periodic
    @Override
    public void periodic() {
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
        DriverStation.getAlliance().ifPresent((allianceColor) ->
        {this.setOperatorPerspectiveForward(allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation: BlueAlliancePerspectiveRotation);
        hasAppliedOperatorPerspective = true;
        });
    }

    _vision.updateLimelightPosition(getPigeon2().getRotation2d());
    
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

        switch (_currentTarget) {
            case CORAL_SOURCE:
                targetSource(GeometryUtil::isRedAlliance);
                targetRotation = GeometryUtil.getRotationDifference(this::getPose, _target.getRotation().getDegrees()) / 50;
            break;
            case PROCESSOR:
                if (GeometryUtil.isRedAlliance()) {
                    targetRotation = GeometryUtil.getRotationDifference(this::getPose, 90) / 50;
                } else {
                    targetRotation = GeometryUtil.getRotationDifference(this::getPose, 270) / 50;
                }
            break;
            case CAGE:
                targetCage();
                targetRotation = GeometryUtil.getRotationDifference(this::getPose, _target.getRotation().getDegrees()) / 50;
            break;
            case REEF:
                targetReef(GeometryUtil::isRedAlliance);
                targetRotation = GeometryUtil.getRotationDifference(this::getPose, _target.getRotation().getDegrees()) / 50;
            break;
            case NONE:
            break;
            default:
                targetRotation = GeometryUtil.getAngleToTarget(_target.getTranslation(), this::getPose, _isFollowingFront) / 50;
            break;
        }

        SmartDashboard.putNumber("XDiff", GeometryUtil.getXDifference(_target, this::getPose));
        SmartDashboard.putNumber("YDiff", GeometryUtil.getYDifference(_target, this::getPose));
        SmartDashboard.putString("currentTarget", getCurrentTarget().name());

        if (Math.abs(GeometryUtil.getXDifference(_target, this::getPose)) < 1 && Math.abs(GeometryUtil.getYDifference(_target, this::getPose)) < 1) {
            followP = .9;
        } else {
            followP = 4;
        }

        xFollow = GeometryUtil.getXDifference(_target, this::getPose) / followP;
        yFollow = GeometryUtil.getYDifference(_target, this::getPose) / followP;

        if (yFollow > 1) {
            yFollow = 1;
        } 
        if (xFollow > 1) {
            xFollow = 1;
        }

        if (Math.abs(targetRotation) >= .5) {
            if (targetRotation > 0) {
                targetRotation = .5;
            } else {
                targetRotation = -0.5;
            }
        }
        updateOdometry();
        handleCurrentState().schedule();
    }

    //#region State logic
    private Command handleCurrentState() {
        switch (_currentState) {
            case IDLE:
                return applyRequest(() -> idle);
            case OPEN_LOOP:
                return applyRequest(() -> drive.withVelocityX(this.periodicIO.VxCmd *
                DrivetrainConstants.kSpeedAt12VoltsMps).withVelocityY(this.periodicIO.VyCmd *
                DrivetrainConstants.kSpeedAt12VoltsMps).withRotationalRate(this.periodicIO.WzCmd *
                DrivetrainConstants.MaxAngularRate));
            case ROTATION_FOLLOW:
                return applyRequest(() -> drive.withVelocityX(this.periodicIO.VxCmd *
                DrivetrainConstants.kSpeedAt12VoltsMps).withVelocityY(this.periodicIO.VyCmd *
                DrivetrainConstants.kSpeedAt12VoltsMps).withRotationalRate(targetRotation * 
                DrivetrainConstants.MaxAngularRate));
            case POINT_FOLLOW:
                return applyRequest(() -> drive.withVelocityX(-xFollow *
                DrivetrainConstants.kSpeedAt12VoltsMps).withVelocityY(-yFollow *
                DrivetrainConstants.kSpeedAt12VoltsMps).withRotationalRate(targetRotation * 
                DrivetrainConstants.MaxAngularRate));
            default:
                return applyRequest(() -> idle);
        }
    }

    @Override
    public InstantCommand setWantedState(DrivetrainState state) {
        return new InstantCommand(() -> {
            if (state != _currentState) {
                _previousState = _currentState;
                _currentState = state;
            }
        }, this);
    }

    public InstantCommand setWantedTarget(LocationTarget newTarget) {
        return new InstantCommand(() ->{
            if (newTarget != _currentTarget) {
                _currentTarget = newTarget;
            }
        }, this);
    }
    ////#endregion

    //#region setTargetFunctions
    public void targetSource(Supplier<Boolean> isRedAlliance) {
        Pose2d[] _sourcePositionArray;
        
        _isFollowingFront = false;
        if (getPose().getTranslation().getY() > FieldConstants.kHalfFieldWidth) {
             _sourcePositionArray = isRedAlliance.get() ? Constants.FieldConstants.kRedSourceTop : Constants.FieldConstants.kBlueSourceTop;
        } else {
            _sourcePositionArray = isRedAlliance.get() ? Constants.FieldConstants.kRedSourceBottom : Constants.FieldConstants.kBlueSourceBottom;
        }

        Pose2d closestPoint = new Pose2d();
        Translation2d currentPose = getPose().getTranslation();
        for(Pose2d position : _sourcePositionArray)
        {
            var positionDistance = position.getTranslation().getDistance(currentPose);
            var closestPointDistance = closestPoint.getTranslation().getDistance(currentPose);
            if(positionDistance < closestPointDistance)
            {
                closestPoint = position;
            }
        }
        _target = closestPoint;
    }

    public void setTargetProcessor(Supplier<Boolean> isRedAlliance)
    {
        _currentTarget = LocationTarget.PROCESSOR;
        _isFollowingFront = false;
    }

    public void setTargetBarge(Supplier<Boolean> isRedAlliance)
    {
        _currentTarget = LocationTarget.BARGE;
    }

    public void targetCage()
    {
        _target = FieldConstants.kCage;
    }

    public void targetReef(Supplier<Boolean> isRedAlliance) {
        if (isRedAlliance.get()) {
            _target = FieldConstants.kRedReefFaces[targetReefFace].getAllPoles()[reefTarget];
        } else {
            _target = FieldConstants.kBlueReefFaces[targetReefFace].getAllPoles()[reefTarget];
        }
    }

    ////#endregion
}
