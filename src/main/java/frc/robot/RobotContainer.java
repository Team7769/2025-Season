// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatinatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ReefConstants;
import frc.robot.enums.CageState;
import frc.robot.enums.CalsificationinatorState;
import frc.robot.enums.ClawState;
import frc.robot.enums.DrivetrainState;
import frc.robot.enums.ElavatinatorState;
import frc.robot.enums.LEDinatorState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevatinator;
import frc.robot.utilities.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.enums.ScoringTarget;
import frc.robot.subsystems.Ascendinator;
import frc.robot.subsystems.Calsificationinator;
import frc.robot.subsystems.LEDinator;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController _driverController;
  private final CommandXboxController _operatorController;
  private final CommandXboxController _reefController;
  private final Drivetrain _drivetrain;
  private final Claw _claw;
  // private final Ascendinator _ascendinator;
  private final Vision _vision;
  private final Elevatinator _elevatinator;
  // private final SendableChooser<Command> _autoChooser;
  private final Calsificationinator _calsificationinator;
  // private final LEDinator _ledinator;
  private CalsificationinatorState _targetCalsificationinatorState = CalsificationinatorState.IDLE;
  private ClawState _targetClawState = ClawState.IDLE;
  private ScoringTarget _targetScore = ScoringTarget.REEF;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    _claw = new Claw();
    // _ascendinator = new Ascendinator();
    _driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    _operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
    _reefController = new CommandXboxController(OperatorConstants.kReefControllerPort);
    _elevatinator = new Elevatinator();
    _vision = new Vision();
    _drivetrain = new Drivetrain(_driverController, _vision);
    _calsificationinator = new Calsificationinator();
    // _ledinator = new
    // LEDinator(_calsificationinator,_claw,_elevatinator,_ascendinator);
    // // Configure the trigger bindings
    // new
    // EventTrigger("Initialize").onTrue(_drivetrain.setWantedState(DrivetrainState.AUTO).withTimeout(.05));
    // _autoChooser = AutoBuilder.buildAutoChooser("Test Auto");
    // SmartDashboard.putData("AutoChooser", _autoChooser);
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    _drivetrain.setDefaultCommand(
        _drivetrain.applyRequest(() -> _drivetrain.drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)));

    _driverController.rightTrigger().onTrue(scoreinator(this::GetDesiredScoringTarget)).onFalse(goHomeinator());
    _driverController.leftTrigger().onTrue(doinator(this::GetDesiredClawState, _targetCalsificationinatorState, null));
    _driverController.start().onTrue(_drivetrain.resetGyro());
    // _driverController.leftBumper().onTrue(_drivetrain.setWantedState(DrivetrainState.TARGET_FOLLOW))
    // .onFalse(_drivetrain.setWantedState(DrivetrainState.OPEN_LOOP));

    // _driverController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    // _driverController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
    // _driverController.y().whileTrue(_drivetrain.sysIdQuasistatic(Direction.kForward));
    // _driverController.b().whileTrue(_drivetrain.sysIdQuasistatic(Direction.kReverse));
    // _driverController.a().whileTrue(_drivetrain.sysIdDynamic(Direction.kForward));
    // _driverController.x().whileTrue(_drivetrain.sysIdDynamic(Direction.kReverse));
    // new Trigger
    // (_ascendinator::hasCage).negate().and(_driverController.back()).onTrue(new
    // ParallelCommandGroup(_ascendinator.setWantedState(CageState.DEPLOY),
    // _ledinator.setWantedState(LEDinatorState.CAGE)));
    // new
    // Trigger(_ascendinator::hasCage).and(_driverController.back()).onTrue(_ascendinator.setWantedState(CageState.ASCEND));

    // new
    // Trigger(_claw::hasAlgae).negate().and(_driverController.rightBumper()).onTrue(doinator(ClawState.FLOOR_INTAKE,
    // ElevatinatorConstants.kAlgaePickup, CalsificationinatorState.IDLE,
    // LEDinatorState.ALGAE)).onFalse(goHomeinator());

    // new
    // Trigger(DriverStation::isAutonomousEnabled).onTrue(_drivetrain.setWantedState(DrivetrainState.AUTO));
    new Trigger(DriverStation::isTeleopEnabled).onTrue(_drivetrain.setWantedState(DrivetrainState.OPEN_LOOP));
    new Trigger(_calsificationinator::hasCoralinator)
        .onTrue(_calsificationinator.setWantedState(CalsificationinatorState.IDLE));
    new Trigger(_calsificationinator::hasCoralinator)
        .onFalse(_calsificationinator.setWantedState(CalsificationinatorState.PICKUP));

    _operatorController.y()
        .onTrue(algaeSetinator(ElevatinatorConstants.kAlgaeNet, CalsificationinatorState.IDLE, ClawState.PREP_NET));
    _operatorController.x().onTrue(
        algaeSetinator(ElevatinatorConstants.kAlgaeProcessor, CalsificationinatorState.IDLE, ClawState.PREP_PROCESSOR));
    _operatorController.b()
        .onTrue(doinator(this::GetDesiredClawState, CalsificationinatorState.PICKUP, LEDinatorState.CORAL));
    _reefController.rightBumper().onTrue(new InstantCommand(() -> _drivetrain.setReefTargetFace(4)));
    _reefController.leftBumper().onTrue(new InstantCommand(() -> _drivetrain.setReefTargetFace(5)));
    _reefController.y().onTrue(new InstantCommand(() -> _drivetrain.setReefTargetFace(0)));
    _reefController.x().onTrue(new InstantCommand(() -> _drivetrain.setReefTargetFace(1)));
    _reefController.b().onTrue(new InstantCommand(() -> _drivetrain.setReefTargetFace(2)));
    _reefController.a().onTrue(new InstantCommand(() -> _drivetrain.setReefTargetFace(3)));
    _reefController.povLeft().onTrue(reefSetinator(ElevatinatorConstants.kL4Coral, ReefConstants.kReefLeft,
        CalsificationinatorState.L4, ClawState.IDLE, ScoringTarget.REEF));
    _reefController.leftStick().onTrue(reefSetinator(ElevatinatorConstants.kL4Coral, ReefConstants.kReefRight,
        CalsificationinatorState.L4, ClawState.IDLE, ScoringTarget.REEF));
    _reefController.back().onTrue(reefSetinator(ElevatinatorConstants.kL3Coral, ReefConstants.kReefLeft,
        CalsificationinatorState.L3, ClawState.IDLE, ScoringTarget.REEF));
    _reefController.rightStick().onTrue(reefSetinator(ElevatinatorConstants.kL3Coral, ReefConstants.kReefRight,
        CalsificationinatorState.L3, ClawState.IDLE, ScoringTarget.REEF));
    _reefController.start().onTrue(reefSetinator(ElevatinatorConstants.kL2Coral, ReefConstants.kReefLeft,
        CalsificationinatorState.L2, ClawState.IDLE, ScoringTarget.REEF));
    _reefController.povUp().onTrue(reefSetinator(ElevatinatorConstants.kL2Coral, ReefConstants.kReefRight,
        CalsificationinatorState.L2, ClawState.IDLE, ScoringTarget.REEF));
    // _reefController.povRight().onTrue(reefSetinator(ElevatinatorConstants.kL1Coral,
    // ReefConstants.kReefAlgae, CalsificationinatorState.L1, ClawState.IDLE,
    // ScoringTarget.REEF));
    _operatorController.a().onTrue(L1());
    _reefController.povDown().onTrue(reefSetinator(ElevatinatorConstants.kHome, ReefConstants.kReefAlgae,
        CalsificationinatorState.IDLE, ClawState.DEALGIFY, ScoringTarget.REEF));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // return _autoChooser.getSelected();
  // }

  public ParallelCommandGroup goHomeinator() {
    return new ParallelCommandGroup(
        // _claw.setWantedState(ClawState.IDLE),
        _elevatinator.setWantedState(ElavatinatorState.HOME),
        _calsificationinator.setWantedState(CalsificationinatorState.PICKUP));
  }

  public ParallelCommandGroup doinator(Supplier<ClawState> clawinator,
      CalsificationinatorState calsificationinatorState, LEDinatorState ledinatorState) {
    return new ParallelCommandGroup(
        _claw.setWantedState(clawinator),
        _elevatinator.setWantedState(ElavatinatorState.HOLD),
        _calsificationinator.setWantedState(CalsificationinatorState.L4)
    // _ledinator.setWantedState(ledinatorState)
    );
  }

  public ParallelCommandGroup L1() {
    return new ParallelCommandGroup(
        // _claw.setWantedState(clawinator),
        _elevatinator.setWantedState(ElavatinatorState.HOLD),
        _calsificationinator.setWantedState(CalsificationinatorState.L4)
    // _ledinator.setWantedState(ledinatorState)
    );
  }

  public ParallelCommandGroup reefSetinator(double position, int side,
      CalsificationinatorState calsificationinatorState, ClawState clawState, ScoringTarget scoringTarget) {
    return new ParallelCommandGroup(new InstantCommand(() -> {
      if (side == ReefConstants.kReefAlgae) {
        if (_drivetrain.getReefTargetFace() % 2 == 0) {
          _elevatinator.setPositioninator(ElevatinatorConstants.kL2Algae);
        } else {
          _elevatinator.setPositioninator(ElevatinatorConstants.kL3Algae);
        }
      } else {        
        _elevatinator.setPositioninator(position);
      }

      _elevatinator.setPositioninator(position);
      _drivetrain.setReefTargetSide(side);
      _targetCalsificationinatorState = calsificationinatorState;
      _targetClawState = clawState;
      _targetScore = scoringTarget;

    })
    // , _ledinator.setWantedState(LEDinatorState.CORAL)
    );
  }

  public ParallelCommandGroup algaeSetinator(double position, CalsificationinatorState calsificationinatorState,
      ClawState clawState) {
    return new ParallelCommandGroup(new InstantCommand(() -> {
      _targetCalsificationinatorState = calsificationinatorState;
      _targetClawState = clawState;
    })
    // _ledinator.setWantedState(LEDinatorState.ALGAE)
    );
  }

  public InstantCommand scoreinator(Supplier<ScoringTarget> scoringTarget) {
    switch (scoringTarget.get()) {
      case PROCESSOR:
      case NET:
        // return _claw.setWantedState(ClawState.SCORE);
      default:
        return _calsificationinator.setWantedState(CalsificationinatorState.SCORE);
    }
  }

  private ScoringTarget GetDesiredScoringTarget() {
    return _targetScore;
  }

  private ClawState GetDesiredClawState() {
    return _targetClawState;
  }
}
