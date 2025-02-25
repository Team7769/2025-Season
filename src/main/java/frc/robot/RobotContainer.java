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
import frc.robot.enums.LocationTarget;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final SendableChooser<Command> _autoChooser;
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

    registerEventTriggersForAuto();
    registerNamedCommandsForAuto();
    _autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", _autoChooser);
    configureBindings();
  }

  private void registerNamedCommandsForAuto()
  {
    // NamedCommands.registerCommand("Wait Until Coral Scored", Commands.waitUntil(_calsificationinator::doesNotHaveCoralinator).andThen(goHomeinator()));
    NamedCommands.registerCommand("Score Coral", Commands.sequence(Commands.waitUntil(_elevatinator::isReady), _calsificationinator.setWantedState(CalsificationinatorState.SCORE),
      Commands.waitUntil(_calsificationinator::doesNotHaveCoralinator), Commands.parallel(
        _claw.setWantedState(ClawState.IDLE),
        _calsificationinator.setWantedState(CalsificationinatorState.PICKUP))));
    NamedCommands.registerCommand("Wait For Coral", Commands.sequence(Commands.waitUntil(_calsificationinator::hasCoralinator), _calsificationinator.setWantedState(CalsificationinatorState.IDLE)));
    // NamedCommands.registerCommand("Go Home", Commands.parallel(
    //   _claw.setWantedState(ClawState.IDLE),
    //   _calsificationinator.setWantedState(CalsificationinatorState.PICKUP)));
  }

  private void registerEventTriggersForAuto() {
    // new EventTrigger("Score Coral").onTrue(
    //   Commands.sequence(Commands.waitUntil(_elevatinator::isReady), _calsificationinator.setWantedState(CalsificationinatorState.SCORE), Commands.waitSeconds(.4))
    // );
    // new EventTrigger("Prep For Coral").onTrue(
    //   Commands.sequence(
    //     new InstantCommand(() -> _elevatinator.setPositioninator(ElevatinatorConstants.kL4Coral), _elevatinator),
    //     _elevatinator.setWantedState(ElavatinatorState.HOLD),
    //     _calsificationinator.setWantedState(CalsificationinatorState.L4)));
    new EventTrigger("Prep For Coral").onTrue(
      Commands.sequence(
        new InstantCommand(() -> _elevatinator.setPositioninator(ElevatinatorConstants.kL4Coral), _elevatinator),
        _elevatinator.setWantedState(ElavatinatorState.HOLD)));
    // new EventTrigger("Go Home").onTrue(
    //   Commands.parallel(
    //     _claw.setWantedState(ClawState.IDLE),
    //     _elevatinator.setWantedState(ElavatinatorState.HOME),
    //     _calsificationinator.setWantedState(CalsificationinatorState.PICKUP))
    // );  
    new EventTrigger("Go Home Elevator").onTrue(_elevatinator.setWantedState(ElavatinatorState.HOME));
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

    _driverController.rightTrigger().onTrue(scoreinator()).onFalse(goHomeinator());
    _driverController.leftTrigger().onTrue(doinator(null));
    _driverController.start().onTrue(_drivetrain.resetGyro());
    _driverController.back().onTrue(goHomeinator());
    _driverController.leftBumper().onTrue(_drivetrain.setWantedState(DrivetrainState.TARGET_FOLLOW))
    .onFalse(_drivetrain.setWantedState(DrivetrainState.OPEN_LOOP));

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

    new Trigger(DriverStation::isTeleopEnabled).onTrue(_drivetrain.setWantedState(DrivetrainState.OPEN_LOOP));

    // new
    // Trigger(_claw::doesNotHaveAlgae).and(_driverController.rightBumper()).onTrue(Commands.sequence(new InstantCommand(()->{
    //   _targetClawState = ClawState.FLOOR_INTAKE;
    //   _elevatinator.setPositioninator(Constants.ElevatinatorConstants.kAlgaePickup);
      
    // }), Commands.parallel(
    //   _claw.setWantedState(ClawState.FLOOR_INTAKE), 
    //   _calsificationinator.setWantedState(CalsificationinatorState.PICKUP),
    //   _elevatinator.setWantedState(ElavatinatorState.HOLD))));

    // new Trigger(_driverController.rightBumper()).negate().and(_claw::doesNotHaveAlgae).onTrue(goHomeinatorForFloorPickup());
    // new Trigger(_claw::hasAlgae).onTrue(goHomeinatorWithAlgae());
    
    new Trigger(_calsificationinator::hasCoralinator).and(DriverStation::isTeleopEnabled)
        .onTrue(_calsificationinator.setWantedState(CalsificationinatorState.IDLE))
        .onFalse(_calsificationinator.setWantedState(CalsificationinatorState.PICKUP));

    // _operatorController.y()
    //     .onTrue(algaeSetinator(ElevatinatorConstants.kAlgaeNet, CalsificationinatorState.PICKUP, ClawState.PREP_NET));
    // _operatorController.x().onTrue(
    //     algaeSetinator(ElevatinatorConstants.kAlgaeProcessor, CalsificationinatorState.PICKUP, ClawState.PREP_PROCESSOR));
    //_operatorController.b()
    //    .onTrue(doinator(ClawState.IDLE, CalsificationinatorState.PICKUP, LEDinatorState.CORAL));
    _reefController.rightBumper().onTrue(new InstantCommand(() -> 
    {
      _drivetrain.setReefTargetFace(1);
      _elevatinator.setAlgaePosition(1);
    }));
    _reefController.leftBumper().onTrue(new InstantCommand(() ->{
      _drivetrain.setReefTargetFace(2);
      _elevatinator.setAlgaePosition(2);
    }));
    _reefController.y().onTrue(new InstantCommand(() -> {
      _drivetrain.setReefTargetFace(3);
      _elevatinator.setAlgaePosition(3);
    }));
    _reefController.x().onTrue(new InstantCommand(() -> {
      _drivetrain.setReefTargetFace(4);
      _elevatinator.setAlgaePosition(4);
    }));
    _reefController.b().onTrue(new InstantCommand(() -> {
      _drivetrain.setReefTargetFace(5);
      _elevatinator.setAlgaePosition(5);
    }));
    _reefController.a().onTrue(new InstantCommand(() -> {
      _drivetrain.setReefTargetFace(0);
      _elevatinator.setAlgaePosition(0);
    }));

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
    
    _operatorController.a().onTrue(dealgifyLow(null));
    _operatorController.b().onTrue(dealgifyHigh(null));    
    _operatorController.x().onTrue(algaeNet(null));
    _operatorController.y().onTrue(algaeProcessor(null));
    _reefController.povDown().onTrue(reefSetinator(ElevatinatorConstants.kHome, ReefConstants.kReefAlgae,
        CalsificationinatorState.IDLE, ClawState.DEALGIFY, ScoringTarget.REEF));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
  return _autoChooser.getSelected();
  }

  public Command goHomeinator() {
    return Commands.parallel(
        new InstantCommand(() -> {
          _elevatinator.setHoldAlgaePosition(false);
          _drivetrain.targetSource(GeometryUtil::isRedAlliance);
        }
        //,  _elevatinator, _drivetrain
        ),
        _drivetrain.setWantedTarget(LocationTarget.CORAL_SOURCE),
        _claw.setWantedState(ClawState.IDLE),
        _elevatinator.setWantedState(ElavatinatorState.HOME),
        _calsificationinator.setWantedState(CalsificationinatorState.PICKUP));
  }

  public Command goHomeinatorForFloorPickup() {
    return Commands.sequence(_claw.setWantedState(ClawState.IDLE), Commands.waitSeconds(0.5), Commands.parallel(
        _elevatinator.setWantedState(ElavatinatorState.HOME),
        _calsificationinator.setWantedState(CalsificationinatorState.PICKUP)));
  }

  public Command goHomeinatorWithAlgae() {
    return Commands.sequence(_claw.setWantedState(ClawState.PREP_PROCESSOR), Commands.waitSeconds(0.5), Commands.parallel(
        _elevatinator.setWantedState(ElavatinatorState.HOMEWITHALGAE),
        _calsificationinator.setWantedState(CalsificationinatorState.PICKUP)));
  }

  public Command doinator(ClawState clawinator,
      CalsificationinatorState calsificationinatorState, LEDinatorState ledinatorState) {
    return Commands.parallel(
      //_claw.setWantedState(clawinator),
      _elevatinator.setWantedState(ElavatinatorState.HOLD),
      _calsificationinator.setWantedState(CalsificationinatorState.L4)
      // _ledinator.setWantedState(ledinatorState)
      );
  }

  public Command doinator(LEDinatorState ledinatorState) {
    return Commands.parallel(
      _claw.setWantedState(ClawState.IDLE),
      _elevatinator.setWantedState(ElavatinatorState.HOLD),
      _calsificationinator.setWantedState(CalsificationinatorState.L4)
      // _ledinator.setWantedState(ledinatorState)
    );
  }

  public Command dealgifyHigh(LEDinatorState ledinatorState) {
    return Commands.sequence(new InstantCommand(() -> {
      _elevatinator.setPositioninator(ElevatinatorConstants.kL3Algae);
    }), Commands.parallel(
      _claw.setWantedState(ClawState.DEALGIFY),
      _elevatinator.setWantedState(ElavatinatorState.HOLD),
      _calsificationinator.setWantedState(CalsificationinatorState.PICKUP)
      // _ledinator.setWantedState(ledinatorState)
    ));
  }

  public Command dealgifyLow(LEDinatorState ledinatorState) {
    return Commands.sequence(new InstantCommand(() -> {
      _elevatinator.setPositioninator(ElevatinatorConstants.kL2Algae);
    }), Commands.parallel(
      _claw.setWantedState(ClawState.DEALGIFY),
      _elevatinator.setWantedState(ElavatinatorState.HOLD),
      _calsificationinator.setWantedState(CalsificationinatorState.PICKUP)
      // _ledinator.setWantedState(ledinatorState)
    ));
  }
  
  public Command algaeNet(LEDinatorState ledinatorState) {
    return Commands.sequence(new InstantCommand(() -> {
      _elevatinator.setPositioninator(ElevatinatorConstants.kAlgaeNet);
    }), Commands.parallel(
      _claw.setWantedState(ClawState.PREP_NET),
      _elevatinator.setWantedState(ElavatinatorState.HOLD),
      _calsificationinator.setWantedState(CalsificationinatorState.PICKUP)
      // _ledinator.setWantedState(ledinatorState)
    ));
  }
  
  public Command algaeProcessor(LEDinatorState ledinatorState) {
    return Commands.sequence(new InstantCommand(() -> {
      _elevatinator.setPositioninator(ElevatinatorConstants.kAlgaeProcessor);
    }), Commands.parallel(
      _claw.setWantedState(ClawState.PREP_PROCESSOR),
      _elevatinator.setWantedState(ElavatinatorState.HOLD),
      _calsificationinator.setWantedState(CalsificationinatorState.PICKUP)
      // _ledinator.setWantedState(ledinatorState)
    ));
  }

  public Command L1() {
    return Commands.parallel(
        // _claw.setWantedState(clawinator),
        _elevatinator.setWantedState(ElavatinatorState.HOLD),
        _calsificationinator.setWantedState(CalsificationinatorState.L4)
    // _ledinator.setWantedState(ledinatorState)
    );
  }

  public Command reefSetinator(double position, int side,
      CalsificationinatorState calsificationinatorState, ClawState clawState, ScoringTarget scoringTarget) {
    return Commands.parallel(new InstantCommand(() -> {
      _elevatinator.setPositioninator(position);
      _drivetrain.setReefTargetSide(side);
      _calsificationinator.setTargetState(calsificationinatorState);
      _elevatinator.setHoldAlgaePosition(false);
      _drivetrain.targetReef(GeometryUtil::isRedAlliance);
      _targetClawState = clawState;
      _targetScore = scoringTarget;

    }),
    _drivetrain.setWantedTarget(LocationTarget.REEF)
    // , _ledinator.setWantedState(LEDinatorState.CORAL)
    );
  }

  public Command algaeSetinator(double position, CalsificationinatorState calsificationinatorState,
      ClawState clawState) {
    return Commands.parallel(new InstantCommand(() -> {
      _calsificationinator.setTargetState(calsificationinatorState);
      _claw.setTargetState(clawState);
      _elevatinator.setPositioninator(position);
    })
    // _ledinator.setWantedState(LEDinatorState.ALGAE)
    );
  }

  public Command scoreinator() {
    // switch (scoringTarget.get()) {
    //   case PROCESSOR:
    //   case NET:
    //   default:
    //     return _claw.setWantedState(ClawState.SCORE);
    //     return _calsificationinator.setWantedState(CalsificationinatorState.SCORE);
    // }

    return Commands.parallel(_claw.setWantedState(ClawState.SCORE), _calsificationinator.setWantedState(CalsificationinatorState.SCORE));
  }

  private ScoringTarget GetDesiredScoringTarget() {
    return _targetScore;
  }

  private ClawState GetDesiredClawState() {
    return _targetClawState;
  }
}
