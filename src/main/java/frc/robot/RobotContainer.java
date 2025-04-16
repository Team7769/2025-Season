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

import java.util.Set;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

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
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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
  private final Ascendinator _ascendinator;
  private final Vision _vision;
  private final Elevatinator _elevatinator;
  private final SendableChooser<Command> _autoChooser;
  private final Calsificationinator _calsificationinator;
  private final LEDinator _ledinator;
  private CalsificationinatorState _targetCalsificationinatorState = CalsificationinatorState.IDLE;
  private ClawState _targetClawState = ClawState.IDLE;
  private ScoringTarget _targetScore = ScoringTarget.REEF;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    _claw = new Claw();
    _ascendinator = new Ascendinator();
    _driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    _operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
    _reefController = new CommandXboxController(OperatorConstants.kReefControllerPort);
    _elevatinator = new Elevatinator();
    _vision = new Vision();
    _drivetrain = new Drivetrain(_driverController, _vision);
    _calsificationinator = new Calsificationinator();
    _ledinator = new
    LEDinator(_calsificationinator,_claw,_elevatinator,_ascendinator);

    SmartDashboard.putData(_elevatinator);
    SmartDashboard.putData(_claw);
    SmartDashboard.putData(_calsificationinator);
    SmartDashboard.putData(_ascendinator);

    registerEventTriggersForAuto();
    registerNamedCommandsForAuto();
    _autoChooser = AutoBuilder.buildAutoChooser();
    _autoChooser.addOption("RCH Special", RCHSpecial());
    _autoChooser.addOption("RCH Safely Cheesy", right3CoralAlgae());
    _autoChooser.addOption("LGHP Special", LGHPSpecial());
    _autoChooser.addOption("LGHP Safely Cheesy", left3CoralAlgae());
    _autoChooser.addOption( "MUFIC Special", getAutoMiddle());
    _autoChooser.addOption("Procesceor Special", processorAuto());
    _autoChooser.addOption("Test Right", right3CoralPrep());
    _autoChooser.addOption("Test Left", left3CoralPrep());
    SmartDashboard.putData("AutoChooser", _autoChooser);
    configureBindings();
  }

  private void registerNamedCommandsForAuto()
  {
    NamedCommands.registerCommand("Score Coral", scoreSequence());
    NamedCommands.registerCommand("Wait For Coral", waitForCoral());
  }

  private void registerEventTriggersForAuto() {
    new EventTrigger("Prep For Coral").onTrue(
      Commands.sequence(
        new InstantCommand(() -> _elevatinator.setPositioninator(ElevatinatorConstants.kL4Coral), _elevatinator),
        _elevatinator.setWantedState(ElavatinatorState.HOLD)));
        
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

    _driverController.rightTrigger().onTrue(Commands.defer(this::scoreinator, Set.of())).onFalse(goHomeinator());
    //_driverController.leftTrigger().onTrue(doinator(null));
    _driverController.leftTrigger().onTrue(Commands.defer(this::doThing, Set.of()));

    _driverController.start().onTrue(_drivetrain.resetGyro());
    new Trigger(_claw::hasAlgae).and(_driverController.a()).onTrue(goHomeinatorWithAlgae());
    new Trigger(_claw::doesNotHaveAlgae).and(_driverController.a()).onTrue(goHomeinator());
    _driverController.b().onTrue(Commands.defer(this::doinator, Set.of()));
    _driverController.leftBumper().onTrue(_drivetrain.setWantedState(DrivetrainState.TARGET_FOLLOW))
    .onFalse(_drivetrain.setWantedState(DrivetrainState.OPEN_LOOP));

    _driverController.povUp().onTrue(Commands.parallel
    (_drivetrain.setWantedTarget(LocationTarget.CAGE),
      _ascendinator.setWantedState(CageState.DEPLOY),
      _ledinator.setWantedState(LEDinatorState.CAGE),
     _calsificationinator.setWantedState(CalsificationinatorState.PICKUP),
     _claw.setWantedState(ClawState.PREP_CLIMB)
    ));
    new Trigger(_ascendinator::isReady).and(_driverController.back()).onTrue(Commands.parallel(_ascendinator.setWantedState(CageState.ASCEND), 
    _claw.setWantedState(ClawState.PREP_CLIMB), _calsificationinator.setWantedState(CalsificationinatorState.PREP_CLIMB), _elevatinator.setWantedState(ElavatinatorState.HOME)));

    new Trigger(DriverStation::isTeleopEnabled).onTrue(_drivetrain.setWantedState(DrivetrainState.OPEN_LOOP));

    _driverController.x().onTrue(Commands.parallel(
      new InstantCommand(() -> _elevatinator.setPositioninator(ElevatinatorConstants.kAlgaeEmergency)), 
    _elevatinator.setWantedState(ElavatinatorState.HOLD),
    _claw.setWantedState(ClawState.EMERGENCY), 
    _ledinator.setWantedState(LEDinatorState.ALGAE), 
    _calsificationinator.setWantedState(CalsificationinatorState.PICKUP)))
    .onFalse(goHomeinatorForEmergencyPickup());

    _driverController.y().onTrue(Commands.parallel(
      new InstantCommand(() -> _elevatinator.setPositioninator(ElevatinatorConstants.kLolipop)), 
    _elevatinator.setWantedState(ElavatinatorState.HOLD),
    _claw.setWantedState(ClawState.PRAISE_BE_THE_ONE_TRUE_LORD),
    _ledinator.setWantedState(LEDinatorState.ALGAE), 
    _calsificationinator.setWantedState(CalsificationinatorState.PICKUP)));

    _driverController.rightBumper().onTrue(
      Commands.sequence(
        _calsificationinator.setWantedState(CalsificationinatorState.PICKUP),
        Commands.waitUntil(_calsificationinator::isReady),
        new InstantCommand(() -> _elevatinator.setPositioninator(ElevatinatorConstants.kAlgaePickup)), 
      _elevatinator.setWantedState(ElavatinatorState.HOLD), 
      _claw.setWantedState(ClawState.FLOOR_INTAKE), 
      _ledinator.setWantedState(LEDinatorState.ALGAE)));
      
    new Trigger(_claw::hasAlgae).and(DriverStation::isTeleopEnabled).onTrue(goHomeinatorWithAlgae());
    new Trigger(_driverController.rightBumper().negate()).and(_claw::doesNotHaveAlgae).and(DriverStation::isTeleopEnabled).onTrue(goHomeinatorForFloorPickup());
    if (!_ascendinator.isReady()){
      // new Trigger(_calsificationinator::hasCoralinator).and(DriverStation::isTeleopEnabled)
      // .onTrue(_calsificationinator.setWantedState(CalsificationinatorState.IDLE))
      // .onFalse(_calsificationinator.setWantedState(CalsificationinatorState.PICKUP));
      new Trigger(_calsificationinator::hasCoralinator).and(DriverStation::isTeleopEnabled)
      .onTrue(_calsificationinator.setWantedState(CalsificationinatorState.IDLE_MIDDLE))
      .onFalse(_calsificationinator.setWantedState(CalsificationinatorState.PICKUP));
    }

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
    _reefController.povLeft().and(_calsificationinator::hasCoralinator).onTrue(reefSetinator(ElevatinatorConstants.kL4Coral, ReefConstants.kReefLeft,
        CalsificationinatorState.L4, ClawState.IDLE, ScoringTarget.REEF));
    _reefController.leftStick().and(_calsificationinator::hasCoralinator).onTrue(reefSetinator(ElevatinatorConstants.kL4Coral, ReefConstants.kReefRight,
        CalsificationinatorState.L4, ClawState.IDLE, ScoringTarget.REEF));
    _reefController.back().and(_calsificationinator::hasCoralinator).onTrue(reefSetinator(ElevatinatorConstants.kL3Coral, ReefConstants.kReefLeft,
        CalsificationinatorState.L3, ClawState.IDLE, ScoringTarget.REEF));
    _reefController.rightStick().and(_calsificationinator::hasCoralinator).onTrue(reefSetinator(ElevatinatorConstants.kL3Coral, ReefConstants.kReefRight,
        CalsificationinatorState.L3, ClawState.IDLE, ScoringTarget.REEF));
    _reefController.start().and(_calsificationinator::hasCoralinator).onTrue(reefSetinator(ElevatinatorConstants.kL2Coral, ReefConstants.kReefLeft,
        CalsificationinatorState.L2, ClawState.IDLE, ScoringTarget.REEF));
    _reefController.povUp().and(_calsificationinator::hasCoralinator).onTrue(reefSetinator(ElevatinatorConstants.kL2Coral, ReefConstants.kReefRight,
        CalsificationinatorState.L2, ClawState.IDLE, ScoringTarget.REEF));
    // _reefController.povRight().onTrue(reefSetinator(ElevatinatorConstants.kL1Coral,
    // ReefConstants.kReefAlgae, CalsificationinatorState.L1, ClawState.IDLE,
    // ScoringTarget.REEF));
    _operatorController.leftBumper().and(_calsificationinator::hasCoralinator).onTrue(reefSetinator(ElevatinatorConstants.kL4Coral, ReefConstants.kReefRight,
        CalsificationinatorState.KILL_MODE, ClawState.IDLE, ScoringTarget.REEF));
    _operatorController.rightBumper().and(_calsificationinator::hasCoralinator).onTrue(reefSetinator(ElevatinatorConstants.kL4Coral, ReefConstants.kReefLeft,
        CalsificationinatorState.KILL_MODE, ClawState.IDLE, ScoringTarget.REEF));
    _operatorController.x().onTrue(algaeNet(null));
    // _operatorController.b().onTrue(dealgifyHigh(null)); 
    _operatorController.b().onTrue(dealgify()); 
    _operatorController.y().onTrue(algaeProcessor(null));
    _operatorController.a().and(_calsificationinator::hasCoralinator).onTrue(reefSetinator(ElevatinatorConstants.kL1Coral, ReefConstants.kReefAlgae,
    CalsificationinatorState.L1, ClawState.IDLE, ScoringTarget.REEF));
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

  public Command getReturnCommand(){
    return _claw.hasAlgae() ? new ParallelCommandGroup(goHomeinatorWithAlgae(), _ledinator.setWantedState(LEDinatorState.ALGAE))
                            : new ParallelCommandGroup(goHomeinatorForFloorPickup(), _ledinator.setWantedState(LEDinatorState.CORAL));
  }

  public Command doThing(){
    if (_drivetrain.getCurrentTarget() != LocationTarget.CORAL_SOURCE && _drivetrain.getCurrentTarget() != LocationTarget.CAGE)
    {
      if (_claw.hasAlgae()){
        SmartDashboard.putString("Current Action", "Claw has algae for DoThing");
        if (_drivetrain.getCurrentTarget() == LocationTarget.PROCESSOR) {
          return 
          Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.TARGET_FOLLOW))
          .andThen(doinator(null))
          .andThen(Commands.waitUntil(_drivetrain::isAtTarget))
          .andThen(Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.OPEN_LOOP)))
          .handleInterrupt(() -> System.out.println("Interrupted doThing."))
          .until(_driverController.a().or(_driverController.rightTrigger())).andThen(Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.OPEN_LOOP)));
        } else {
          return 
          Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.TARGET_FOLLOW))
          .andThen(Commands.waitUntil(_drivetrain::isAtTarget))
          .andThen(doinator(null))
          .andThen(scoreBarge())
          .andThen(goHomeinator().alongWith(Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.OPEN_LOOP))))
          .andThen(Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.OPEN_LOOP)))
          .handleInterrupt(() -> System.out.println("Interrupted doThing."))
          .until(_driverController.a().or(_driverController.rightTrigger())).andThen(Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.OPEN_LOOP)));
        }
      } else if (_claw.getTargetState() == ClawState.DEALGIFY){
        SmartDashboard.putString("Current Action", "Claw is dealgifying for DoThing");
        return Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.TARGET_FOLLOW))
        .andThen(Commands.waitUntil(_drivetrain::isNearTarget))
        .andThen(doinator(null))
        .andThen(Commands.waitUntil(_claw::hasAlgae))
        .andThen(goHomeinatorWithAlgae().alongWith(Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.OPEN_LOOP))))
        .handleInterrupt(() -> System.out.println("Interrupted doThing."))
        .until(_driverController.a()).andThen(Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.OPEN_LOOP))
        );
      } else if (_calsificationinator.getTargetState() != CalsificationinatorState.KILL_MODE) {
      SmartDashboard.putString("Current Action", "Coral for DoThing");
      return 
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.TARGET_FOLLOW))
      .andThen(Commands.waitUntil(_drivetrain::isNearTarget))
      .andThen(doinator(null))
      .andThen(Commands.waitUntil(_drivetrain::isAtTarget))
      .andThen(Commands.waitUntil(_elevatinator::isReady))
      .andThen(Commands.waitUntil(_calsificationinator::isReady))
      .andThen(scoreSequence())
      .andThen(goHomeinator().alongWith(Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.OPEN_LOOP))))
      .handleInterrupt(() -> System.out.println("Interrupted doThing."))
      // .until(_driverController.a().or(_driverController.rightTrigger())).andThen(goHomeinator().alongWith(Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.OPEN_LOOP)))
      .until(_driverController.a().or(_driverController.rightTrigger())).andThen(() -> _drivetrain.setWantedStateNormal(DrivetrainState.OPEN_LOOP)
      );
    } else {
      SmartDashboard.putString("Current Action", "Coral and Dealgify for DoThing");
      return 
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.TARGET_FOLLOW))
      .andThen(Commands.waitUntil(_drivetrain::isNearTarget))
      .andThen(doinator(null))
      .andThen(Commands.waitUntil(_drivetrain::isAtTarget))
      .andThen(Commands.waitUntil(_elevatinator::isReady))
      .andThen(Commands.waitUntil(_calsificationinator::isReady))
      .andThen(scoreSequence())
      .andThen(dealgify())
      .andThen(doinator(null))
      .andThen(Commands.waitUntil(_claw::hasAlgae))
      .andThen(goHomeinatorWithAlgae().alongWith(Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.OPEN_LOOP))))
      .handleInterrupt(() -> System.out.println("Interrupted doThing."))
      .until(_driverController.a().or(_driverController.rightTrigger())).andThen(Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.OPEN_LOOP)));
    }
    } else {
      return new InstantCommand();
    }
  }

  public Command goHomeinator() {
    return Commands.parallel(
        new InstantCommand(() -> {
          _elevatinator.setHoldAlgaePosition(false);
          _drivetrain.targetSource(GeometryUtil::isRedAlliance);
        }
        //,  _elevatinator, _drivetrain
        ),
        Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.OPEN_LOOP)),
        Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.OPEN_LOOP)),
        _drivetrain.setWantedTarget(LocationTarget.CORAL_SOURCE),
        _ledinator.setWantedState(LEDinatorState.CORAL),
        _claw.setWantedState(ClawState.IDLE),
        _elevatinator.setWantedState(ElavatinatorState.HOME),
        _calsificationinator.setWantedState(CalsificationinatorState.PICKUP));
  }

  public Command goHomeinatorForFloorPickup() {
    return Commands.sequence(_claw.setWantedState(ClawState.IDLE), Commands.waitSeconds(0.5), Commands.parallel(
        _elevatinator.setWantedState(ElavatinatorState.HOME),
        _calsificationinator.setWantedState(CalsificationinatorState.PICKUP)));
  }

  public Command goHomeinatorForEmergencyPickup() {
    return Commands.sequence(_elevatinator.setWantedState(ElavatinatorState.HOMEWITHALGAE), Commands.waitUntil(_elevatinator::isReady), Commands.parallel(
      _claw.setWantedState(ClawState.IDLE)),
        _calsificationinator.setWantedState(CalsificationinatorState.PICKUP));
  }

  public Command goHomeinatorWithAlgae() {
    return Commands.parallel(
        new InstantCommand(() -> {
          _elevatinator.setHoldAlgaePosition(false);
          // _elevatinator.setPositioninator(ElevatinatorConstants.kAlgaeHold);
        }
        ),
        Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.OPEN_LOOP)),
        Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.OPEN_LOOP)),
        _claw.setWantedState(ClawState.IDLE_WITH_ALGAE),
        _elevatinator.setWantedState(ElavatinatorState.HOMEWITHALGAE),
        _ledinator.setWantedState(LEDinatorState.ALGAE),
        _calsificationinator.setWantedState(CalsificationinatorState.PICKUP));
  }

  public Command doinator(LEDinatorState ledinatorState) {
    if (_drivetrain.getCurrentTarget() == LocationTarget.REEF) {
      return Commands.sequence(
        _claw.setWantedState(ClawState.TARGET),
        _elevatinator.setWantedState(ElavatinatorState.HOLD),
        Commands.waitUntil(_elevatinator::isCloseEnough),
        _calsificationinator.setWantedState(CalsificationinatorState.TARGET)
      );
    } else if(_drivetrain.getCurrentTarget() != LocationTarget.PROCESSOR) {
      return Commands.parallel(
        _claw.setWantedState(ClawState.TARGET),
        _elevatinator.setWantedState(ElavatinatorState.HOLD),
        _calsificationinator.setWantedState(CalsificationinatorState.TARGET)
      );
    } else {
      return Commands.sequence(
        _claw.setWantedState(ClawState.TARGET),
        Commands.waitUntil(_claw::isProcceorReady),
        _elevatinator.setWantedState(ElavatinatorState.HOLD),
        _calsificationinator.setWantedState(CalsificationinatorState.TARGET)
      );
    }
  }

  public Command doinator() {
    if (_drivetrain.getCurrentTarget() == LocationTarget.REEF) {
      return Commands.sequence(
        _claw.setWantedState(ClawState.TARGET),
        _elevatinator.setWantedState(ElavatinatorState.HOLD),
        Commands.waitUntil(_elevatinator::isCloseEnough),
        _calsificationinator.setWantedState(CalsificationinatorState.TARGET)
      );
    } else if(_drivetrain.getCurrentTarget() != LocationTarget.PROCESSOR) {
      return Commands.parallel(
        _claw.setWantedState(ClawState.TARGET),
        _elevatinator.setWantedState(ElavatinatorState.HOLD),
        _calsificationinator.setWantedState(CalsificationinatorState.TARGET)
      );
    } else {
      return Commands.sequence(
        _claw.setWantedState(ClawState.TARGET),
        Commands.waitUntil(_claw::isProcceorReady),
        _elevatinator.setWantedState(ElavatinatorState.HOLD),
        _calsificationinator.setWantedState(CalsificationinatorState.TARGET)
      );
    }
  }

  public Command dealgify() {
    return Commands.parallel(new InstantCommand(()-> {
      _elevatinator.setPositioninator(_drivetrain.getReefTargetFace() % 2 == 0 ? ElevatinatorConstants.kL3Algae : ElevatinatorConstants.kL2Algae);
      _claw.setTargetState(ClawState.DEALGIFY);
      _ledinator.setWantedState(LEDinatorState.ALGAE);
      _calsificationinator.setTargetState(CalsificationinatorState.PICKUP);
      _drivetrain.setReefTargetSide(ReefConstants.kReefAlgae);
    }), _drivetrain.setWantedTarget(LocationTarget.REEF));
  }

  public Command dealgifyHigh(LEDinatorState ledinatorState) {
    return Commands.parallel(new InstantCommand(()-> {
      _elevatinator.setPositioninator(ElevatinatorConstants.kL3Algae);
      _claw.setTargetState(ClawState.DEALGIFY);
      _ledinator.setWantedState(LEDinatorState.ALGAE);
      _calsificationinator.setTargetState(CalsificationinatorState.PICKUP);
      _drivetrain.setReefTargetSide(ReefConstants.kReefAlgae);
    }), _drivetrain.setWantedTarget(LocationTarget.REEF));
  }

  public Command dealgifyLow(LEDinatorState ledinatorState) {
    return Commands.parallel(new InstantCommand(()-> {
      _elevatinator.setPositioninator(ElevatinatorConstants.kL2Algae);
      _claw.setTargetState(ClawState.DEALGIFY);
      _ledinator.setWantedState(LEDinatorState.ALGAE);
      _calsificationinator.setTargetState(CalsificationinatorState.PICKUP);
      _drivetrain.setReefTargetSide(ReefConstants.kReefAlgae);
    }), _drivetrain.setWantedTarget(LocationTarget.REEF));
  }
  
  public Command algaeNet(LEDinatorState ledinatorState) {
    return Commands.parallel(new InstantCommand(()-> {
      _elevatinator.setPositioninator(ElevatinatorConstants.kAlgaeNet);
      _claw.setTargetState(ClawState.PREP_NET);
      _ledinator.setWantedState(LEDinatorState.ALGAE);
      _calsificationinator.setTargetState(CalsificationinatorState.PICKUP);
    }), _drivetrain.setWantedTarget(LocationTarget.BARGE));
  }
  
  public Command algaeProcessor(LEDinatorState ledinatorState) {
    return Commands.parallel(new InstantCommand(()-> {
      _elevatinator.setPositioninator(ElevatinatorConstants.kAlgaeProcessor);
      _claw.setTargetState(ClawState.PREP_PROCESSOR);
      _ledinator.setWantedState(LEDinatorState.ALGAE);
      _calsificationinator.setTargetState(CalsificationinatorState.PICKUP);
    }), _drivetrain.setWantedTarget(LocationTarget.PROCESSOR));
  }

  public Command L1() {
    return Commands.parallel(new InstantCommand(
      ()->_calsificationinator.setTargetState(CalsificationinatorState.L4)
    ), _elevatinator.setWantedState(ElavatinatorState.HOLD)
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
      _claw.setTargetState(clawState);
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
    if(_elevatinator.getPositioninator() == ElevatinatorConstants.kAlgaeProcessor || _elevatinator.getPositioninator() == ElevatinatorConstants.kAlgaeNet) {
      return _claw.setWantedState(ClawState.SCORE);
    } else {
      return _calsificationinator.setWantedState(CalsificationinatorState.SCORE);
    }
  }

  public Command targetReef2Right()
  {
    return Commands.runOnce(()->
    {
      _drivetrain.setReefTargetFace(1);
      _drivetrain.setReefTargetSide(ReefConstants.kReefRight);
      _drivetrain.setWantedTargetNormal(LocationTarget.REEF);
      _drivetrain.targetReef(GeometryUtil::isRedAlliance);
      _drivetrain.setWantedStateNormal(DrivetrainState.TARGET_FOLLOW);
    });
  }

  public Command targetReef3Left() {
    return Commands.runOnce(() -> {
      _drivetrain.setReefTargetFace(2);
      _drivetrain.setReefTargetSide(ReefConstants.kReefLeft);
      _drivetrain.setWantedTargetNormal(LocationTarget.REEF);
      _drivetrain.targetReef(GeometryUtil::isRedAlliance);
      _drivetrain.setWantedStateNormal(DrivetrainState.TARGET_FOLLOW);
    });
  }

  public Command targetReef3Right() {
    return Commands.runOnce(() -> {
      _drivetrain.setReefTargetFace(2);
      _drivetrain.setReefTargetSide(ReefConstants.kReefRight);
      _drivetrain.setWantedTargetNormal(LocationTarget.REEF);
      _drivetrain.targetReef(GeometryUtil::isRedAlliance);
      _drivetrain.setWantedStateNormal(DrivetrainState.TARGET_FOLLOW);
    });
  }
  
  public Command targetReef4Right() {
    return Commands.runOnce(() -> {
      _drivetrain.setReefTargetFace(3);
      _drivetrain.setReefTargetSide(ReefConstants.kReefRight);
      _drivetrain.setWantedTargetNormal(LocationTarget.REEF);
      _drivetrain.targetReef(GeometryUtil::isRedAlliance);
      _drivetrain.setWantedStateNormal(DrivetrainState.TARGET_FOLLOW);
    });
  }

  public Command targetReef4Left() {
    return Commands.runOnce(() -> {
      _drivetrain.setReefTargetFace(3);
      _drivetrain.setReefTargetSide(ReefConstants.kReefLeft);
      _drivetrain.setWantedTargetNormal(LocationTarget.REEF);
      _drivetrain.targetReef(GeometryUtil::isRedAlliance);
      _drivetrain.setWantedStateNormal(DrivetrainState.TARGET_FOLLOW);
    });
  }

  public Command targetReef5Left() {
    return Commands.runOnce(() -> {
      _drivetrain.setReefTargetFace(4);
      _drivetrain.setReefTargetSide(ReefConstants.kReefLeft);
      _drivetrain.setWantedTargetNormal(LocationTarget.REEF);
      _drivetrain.targetReef(GeometryUtil::isRedAlliance);
      _drivetrain.setWantedStateNormal(DrivetrainState.TARGET_FOLLOW);
    });
  }

  public Command targetReef5Right() {
    return Commands.runOnce(() -> {
      _drivetrain.setReefTargetFace(4);
      _drivetrain.setReefTargetSide(ReefConstants.kReefRight);
      _drivetrain.setWantedTargetNormal(LocationTarget.REEF);
      _drivetrain.targetReef(GeometryUtil::isRedAlliance);
      _drivetrain.setWantedStateNormal(DrivetrainState.TARGET_FOLLOW);
    });
  }

  public Command targetReef1Right() {
    return Commands.runOnce(() -> {
      _drivetrain.setReefTargetFace(0);
      _drivetrain.setReefTargetSide(ReefConstants.kReefRight);
      _drivetrain.setWantedTargetNormal(LocationTarget.REEF);
      _drivetrain.targetReef(GeometryUtil::isRedAlliance);
      _drivetrain.setWantedStateNormal(DrivetrainState.TARGET_FOLLOW);
    });
  }

  public Command targetReef1Left() {
    return Commands.runOnce(() -> {
      _drivetrain.setReefTargetFace(0);
      _drivetrain.setReefTargetSide(ReefConstants.kReefLeft);
      _drivetrain.setWantedTargetNormal(LocationTarget.REEF);
      _drivetrain.targetReef(GeometryUtil::isRedAlliance);
      _drivetrain.setWantedStateNormal(DrivetrainState.TARGET_FOLLOW);
    });
  }

  public Command targetReef6Left() {
    return Commands.runOnce(() -> {
      _drivetrain.setReefTargetFace(5);
      _drivetrain.setReefTargetSide(ReefConstants.kReefLeft);
      _drivetrain.setWantedTargetNormal(LocationTarget.REEF);
      _drivetrain.targetReef(GeometryUtil::isRedAlliance);
      _drivetrain.setWantedStateNormal(DrivetrainState.TARGET_FOLLOW);
    });
  }

  public Command targetReef6Right() {
    return Commands.runOnce(() -> {
      _drivetrain.setReefTargetFace(5);
      _drivetrain.setReefTargetSide(ReefConstants.kReefRight);
      _drivetrain.setWantedTargetNormal(LocationTarget.REEF);
      _drivetrain.targetReef(GeometryUtil::isRedAlliance);
      _drivetrain.setWantedStateNormal(DrivetrainState.TARGET_FOLLOW);
    });
  }

  public Command autoDealgify(){
    return Commands.parallel(
      _claw.setWantedState(ClawState.DEALGIFY),
      _calsificationinator.setWantedState(CalsificationinatorState.PICKUP)
    );
  }


  public Command scoreSequence() {
    return Commands.sequence(
      Commands.waitUntil(_elevatinator::isReady),
      _calsificationinator.setWantedState(CalsificationinatorState.SCORE),
      Commands.waitUntil(_calsificationinator::hasScored),
      Commands.parallel(
        _claw.setWantedState(ClawState.IDLE),
        _calsificationinator.setWantedState(CalsificationinatorState.PICKUP)
        // _elevatinator.setWantedState(ElavatinatorState.HOME)
      )
    );
  }

  public Command scoreSequenceDontWait() {
    return Commands.sequence(
      _calsificationinator.setWantedState(CalsificationinatorState.SCORE),
      Commands.waitUntil(_calsificationinator::doesNotHaveCoralinator),
      Commands.parallel(
        _claw.setWantedState(ClawState.IDLE),
        _calsificationinator.setWantedState(CalsificationinatorState.PICKUP)
        // _elevatinator.setWantedState(ElavatinatorState.HOME)
      )
    );
  }



  public Command waitForCoral() {
    return Commands.sequence(
      Commands.waitUntil(_calsificationinator::hasCoralinator), 
      _calsificationinator.setWantedState(CalsificationinatorState.IDLE)
    );
  }

  public Command prepArm() {
    return _calsificationinator.setWantedState(CalsificationinatorState.IDLE);
  }

  public Command prepCoralL4() {
    return Commands.sequence(
      new InstantCommand(() -> _elevatinator.setPositioninator(ElevatinatorConstants.kL4Coral), _elevatinator),
      _elevatinator.setWantedState(ElavatinatorState.HOLD));
  }

  public Command prepCoralL3() {
    return Commands.sequence(
      new InstantCommand(() -> _elevatinator.setPositioninator(ElevatinatorConstants.kL3Coral), _elevatinator),
      _elevatinator.setWantedState(ElavatinatorState.HOLD));
  }
  public Command prepCoralL2() {
    return Commands.sequence(
      new InstantCommand(() -> _elevatinator.setPositioninator(ElevatinatorConstants.kL2Coral), _elevatinator),
      _elevatinator.setWantedState(ElavatinatorState.HOLD));
  }

  public Command prepDealgify() {
    return Commands.sequence(
      new InstantCommand(() -> _elevatinator.setPositioninator(ElevatinatorConstants.kL2Algae), _elevatinator),
      _elevatinator.setWantedState(ElavatinatorState.HOLD));
  }

  public Command autoHomeAlgae() {
    return Commands.parallel(_elevatinator.setWantedState(ElavatinatorState.HOMEWITHALGAE),
    _claw.setWantedState(ClawState.IDLE_WITH_ALGAE));
  }

  public Command homeElevator() {
    return _elevatinator.setWantedState(ElavatinatorState.HOME);
  }

  public Command homeElevatorWithAlgae()
  {
    return _elevatinator.setWantedState(ElavatinatorState.HOMEWITHALGAE);
  }

  public Command targetReef2Algae()
  {
    return Commands.runOnce(()-> {
      _drivetrain.setReefTargetFace(1);
      _drivetrain.setReefTargetSide(ReefConstants.kReefAlgae);
      _drivetrain.setWantedTargetNormal(LocationTarget.REEF);
      _drivetrain.targetReef(GeometryUtil::isRedAlliance);
      _drivetrain.setWantedStateNormal(DrivetrainState.TARGET_FOLLOW);

    });
  }

  public Command targetReef3Algae()
  {
    return Commands.runOnce(()-> {
      _drivetrain.setReefTargetFace(2);
      _drivetrain.setReefTargetSide(ReefConstants.kReefAlgae);
      _drivetrain.setWantedTargetNormal(LocationTarget.REEF);
      _drivetrain.targetReef(GeometryUtil::isRedAlliance);
      _drivetrain.setWantedStateNormal(DrivetrainState.TARGET_FOLLOW);

    });
  }

  public Command targetReef1Algae()
  {
    return Commands.runOnce(()-> {
      _drivetrain.setReefTargetFace(0);
      _drivetrain.setReefTargetSide(ReefConstants.kReefAlgae);
      _drivetrain.setWantedTargetNormal(LocationTarget.REEF);
      _drivetrain.targetReef(GeometryUtil::isRedAlliance);
      _drivetrain.setWantedStateNormal(DrivetrainState.TARGET_FOLLOW);

    });
  }
  
  public Command prepDealgifyHigh()
  {
    return Commands.sequence(
      new InstantCommand(() -> _elevatinator.setPositioninator(ElevatinatorConstants.kL3Algae), _elevatinator),
      _elevatinator.setWantedState(ElavatinatorState.HOLD));
  }

  public Command prepDealgifyLow()
  {
    return Commands.sequence(
      new InstantCommand(() -> _elevatinator.setPositioninator(ElevatinatorConstants.kL2Algae), _elevatinator),
      _elevatinator.setWantedState(ElavatinatorState.HOLD));
  }

  public Command retractClaw()
  {
    return _claw.setWantedState(ClawState.IDLE);
  }

  public Command prepBarge()
  {
    return Commands.sequence(
      new InstantCommand(() -> _elevatinator.setPositioninator(ElevatinatorConstants.kAlgaeNet), _elevatinator),
      _elevatinator.setWantedState(ElavatinatorState.HOLD));
  }

  public Command autoBarge()
  {
    return _claw.setWantedState(ClawState.PREP_NET_AUTO);
  }

  public Command scoreBarge()
  {
    return Commands.sequence(
      Commands.waitUntil(_elevatinator::isReady),
      Commands.runOnce(() -> SmartDashboard.putString("SCORE STEP", "Elevator ready")),
      Commands.waitUntil(_claw::isReadytoShootAuto),
      Commands.runOnce(() -> SmartDashboard.putString("SCORE STEP", "Claw ready to shoot")),
      _claw.setWantedState(ClawState.SCORE),
      Commands.runOnce(() -> SmartDashboard.putString("SCORE STEP", "Claw set to score state")),
      Commands.waitUntil(_claw::doesNotHaveAlgae),
      _claw.setWantedState(ClawState.IDLE),
      Commands.runOnce(() -> SmartDashboard.putString("SCORE STEP", "Claw set to IDLE"))
    );
  }

  public Command prepProcessor()
  {
    return Commands.sequence(
      new InstantCommand(() -> _elevatinator.setPositioninator(ElevatinatorConstants.kAlgaeProcessor), _elevatinator),
      _elevatinator.setWantedState(ElavatinatorState.HOLD));
  }

  public Command autoProcessor()
  {
    return _claw.setWantedState(ClawState.PREP_PROCESSOR);
  }

  public Command scoreProcessor()
  {
    return Commands.sequence(
      Commands.waitUntil(_elevatinator::isReady),
      Commands.runOnce(() -> SmartDashboard.putString("SCORE STEP", "Elevator ready")),
      Commands.waitUntil(_claw::isProcceorReady),
      Commands.runOnce(() -> SmartDashboard.putString("SCORE STEP", "Claw ready to shoot")),
      _claw.setWantedState(ClawState.SCORE),
      Commands.runOnce(() -> SmartDashboard.putString("SCORE STEP", "Claw set to score state")),
      Commands.waitUntil(_claw::doesNotHaveAlgae),
      _claw.setWantedState(ClawState.IDLE),
      Commands.runOnce(() -> SmartDashboard.putString("SCORE STEP", "Claw set to IDLE"))
    );
  }

  public Command RCHSpecial() {
    return Commands.sequence(
      Commands.runOnce(() -> {
        SmartDashboard.putBoolean("Auto Interrupted", false);
        SmartDashboard.putString("Current Auto Step", "Begin Test Auto");
      }),
      prepCoralL3(),
      _drivetrain.getPathCommand("Bottom Start to Reef 3").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 1 Right")),
      targetReef3Left(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
      // Commands.waitSeconds(1),
      Commands.waitUntil(_drivetrain::isNearTarget),
      prepCoralL4(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.31),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      scoreSequence(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Home Elevator")),
      homeElevator(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Run path to Coral Station")),
      _drivetrain.getPathCommand("Bottom Start to Coral TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting for Coral")),
      // waitForCoral(),
      // prepCoralL3(),
      // prepArm(),
      // _drivetrain.getPathCommand("Bottom Coral to Reef 5 Right TF").asProxy(),
      // Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 6 Right")),
      // targetReef5Right(),
      // Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
      // // Commands.waitSeconds(1),
      // // Commands.waitUntil(_drivetrain::isNearTarget),
      // prepCoralL4(),
      // Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.51),
      // Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      // Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      // Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      // Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      // scoreSequence(),
      // Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Home Elevator")),
      // homeElevator(),
      // _drivetrain.getPathCommand("Bottom Reef 5 Right to Coral TF").asProxy(),
      prepCoralL3(),
      prepArm(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Path Follow: Top Coral to Reef 6 TF")),
      _drivetrain.getPathCommand("Bottom Coral to Reef 4 TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 6 Left")),
      targetReef4Right(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
      // Commands.waitSeconds(1),
      Commands.waitUntil(_drivetrain::isNearTarget),
      prepCoralL4(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.38),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      scoreSequence(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Home Elevator")),
      homeElevator(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Run path to Coral Station")),
      _drivetrain.getPathCommand("Bottom Reef 4 Right to Coral TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting for Coral")),
      // waitForCoral(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Path Follow: Top Coral to Reef 6 TF")),
      // waitForCoral(),
      prepCoralL3(),
      prepArm(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Path Follow: Top Coral to Reef 6 TF")),
      _drivetrain.getPathCommand("Bottom Coral to Reef 4 Left TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 6 Right")),
      targetReef4Left(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
      // Commands.waitSeconds(1),
      Commands.waitUntil(_drivetrain::isNearTarget),
      prepCoralL4(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.41),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      scoreSequence(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Home Elevator")),
      homeElevator(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Run path to Coral Station")),
      _drivetrain.getPathCommand("Bottom Reef 4 Left to Coral TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting for Coral")),
      // waitForCoral(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Done")),
      prepCoralL3(),
      prepArm(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Path Follow: Top Coral to Reef 6 TF")),
      _drivetrain.getPathCommand("Bottom Coral to Reef 3 Right TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 6 Right")),
      targetReef3Right(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
      // Commands.waitSeconds(1),
      Commands.waitUntil(_drivetrain::isNearTarget),
      prepCoralL4(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.41),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      scoreSequenceDontWait(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Home Elevator")),
      homeElevator()
    ).handleInterrupt(() -> {
      SmartDashboard.putBoolean("Auto Interrupted", true);
    });
    // return Commands.sequence(
    //   Commands.runOnce(() -> {
    //     SmartDashboard.putBoolean("Auto Interrupted", false);
    //     SmartDashboard.putString("Current Auto Step", "Begin Test Auto");
    //   }),
    //   _drivetrain.getPathCommand("Bottom Start to Reef 3").asProxy(),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 3 Left")),
    //   targetReef3Left(),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
    //   // Commands.waitSeconds(1),
    //   Commands.waitUntil(_drivetrain::isNearTarget).withTimeout(.75),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
    //   prepCoralL4(),
    //   Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.75),
    //   Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
    //   Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
    //   scoreSequence(),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Home Elevator")),
    //   homeElevator(),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Run path to Coral Station")),
    //   _drivetrain.getPathCommand("Bottom Start to Coral TF").asProxy(),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting for Coral")),
    //   waitForCoral(),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Path Follow: Bottom Coral to Reef 4 Right TF")),
    //   _drivetrain.getPathCommand("Bottom Coral to Reef 4 TF").asProxy(),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 4 Right")),
    //   targetReef4Right(),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
    //   // Commands.waitSeconds(1),
      
    //   Commands.waitUntil(_drivetrain::isNearTarget).withTimeout(.75),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
    //   prepCoralL4(),
    //   Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.75),
    //   Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
    //   Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
    //   scoreSequence(),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Home Elevator")),
    //   homeElevator(),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Run path to Coral Station")),
    //   _drivetrain.getPathCommand("Bottom Reef 4 Right to Coral TF").asProxy(),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting for Coral")),
    //   waitForCoral(),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Path Follow: Bottom Coral to Reef 4 TF")),
    //   _drivetrain.getPathCommand("Bottom Coral to Reef 4 Left TF").asProxy(),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 4 Left")),
    //   targetReef4Left(),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
    //   // Commands.waitSeconds(1),
    //   Commands.waitUntil(_drivetrain::isNearTarget).withTimeout(.75),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
    //   prepCoralL4(),
    //   Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.75),
    //   Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
    //   Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
    //   scoreSequence(),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Home Elevator")),
    //   homeElevator(),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Run path to Coral Station")),
    //   _drivetrain.getPathCommand("Bottom Reef 4 Left to Coral TF").asProxy(),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting for Coral")),
    //   waitForCoral(),
    //   Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Done"))
    // ).handleInterrupt(() -> {
    //   SmartDashboard.putBoolean("Auto Interrupted", true);
    // });
    
  }

  public Command LGHPSpecial() {
    return Commands.sequence(
      Commands.runOnce(() -> {
        SmartDashboard.putBoolean("Auto Interrupted", false);
        SmartDashboard.putString("Current Auto Step", "Begin Test Auto");
      }),
      prepCoralL3(),
      _drivetrain.getPathCommand("Top Start to Reef 1 TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 1 Right")),
      targetReef1Right(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
      // Commands.waitSeconds(1),
      Commands.waitUntil(_drivetrain::isNearTarget),
      prepCoralL4(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.31),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      scoreSequence(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Home Elevator")),
      homeElevator(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Run path to Coral Station")),
      _drivetrain.getPathCommand("Top Start to Coral TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting for Coral")),
      // waitForCoral(),
      // prepCoralL3(),
      // prepArm(),
      // _drivetrain.getPathCommand("Top Coral to Reef 5 Left TF").asProxy(),
      // Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 6 Right")),
      // targetReef5Left(),
      // Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
      // // Commands.waitSeconds(1),
      // // Commands.waitUntil(_drivetrain::isNearTarget),
      // prepCoralL4(),
      // Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.51),
      // Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      // Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      // Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      // Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      // scoreSequence(),
      // Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Home Elevator")),
      // homeElevator(),
      // _drivetrain.getPathCommand("Top Reef 5 Left to Coral TF").asProxy(),
      prepCoralL3(),
      prepArm(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Path Follow: Top Coral to Reef 6 TF")),
      _drivetrain.getPathCommand("Top Coral to Reef 6 TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 6 Left")),
      targetReef6Left(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
      // Commands.waitSeconds(1),
      Commands.waitUntil(_drivetrain::isNearTarget),
      prepCoralL4(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.41),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      scoreSequence(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Home Elevator")),
      homeElevator(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Run path to Coral Station")),
      _drivetrain.getPathCommand("Top Reef 6 Left to Coral TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting for Coral")),
      // waitForCoral(),
      prepCoralL3(),
      prepArm(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Path Follow: Top Coral to Reef 6 TF")),
      _drivetrain.getPathCommand("Top Coral to Reef 6 Right TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 6 Right")),
      targetReef6Right(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
      // Commands.waitSeconds(1),
      Commands.waitUntil(_drivetrain::isNearTarget),
      prepCoralL4(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.41),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      scoreSequence(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Home Elevator")),
      homeElevator(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Run path to Coral Station")),
      _drivetrain.getPathCommand("Top Reef 6 Right to Coral TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting for Coral")),
      // waitForCoral(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Done")),
      prepCoralL3(),
      prepArm(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Path Follow: Top Coral to Reef 6 TF")),
      _drivetrain.getPathCommand("Top Coral to Reef 1 Left TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 6 Right")),
      targetReef1Left(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
      // Commands.waitSeconds(1),
      Commands.waitUntil(_drivetrain::isNearTarget),
      prepCoralL4(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.41),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      scoreSequenceDontWait(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Home Elevator")),
      homeElevator()
    ).handleInterrupt(() -> {
      SmartDashboard.putBoolean("Auto Interrupted", true);
    });
  }

  public Command left3CoralAlgae() {
    return Commands.sequence(
      Commands.runOnce(() -> {
        SmartDashboard.putBoolean("Auto Interrupted", false);
        SmartDashboard.putString("Current Auto Step", "Begin Test Auto");
      }),
      _drivetrain.getPathCommand("Top Start to Reef 1 TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 1 Right")),
      targetReef1Right(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
      // Commands.waitSeconds(1),
      Commands.waitUntil(_drivetrain::isNearTarget),
      prepCoralL4(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.75),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      scoreSequence(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Home Elevator")),
      homeElevator(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Run path to Coral Station")),
      _drivetrain.getPathCommand("Top Start to Coral TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting for Coral")),
      waitForCoral().withTimeout(1),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Path Follow: Top Coral to Reef 6 TF")),
      _drivetrain.getPathCommand("Top Coral to Reef 6 TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 6 Left")),
      targetReef6Left(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
      // Commands.waitSeconds(1),
      Commands.waitUntil(_drivetrain::isNearTarget),
      prepCoralL4(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.75),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      scoreSequence(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Home Elevator")),
      homeElevator(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Run path to Coral Station")),
      _drivetrain.getPathCommand("Top Reef 6 Left to Coral TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting for Coral")),
      waitForCoral().withTimeout(1),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Path Follow: Top Coral to Reef 6 TF")),
      _drivetrain.getPathCommand("Top Coral to Reef 6 Right TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 6 Right")),
      targetReef6Right(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
      // Commands.waitSeconds(1),
      Commands.waitUntil(_drivetrain::isNearTarget),
      prepCoralL4(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.75),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      scoreSequence(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Home Elevator")),
      prepDealgify(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Run path to Coral Station")),
      _drivetrain.getPathCommand("Top Coral to Algae").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting for Coral")),
      autoDealgify(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Done")),
      Commands.waitUntil(_claw::hasAlgae),
      autoHomeAlgae()
      ,
      _drivetrain.getPathCommand("Top Reef 5 Algae to Barge TF").asProxy()
    ).handleInterrupt(() -> {
      SmartDashboard.putBoolean("Auto Interrupted", true);
    });
  }

  public Command right3CoralAlgae() {
    return Commands.sequence(
      Commands.runOnce(() -> {
        SmartDashboard.putBoolean("Auto Interrupted", false);
        SmartDashboard.putString("Current Auto Step", "Begin Test Auto");
      }),
      _drivetrain.getPathCommand("Bottom Start to Reef 3").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 1 Right")),
      targetReef3Left(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
      // Commands.waitSeconds(1),
      Commands.waitUntil(_drivetrain::isNearTarget),
      prepCoralL4(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.75),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      scoreSequence(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Home Elevator")),
      homeElevator(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Run path to Coral Station")),
      _drivetrain.getPathCommand("Bottom Start to Coral TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting for Coral")),
      waitForCoral().withTimeout(1),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Path Follow: Top Coral to Reef 6 TF")),
      _drivetrain.getPathCommand("Bottom Coral to Reef 4 TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 6 Left")),
      targetReef4Right(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
      // Commands.waitSeconds(1),
      Commands.waitUntil(_drivetrain::isNearTarget),
      prepCoralL4(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.75),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      scoreSequence(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Home Elevator")),
      homeElevator(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Run path to Coral Station")),
      _drivetrain.getPathCommand("Bottom Reef 4 Right to Coral TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting for Coral")),
      waitForCoral().withTimeout(1),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Path Follow: Top Coral to Reef 6 TF")),
      _drivetrain.getPathCommand("Bottom Coral to Reef 4 Left TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 6 Right")),
      targetReef4Left(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
      // Commands.waitSeconds(1),
      Commands.waitUntil(_drivetrain::isNearTarget),
      prepCoralL4(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.75),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      scoreSequence(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Home Elevator")),
      prepDealgify(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Run path to Coral Station")),
      _drivetrain.getPathCommand("Bottom Coral to Algae").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting for Coral")),
      autoDealgify(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Done")),
      Commands.waitUntil(_claw::hasAlgae),
      autoHomeAlgae()
    ).handleInterrupt(() -> {
      SmartDashboard.putBoolean("Auto Interrupted", true);
    });
  }

  public Command getAutoMiddle()
  {
    return Commands.sequence(
      Commands.runOnce(()-> {
        SmartDashboard.putBoolean("Auto Interrupted", false);
        SmartDashboard.putString("Current Auto Step", "Begin Test Auto");
      }),
      prepCoralL3(),
      _drivetrain.getPathCommand("Middle to Reef 2").asProxy(),
      prepCoralL4(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 2 Left")),
      targetReef2Right(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.75),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      scoreSequence(),

      prepDealgifyLow(),
      autoDealgify(),
      targetReef2Algae(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.75),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.waitUntil(_claw::hasAlgae),
      //retractClaw(),
      prepBarge(),
      autoBarge(),
      _drivetrain.getPathCommand("Dealgify to barge 1").asProxy(),
      //homeElevatorWithAlgae(),
      
      scoreBarge(),
      //homeElevator(),
      prepDealgifyHigh(),
      autoDealgify(),
      _drivetrain.getPathCommand("Barge 1 to reef 3").asProxy(),
      
      targetReef3Algae(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.75),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.waitUntil(_claw::hasAlgae),
      prepBarge(),
      autoBarge(),
      _drivetrain.getPathCommand("Dealgify to barge 2").asProxy(),
      
      //retractClaw(),
      //homeElevatorWithAlgae(),
      
      scoreBarge(),
      //homeElevator(),
      prepDealgifyHigh(),
      autoDealgify(),

      _drivetrain.getPathCommand("Barge 2 to reef 1").asProxy(),
      
      targetReef1Algae(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.75),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.waitUntil(_claw::hasAlgae),
      prepBarge(),
      autoBarge(),
      _drivetrain.getPathCommand("Dealgify to barge 3").asProxy(),
      
      // retractClaw(),
      // homeElevatorWithAlgae(),
      
      scoreBarge(),
      homeElevator()
    ).handleInterrupt(() -> {
      SmartDashboard.putBoolean("Auto Interrupted", true);
    });
  }

  public Command processorAuto()
  {
    return Commands.sequence(
      Commands.runOnce(()-> {
        SmartDashboard.putBoolean("Auto Interrupted", false);
        SmartDashboard.putString("Current Auto Step", "Begin Test Auto");
      }),
      prepCoralL3(),
      _drivetrain.getPathCommand("Processor - Middle to Reef 2").asProxy(),
      prepCoralL4(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 2 Left")),
      targetReef2Right(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.75),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      scoreSequence(),

      prepDealgifyLow(),
      autoDealgify(),
      targetReef2Algae(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.75),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.waitUntil(_claw::hasAlgae),
      retractClaw(),
      homeElevatorWithAlgae(),
      prepProcessor(),
      autoProcessor(),
      _drivetrain.getPathCommand("reef 2 to processor").asProxy(),
      scoreProcessor(),
      homeElevator(),


      _drivetrain.getPathCommand("processor to reef 3").asProxy(),
      prepDealgifyHigh(),
      autoDealgify(),
      targetReef3Algae(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.75),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.waitUntil(_claw::hasAlgae),
      retractClaw(),
      homeElevatorWithAlgae(),
      _drivetrain.getPathCommand("reef 3 to processor").asProxy(),
      prepProcessor(),
      autoProcessor(),
      scoreProcessor(),
      homeElevator(),

      _drivetrain.getPathCommand("processor to reef 1").asProxy(),
      prepDealgifyHigh(),
      autoDealgify(),
      targetReef1Algae(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.75),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.waitUntil(_claw::hasAlgae),
      retractClaw(),
      homeElevatorWithAlgae(),
      _drivetrain.getPathCommand("reef 1 to processor").asProxy(),
      prepProcessor(),
      autoProcessor(),
      scoreProcessor(),
      homeElevator()
    ).handleInterrupt(() -> {
      SmartDashboard.putBoolean("Auto Interrupted", true);
    });
  }

  public Command right3CoralPrep() {
    return Commands.sequence(
      Commands.runOnce(() -> {
        SmartDashboard.putBoolean("Auto Interrupted", false);
        SmartDashboard.putString("Current Auto Step", "Begin Test Auto");
      }),
      Commands.parallel(_drivetrain.getPathCommand("Bottom Start to Reef 3").asProxy(), Commands.waitSeconds(1).andThen(prepCoralL4())),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 1 Right")),
      targetReef3Left(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
      // Commands.waitSeconds(1),
      Commands.waitUntil(_drivetrain::isNearTarget),
      // prepCoralL4(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.75),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      scoreSequence(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Home Elevator")),
      homeElevator(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Run path to Coral Station")),
      _drivetrain.getPathCommand("Bottom Start to Coral TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting for Coral")),
      waitForCoral(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Path Follow: Top Coral to Reef 6 TF")),
      Commands.parallel(_drivetrain.getPathCommand("Bottom Coral to Reef 4 TF").asProxy(), Commands.waitSeconds(1).andThen(prepCoralL4())),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 6 Left")),
      targetReef4Right(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
      // Commands.waitSeconds(1),
      Commands.waitUntil(_drivetrain::isNearTarget),
      // prepCoralL4(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.75),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      scoreSequence(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Home Elevator")),
      homeElevator(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Run path to Coral Station")),
      _drivetrain.getPathCommand("Bottom Reef 4 Right to Coral TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting for Coral")),
      waitForCoral(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Path Follow: Top Coral to Reef 6 TF")),
      Commands.parallel(_drivetrain.getPathCommand("Bottom Coral to Reef 4 Left TF").asProxy(), Commands.waitSeconds(1).andThen(prepCoralL4())),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 6 Right")),
      targetReef4Left(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
      // Commands.waitSeconds(1),
      Commands.waitUntil(_drivetrain::isNearTarget),
      // prepCoralL4(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.75),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      scoreSequence(),
      homeElevator(),
      _drivetrain.getPathCommand("Bottom Reef 4 Left to Coral TF").asProxy(),
      waitForCoral(),
      _drivetrain.getPathCommand("Bottom Coral to Reef 3 Right TF").asProxy()
    ).handleInterrupt(() -> {
      SmartDashboard.putBoolean("Auto Interrupted", true);
    });
  }

  public Command left3CoralPrep() {
    return Commands.sequence(
      Commands.runOnce(() -> {
        SmartDashboard.putBoolean("Auto Interrupted", false);
        SmartDashboard.putString("Current Auto Step", "Begin Test Auto");
      }),
      Commands.parallel(_drivetrain.getPathCommand("Top Start to Reef 1 TF").asProxy(),Commands.waitSeconds(1).andThen(prepCoralL4())),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 1 Right")),
      targetReef1Right(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
      // Commands.waitSeconds(1),
      Commands.waitUntil(_drivetrain::isNearTarget),
      // prepCoralL4(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.75),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      scoreSequence(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Home Elevator")),
      homeElevator(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Run path to Coral Station")),
      _drivetrain.getPathCommand("Top Start to Coral TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting for Coral")),
      waitForCoral(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Path Follow: Top Coral to Reef 6 TF")),
      Commands.parallel(_drivetrain.getPathCommand("Top Coral to Reef 6 TF").asProxy(),Commands.waitSeconds(1).andThen(prepCoralL4())),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 6 Left")),
      targetReef6Left(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
      // Commands.waitSeconds(1),
      Commands.waitUntil(_drivetrain::isNearTarget),
      // prepCoralL4(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.75),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      scoreSequence(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Home Elevator")),
      homeElevator(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Run path to Coral Station")),
      _drivetrain.getPathCommand("Top Reef 6 Left to Coral TF").asProxy(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting for Coral")),
      waitForCoral(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Path Follow: Top Coral to Reef 6 TF")),
      Commands.parallel(_drivetrain.getPathCommand("Top Coral to Reef 6 Right TF").asProxy(),Commands.waitSeconds(1).andThen(prepCoralL4())),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Target Reef 6 Right")),
      targetReef6Right(),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Waiting 1 second")),
      // Commands.waitSeconds(1),
      Commands.waitUntil(_drivetrain::isNearTarget),
      // prepCoralL4(),
      Commands.waitUntil(_drivetrain::isAtTarget).withTimeout(.75),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.IDLE)),
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.AUTO)),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Prep Coral L4")),
      Commands.runOnce(() -> SmartDashboard.putString("Current Auto Step", "Score Sequence")),
      scoreSequence(),
      homeElevator(),
      _drivetrain.getPathCommand("Top Reef 6 Right to Coral TF").asProxy(),
      waitForCoral(),
      _drivetrain.getPathCommand("Top Coral to Reef 1 Left TF").asProxy()
    ).handleInterrupt(() -> {
      SmartDashboard.putBoolean("Auto Interrupted", true);
    });
  }
}
