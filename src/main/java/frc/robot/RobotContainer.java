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
    // _autoChooser.addOption("Test Auto TF - Custom", getTestAuto());
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

    _driverController.rightTrigger().onTrue(scoreinator()).onFalse(goHomeinator());
    //_driverController.leftTrigger().onTrue(doinator(null));
    _driverController.leftTrigger().onTrue(Commands.defer(this::doThing, Set.of()));

    _driverController.start().onTrue(_drivetrain.resetGyro());
    new Trigger(_claw::hasAlgae).and(_driverController.a()).onTrue(goHomeinatorWithAlgae());
    new Trigger(_claw::doesNotHaveAlgae).and(_driverController.a()).onTrue(goHomeinator());
    _driverController.b().onTrue(doinator(null));
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

    

    _driverController.rightBumper().onTrue(
      Commands.parallel(
        new InstantCommand(() -> _elevatinator.setPositioninator(ElevatinatorConstants.kAlgaePickup)), 
      _elevatinator.setWantedState(ElavatinatorState.HOLD), 
      _claw.setWantedState(ClawState.FLOOR_INTAKE), 
      _ledinator.setWantedState(LEDinatorState.ALGAE), 
      _calsificationinator.setWantedState(CalsificationinatorState.PICKUP)));
      
    new Trigger(_claw::hasAlgae).onTrue(goHomeinatorWithAlgae());
    new Trigger(_driverController.rightBumper().negate()).and(_claw::doesNotHaveAlgae).onTrue(goHomeinatorForFloorPickup());

    if (!_ascendinator.isReady()){
      new Trigger(_calsificationinator::hasCoralinator).and(DriverStation::isTeleopEnabled)
      .onTrue(_calsificationinator.setWantedState(CalsificationinatorState.IDLE))
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
    
    _operatorController.x().onTrue(dealgifyLow(null));
    _operatorController.b().onTrue(dealgifyHigh(null));    
    _operatorController.y().onTrue(algaeNet(null));
    _operatorController.a().onTrue(algaeProcessor(null));
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
          .until(_driverController.a()).andThen(Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.OPEN_LOOP)));
        } else {
          return 
          Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.TARGET_FOLLOW))
          .andThen(Commands.waitUntil(_drivetrain::isAtTarget))
          .andThen(doinator(null))
          .andThen(Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.OPEN_LOOP)))
          .handleInterrupt(() -> System.out.println("Interrupted doThing."))
          .until(_driverController.a()).andThen(Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.OPEN_LOOP)));
        }
      } else if (_claw.getTargetState() == ClawState.DEALGIFY){
        SmartDashboard.putString("Current Action", "Claw is dealgifying for DoThing");
        return Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.TARGET_FOLLOW))
        .andThen(Commands.waitUntil(_drivetrain::isAtTarget))
        .andThen(doinator(null))
        .andThen(Commands.waitUntil(_claw::hasAlgae))
        .andThen(goHomeinatorWithAlgae().alongWith(Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.OPEN_LOOP))))
        .handleInterrupt(() -> System.out.println("Interrupted doThing."))
        .until(_driverController.a()).andThen(Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.OPEN_LOOP))
        );
      }
      SmartDashboard.putString("Current Action", "Coral for DoThing");
      return 
      Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.TARGET_FOLLOW))
      .andThen(Commands.waitUntil(_drivetrain::isAtTarget))
      .andThen(doinator(null))
      .andThen(Commands.waitUntil(_elevatinator::isReady))
      .andThen(scoreSequence())
      .andThen(goHomeinator().alongWith(Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.OPEN_LOOP))))
      .handleInterrupt(() -> System.out.println("Interrupted doThing."))
      .until(_driverController.a()).andThen(goHomeinator().alongWith(Commands.runOnce(() -> _drivetrain.setWantedStateNormal(DrivetrainState.OPEN_LOOP)))
      );
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

  public Command goHomeinatorWithAlgae() {
    return Commands.parallel(
        new InstantCommand(() -> {
          _elevatinator.setHoldAlgaePosition(false);
          _elevatinator.setPositioninator(ElevatinatorConstants.kAlgaeHold);
        }
        ),
        _claw.setWantedState(ClawState.IDLE_WITH_ALGAE),
        _elevatinator.setWantedState(ElavatinatorState.HOMEWITHALGAE),
        _calsificationinator.setWantedState(CalsificationinatorState.PICKUP));
  }

  public Command doinator(LEDinatorState ledinatorState) {
    if(_drivetrain.getCurrentTarget() != LocationTarget.PROCESSOR) {
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
      _calsificationinator.setTargetState(CalsificationinatorState.PICKUP);
    }), _drivetrain.setWantedTarget(LocationTarget.BARGE));
  }
  
  public Command algaeProcessor(LEDinatorState ledinatorState) {
    return Commands.parallel(new InstantCommand(()-> {
      _elevatinator.setPositioninator(ElevatinatorConstants.kAlgaeProcessor);
      _claw.setTargetState(ClawState.PREP_PROCESSOR);
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
    return Commands.parallel(_claw.setWantedState(ClawState.SCORE), _calsificationinator.setWantedState(CalsificationinatorState.SCORE));
  }

  public Command targetReef() {
    var targetReefCommand = Commands.sequence(
      Commands.runOnce(() -> {
        _drivetrain.setReefTargetFace(2);
        _drivetrain.setReefTargetSide(ReefConstants.kReefLeft);
        _drivetrain.setWantedTargetNormal(LocationTarget.REEF);
        _drivetrain.targetReef(GeometryUtil::isRedAlliance);
        _drivetrain.setWantedStateNormal(DrivetrainState.TARGET_FOLLOW);
      }),
      Commands.waitSeconds(1)
    );

    //targetReefCommand.addRequirements(_drivetrain);
    return targetReefCommand;
  }

  public Command scoreSequence() {
    return Commands.sequence(
      Commands.waitUntil(_elevatinator::isReady),
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

  public Command prepCoralL4() {
    return Commands.sequence(
      new InstantCommand(() -> _elevatinator.setPositioninator(ElevatinatorConstants.kL4Coral), _elevatinator),
      _elevatinator.setWantedState(ElavatinatorState.HOLD));
  }

  public Command getTestAuto() {
    return Commands.sequence(
      Commands.runOnce(() -> System.out.println("Begin Test Auto")),
      _drivetrain.getPathCommand("Bottom Start to Reef 3").asProxy(),
      Commands.runOnce(() -> System.out.println("Target Reef")),
      targetReef(),
      Commands.runOnce(() -> System.out.println("Prep Coral L4")),
      prepCoralL4(),
      Commands.runOnce(() -> System.out.println("Score Sequence")),
      scoreSequence(),
      _drivetrain.getPathCommand("Bottom Start to Coral TF").asProxy(),
      waitForCoral(),
      Commands.runOnce(() -> System.out.println("Done"))
    ).handleInterrupt(() -> System.out.println("Auto interrupted."));
  }
}
