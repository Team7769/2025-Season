// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.enums.CageState;
import frc.robot.enums.ClawState;
import frc.robot.enums.DrivetrainState;
import frc.robot.enums.LocationTarget;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.*;

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
import frc.robot.enums.RollerState;
import frc.robot.subsystems.Ascendinator;
import frc.robot.subsystems.KitbotRoller;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController _driverController;
  private final CommandXboxController _operatorController;
  private final Drivetrain _drivetrain;
  private final Claw _claw;
  private final Ascendinator _ascendinator;
  private final KitbotRoller _roller;
  private final Vision _vision;
  private final SendableChooser<Command> _autoChooser;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    _claw = new Claw();
    _ascendinator = new Ascendinator();
    _roller = new KitbotRoller();
    _driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    _operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
    _vision = new Vision();
    _drivetrain = new Drivetrain(_driverController, _vision);
    // Configure the trigger bindings
    new EventTrigger("Initialize").onTrue(_drivetrain.setWantedState(DrivetrainState.AUTO).withTimeout(.05));
    new EventTrigger("KitBotScore").onTrue(_roller.score());
    _autoChooser = AutoBuilder.buildAutoChooser("Test Auto");
    SmartDashboard.putData("AutoChooser", _autoChooser);
    configureBindings();
    SmartDashboard.putData("current command", CommandScheduler.getInstance());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    _drivetrain.setDefaultCommand(
      _drivetrain.applyRequest(() -> 
        _drivetrain.drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
    ));

    
    //_driverController.rightTrigger().onTrue(_roller.setWantedState(RollerState.ROLL)).onFalse(_roller.setWantedState(RollerState.STOP));
    _driverController.rightTrigger().onTrue(_claw.setWantedState(ClawState.SCORE)).onFalse(_claw.setWantedState(ClawState.IDLE));
    _driverController.start().onTrue(_drivetrain.resetGyro());
    _driverController.leftBumper().onTrue(_drivetrain.setWantedState(DrivetrainState.TARGET_FOLLOW))
    .onFalse(_drivetrain.setWantedState(DrivetrainState.OPEN_LOOP));

    new Trigger (_ascendinator::hasCage).negate().and(_driverController.back()).onTrue(_ascendinator.setWantedState(CageState.DEPLOY));
    new Trigger(_ascendinator::hasCage).and(_driverController.back()).onTrue(_ascendinator.setWantedState(CageState.ASCEND));

    new Trigger(_claw::hasAlgae).negate().and(_driverController.rightBumper()).onTrue(_claw.setWantedState(ClawState.FLOOR_INTAKE)).onFalse(_claw.setWantedState(ClawState.IDLE));
    
    // new Trigger(DriverStation::isAutonomousEnabled).onTrue(_drivetrain.setWantedState(DrivetrainState.AUTO));
    new Trigger(DriverStation::isTeleopEnabled).onTrue(_drivetrain.setWantedState(DrivetrainState.OPEN_LOOP));

    _operatorController.a().onTrue(_claw.setWantedState(ClawState.DEALGIFY)).onFalse(_claw.setWantedState(ClawState.IDLE));
    _operatorController.y().onTrue(_claw.setWantedState(ClawState.PREP_NET));
    _operatorController.x().onTrue(_claw.setWantedState(ClawState.PREP_PROCESSOR));

    // _operatorController.povRight().onTrue(new InstantCommand(()-> _drivetrain.setReefTargetSideRight(0)));
    // _operatorController.povUp().onTrue(new InstantCommand(()-> _drivetrain.setReefTargetSideRight(1)));
    // _operatorController.povLeft().onTrue(new InstantCommand(()-> _drivetrain.setReefTargetSideRight(2)));
    // _operatorController.x().onTrue(new InstantCommand(() -> _drivetrain.targetNextReefFace()));
    // _operatorController.b().onTrue(_drivetrain.setWantedTarget(LocationTarget.REEF));
    // _operatorController.y().onTrue(_drivetrain.setWantedTarget(LocationTarget.CORAL_SOURCE));
    // _operatorController.back().onTrue(_drivetrain.setWantedTarget(LocationTarget.CAGE));
    // _operatorController.a().onTrue(_drivetrain.setWantedTarget(LocationTarget.PROCESSOR));
    // _operatorController.rightBumper().onTrue(_drivetrain.setWantedTarget(LocationTarget.BARGE));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return _autoChooser.getSelected();
    
    //reference for auto for other teams
    /*return new SequentialCommandGroup(new WaitCommand(3),
                _drivetrain.applyRequest(() -> _drivetrain.drive.withVelocityX(-1)), 
                new WaitCommand(3), 
                _drivetrain.applyRequest(() -> _drivetrain.drive.withVelocityX(0)));*/

  }
}
