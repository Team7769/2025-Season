package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.revrobotics.spark.SparkBase;
import frc.robot.statemachine.StateBasedSubsystem;
import frc.robot.enums.RollerState;
public class KitbotRoller extends StateBasedSubsystem<RollerState>{



private SparkMax _motor;

    public KitbotRoller (){
        _motor = new SparkMax(15, MotorType.kBrushed);
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false)
            .idleMode(IdleMode.kCoast);

        _motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        _currentState = RollerState.AUTO;
        _previousState = RollerState.ROLL;

    }
    public Command stop(){
        return run(() -> _motor.set(0));


    }
    public Command roll(){
        return run(() -> _motor.set(0.5));
    }
    public Command handleCurrentState(){
        switch (_currentState){
            case STOP:
            return stop();

            case ROLL:
            return roll();

            default:
            return stop();
        }
    }
    
    @Override
    public void periodic(){
        if (_currentState != RollerState.AUTO){
            handleCurrentState().schedule();
        }
    }

    public SequentialCommandGroup score(){
        // return new SequentialCommandGroup(setWantedState(RollerState.ROLL), new WaitCommand(0.5), setWantedState(RollerState.STOP));
        return new SequentialCommandGroup(roll().withTimeout(.1), new WaitCommand(0.5), stop().withTimeout(.1));
    }
}
