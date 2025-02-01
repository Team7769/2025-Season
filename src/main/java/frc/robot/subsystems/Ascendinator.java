package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.enums.CageState;
import frc.robot.statemachine.StateBasedSubsystem;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer.DebounceType;

public class Ascendinator extends StateBasedSubsystem<CageState> { 
    private TalonFX PrimaryinatorAscendinator = new TalonFX(Constants.SubsystemConstants.kPrimaryinatorAscendinatorID);
    private TalonFX SecondaryinatorAscendinator = new TalonFX(Constants.SubsystemConstants.kSecondaryinatorAscendinatorID);
    private TalonFXConfiguration Configinator = new TalonFXConfiguration();

     

     private FeedbackConfigs FeedBackConfig = Configinator.Feedback;
    
     public MotionMagicConfigs MotionMagicinator = Configinator.MotionMagic;
    
     private Follower _follower;
    
     public boolean DetecTinator;

    public Ascendinator (){ 
    Slot0Configs slot0 = Configinator.Slot0;
    slot0.kS = 0.25; 
    slot0.kV = 0.12; 
    slot0.kA = 0.01; 
    slot0.kP = 60; 
    slot0.kI = 0; 
    slot0.kD = 0.5;
        
    FeedBackConfig.SensorToMechanismRatio = 3.5;
        
    
        
    MotionMagicConfigs MotionMagicinator = Configinator.MotionMagic;
    MotionMagicinator.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5))
    .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
    .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
    
      
      _follower = new Follower(16, false);
        SecondaryinatorAscendinator.setControl(_follower);
    }

    public void handleCurrentState(){
            switch (_currentState){
                case IDLE:
                    PrimaryinatorAscendinator.setPosition(Rotation.of(0.5));
                    break;
                
                case DEPLOY:
                    PrimaryinatorAscendinator.setPosition(Rotation.of(0.25));
                    break;
                    
                
                case ASCEND:
                    PrimaryinatorAscendinator.setPosition(Rotation.of(0.15));
                    break;
                  
                default:
                PrimaryinatorAscendinator.setPosition(Rotation.of(0.5));;
                    break;
            }
        }
}
