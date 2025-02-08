package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.enums.CageState;
import frc.robot.enums.CalsificationinatorState;
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

public class Calsificationinator extends StateBasedSubsystem<CalsificationinatorState> {
    private TalonFX _suckinator = new TalonFX(Constants.CalsificationinatorConstants.kSuckinatorScoreinatorID);
    
    private TalonFX _pivotinator = new TalonFX(Constants.CalsificationinatorConstants.kPivotinatorID);

    private boolean _hasCoralinator;
   
    private TalonFXConfiguration _pivotConfig;
    private FeedbackConfigs _pivotFeedbackConfigs;
    private final MotionMagicVoltage _magicinator = new MotionMagicVoltage(0);

    private DigitalInput _calsificationDetectinator;
 
    private Debouncer _calsificationDebouncinator;

    private Debouncer _calsificationDebouncinatorTwo;




    

   
    public Calsificationinator(){
        _calsificationDetectinator = new DigitalInput(Constants.CalsificationinatorConstants.kCalsificationDetectinatorChanel);
        _pivotConfig = new TalonFXConfiguration();
        _pivotFeedbackConfigs = _pivotConfig.Feedback;
        _pivotFeedbackConfigs.SensorToMechanismRatio = 1;

        _calsificationDebouncinator = new Debouncer(0);
        _calsificationDebouncinatorTwo = new Debouncer(0);
        Slot0Configs slot0 = _pivotConfig.Slot0;
        slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = 0.5; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output
        slot0.kG = 0;

        MotionMagicConfigs _pivotMotionMagic = _pivotConfig.MotionMagic;
        _pivotMotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5)) // 5 (mechanism) rotations per second
                                                                                  // cruise
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to
                                                                                 // reach max vel
                // Take approximately 0.1 seconds to reach max accel
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
        
    }
    
    public void handleCurrentState() {
        switch (_currentState) {
            case IDLE:
            _pivotinator.setControl(_magicinator.withPosition(0));
            // _pivotinator.setPosition(Rotation.of(0));
                _suckinator.set(0);
                break;
            case PICKUP:
            _pivotinator.setControl(_magicinator.withPosition(0));
            // _pivotinator.setPosition(Rotation.of(0));
                if(_hasCoralinator()){ 
                    _suckinator.set(0.5);}
                    else{
                        _suckinator.set(0);
                    }
                break;

            case L1:
            _pivotinator.setControl(_magicinator.withPosition(0.75));
            // _pivotinator.setPosition(Rotation.of(0));
            if(_hasCoralinator()){ 
                _suckinator.set(0.5);}
                else{
                    _suckinator.set(0);
                }


            case L2:
            _pivotinator.setControl(_magicinator.withPosition(0.65));
            // _pivotinator.setPosition(Rotation.of(0));
            if(_hasCoralinator()){ 
                _suckinator.set(0.5);}
                else{
                    _suckinator.set(0);
                }


            case L4:
            _pivotinator.setControl(_magicinator.withPosition(0.90));
                // _pivotinator.setPosition(Rotation.of(0));
                if(_hasCoralinator()){ 
                    _suckinator.set(0.5);}
                    else{
                        _suckinator.set(0);
                    }

            default:
            _pivotinator.setControl(_magicinator.withPosition(0));
                // _pivotinator.setPosition(Rotation.of(0));
                _suckinator.set(0);
                break;
        }
    }
    @Override
    public void periodic() {
        handleCurrentState();
        _hasCoralinator = _calsificationDebouncinator.calculate(!_calsificationDetectinator.get());
    }

    public boolean _hasCoralinator() {
        return _hasCoralinator ;
    }
   

}
