package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Constants.CalsificationinatorConstants;
import frc.robot.enums.CageState;
import frc.robot.enums.CalsificationinatorState;
import frc.robot.statemachine.StateBasedSubsystem;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;

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

    private VoltageOut voltageOut = new VoltageOut(0);
    public SysIdRoutine pivotinatorRoutine = new SysIdRoutine(new SysIdRoutine.Config(Volts.of(.25).per(Second), Volts.of(2), null,
        state -> SignalLogger.writeString("SysidPivotinator_State", state.toString())), 
        new Mechanism(output -> _pivotinator.setControl(voltageOut.withOutput(output)), null, this));
   
    public Calsificationinator(){
        _currentState = CalsificationinatorState.NOTHING;
        _previousState = CalsificationinatorState.NOTHING;
        _calsificationDetectinator = new DigitalInput(Constants.CalsificationinatorConstants.kCalsificationDetectinatorChanel);
        _pivotConfig = new TalonFXConfiguration();
        _pivotConfig.Feedback.FeedbackRemoteSensorID = CalsificationinatorConstants.kPivotinatorCancoderID;
        _pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        _pivotConfig.Feedback.SensorToMechanismRatio = 1;
        _pivotConfig.Feedback.RotorToSensorRatio = 12;

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
        _pivotinator.getConfigurator().apply(_pivotConfig);
    }
    
    public void handleCurrentState() {
        switch (_currentState) {
            case IDLE:
            _pivotinator.setControl(_magicinator.withPosition(Constants.CalsificationinatorConstants.kIdlePosition));
           
                handleCoral();
                break;
            case PICKUP:
            _pivotinator.setControl(_magicinator.withPosition(Constants.CalsificationinatorConstants.kPickUpPosition));
           handleCoral();
                break;

            case L1:
            _pivotinator.setControl(_magicinator.withPosition(Constants.CalsificationinatorConstants.kL1Position));
         
            handleCoral();

            break;
            case L2:
            _pivotinator.setControl(_magicinator.withPosition(Constants.CalsificationinatorConstants.kL2Position));
            
           handleCoral();
           break;
           case L3:
           _pivotinator.setControl(_magicinator.withPosition(Constants.CalsificationinatorConstants.kL3Position));
           
          handleCoral();
          break;

            case L4:
            _pivotinator.setControl(_magicinator.withPosition(Constants.CalsificationinatorConstants.kL4Position));
                
               handleCoral();
                break;
            case SCORE:
            _suckinator.set(0.3);
            break;
            case NOTHING:
            break;

            default:
            _pivotinator.setControl(_magicinator.withPosition(Constants.CalsificationinatorConstants.kIdlePosition));
            
                handleCoral();
                break;
        }
    }
    @Override
    public void periodic() {
        handleCurrentState();
        _hasCoralinator = _calsificationDebouncinator.calculate(!_calsificationDetectinator.get());
    }

    public boolean hasCoralinator() {
        return _hasCoralinator ;
    }
   private void handleCoral(){
    if(hasCoralinator()){ 
    _suckinator.set(0);}
    else{
        _suckinator.set(-0.25);
    }
}

public InstantCommand zeroMotor() {
    return new InstantCommand(() -> _pivotinator.setControl(voltageOut.withOutput(0)), this);
}

}
