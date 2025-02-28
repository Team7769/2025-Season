package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.enums.CalsificationinatorState;
import frc.robot.statemachine.StateBasedSubsystem;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class Calsificationinator extends SubsystemBase {
    private TalonFX _suckinator = new TalonFX(Constants.CalsificationinatorConstants.kSuckinatorScoreinatorID);

    private TalonFX _pivotinator = new TalonFX(Constants.CalsificationinatorConstants.kPivotinatorID);

    private boolean _hasCoralinator;

    private boolean _hasCoralinatorTwo;

    private TalonFXConfiguration _pivotConfig;
    private final MotionMagicVoltage _magicinator = new MotionMagicVoltage(0);

    private DigitalInput _calsificationDetectinator;

    private DigitalInput _calsificationDetectinatorTwo;

    private Debouncer _calsificationDebouncinator;

    private Debouncer _calsificationDebouncinatorTwo;

    private CalsificationinatorState _targetState;
    private CalsificationinatorState _currentState;
    private CalsificationinatorState _previousState;

    private VoltageOut voltageOut = new VoltageOut(0);
    // public SysIdRoutine pivotinatorRoutine = new SysIdRoutine(
    //         new SysIdRoutine.Config(Volts.of(.25).per(Second), Volts.of(2), null,
    //                 state -> SignalLogger.writeString("SysidPivotinator_State", state.toString())),
    //         new Mechanism(output -> _pivotinator.setControl(voltageOut.withOutput(output)), null, this));

    public Calsificationinator() {
        _targetState = CalsificationinatorState.IDLE;
        _currentState = CalsificationinatorState.IDLE;
        _previousState = CalsificationinatorState.IDLE;
        _calsificationDetectinator = new DigitalInput(
                Constants.CalsificationinatorConstants.kCalsificationDetectinatorChanel);
        _calsificationDetectinatorTwo = new DigitalInput(Constants.CalsificationinatorConstants.kCalsificationDetectinatorTwoChanel);
        _pivotConfig = new TalonFXConfiguration();
        _pivotConfig.Feedback.SensorToMechanismRatio = 12;
        _pivotConfig.Feedback.RotorToSensorRatio = 1;
        _pivotConfig.Feedback.FeedbackRemoteSensorID = 0;
        _pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        _calsificationDebouncinator = new Debouncer(.1);
        _calsificationDebouncinatorTwo = new Debouncer(.1);
        Slot0Configs slot0 = _pivotConfig.Slot0;
        slot0.kS = 0.24; // Add 0.25 V output to overcome static friction
        slot0.kV = 1.4; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0.05; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = 3; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output
        slot0.kG = 0;
        slot0.withGravityType(GravityTypeValue.Arm_Cosine);
        MotionMagicConfigs _pivotMotionMagic = _pivotConfig.MotionMagic;
        _pivotMotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(10)) // 5 (mechanism) rotations per second
                                                                                   // cruise
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to
                                                                                 // reach max vel
                // Take approximately 0.1 seconds to reach max accel
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(0));
        _pivotinator.getConfigurator().apply(_pivotConfig);
        _pivotinator.setNeutralMode(NeutralModeValue.Brake);
        _targetState = CalsificationinatorState.IDLE;
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
                if (_previousState == CalsificationinatorState.L1) {
                    _suckinator.set(0.05);
                } else {
                    _suckinator.set(0.3);
                }
                break;
            case NOTHING:
                break;
            case TARGET:
                _currentState = _targetState;
                break;
            case PREP_CLIMB:
            _pivotinator.setControl(_magicinator.withPosition(Constants.CalsificationinatorConstants.kPrepClimb));
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
        _hasCoralinatorTwo = _calsificationDebouncinatorTwo.calculate(!_calsificationDetectinatorTwo.get());

        SmartDashboard.putString("Calcificationator Target State", _targetState.name());
        SmartDashboard.putString("Calcificationator Current State", _currentState.name());
        SmartDashboard.putBoolean("Top Coral Detected", _hasCoralinator);
        SmartDashboard.putBoolean("Bottom Coral Detected", _hasCoralinatorTwo);
    }

    public boolean hasCoralinator() {
        return _hasCoralinator || _hasCoralinatorTwo;
    }

    public boolean doesNotHaveCoralinator() {
        return !_hasCoralinator && !_hasCoralinatorTwo;
    }

    private void handleCoral() {
        if (_hasCoralinatorTwo) {
            _suckinator.set(0);
        } else if (_hasCoralinator) {
            _suckinator.set(.05);
        } else {
            _suckinator.set(0.15);
        }
    }

    public InstantCommand zeroMotor() {
        return new InstantCommand(() -> _pivotinator.setControl(voltageOut.withOutput(0)), this);
    }

    public void setTargetState(CalsificationinatorState state)
    {
        _targetState = state;
    }

    public InstantCommand setWantedStateToTarget(){
        return this.setWantedState(_targetState);
    }

    public InstantCommand setWantedState(CalsificationinatorState state){
        return new InstantCommand(() -> {
            if(state == null)
            {
                return;
            }
            if (state != _currentState) {
                _previousState = _currentState;
                _currentState = state;
            }
        }, this);
    }
}
