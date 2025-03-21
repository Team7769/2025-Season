package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Constants.AscendinatorConstants;
import frc.robot.enums.CageState;
import frc.robot.enums.CalsificationinatorState;
import frc.robot.statemachine.StateBasedSubsystem;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.filter.Debouncer.DebounceType;

public class Ascendinator extends SubsystemBase {
    private TalonFX _ascendinator = new TalonFX(Constants.AscendinatorConstants.kPrimaryinatorAscendinatorID);
    private TalonFXConfiguration _configinator = new TalonFXConfiguration();

    private boolean _hasCage;
    private boolean _readyToClimb = false;
    private Debouncer _debounceinator;

    private DigitalInput _detectinator;

    private FeedbackConfigs FeedBackConfig = _configinator.Feedback;
    private CageState _currentState = CageState.IDLE;
    private CageState _previousState = CageState.IDLE;

    private VoltageOut voltageOut = new VoltageOut(0);
    public SysIdRoutine pivotinatorRoutine = new SysIdRoutine(new SysIdRoutine.Config(null, Volts.of(4), null,
            state -> SignalLogger.writeString("SysidAscendinator_State", state.toString())),
            new Mechanism(output -> _ascendinator.setControl(voltageOut.withOutput(output)), null, this));

    public Ascendinator() {
        Slot0Configs slot0 = _configinator.Slot0;
        slot0.kS = 0.25;
        slot0.kV = 0.12;
        slot0.kA = 0.01;
        slot0.kP = 60;
        slot0.kI = 0;
        slot0.kD = 0.5;

        FeedBackConfig.SensorToMechanismRatio = 350;

        MotionMagicConfigs MotionMagicinator = _configinator.MotionMagic;
        MotionMagicinator.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

        _detectinator = new DigitalInput(Constants.AscendinatorConstants.kAscendinatorDetectinatorChannel);

        _debounceinator = new Debouncer(Constants.AscendinatorConstants.kDebouncinatorTime, DebounceType.kBoth);
    }

    public void handleCurrentState() {
        switch (_currentState) {
            case IDLE:
                _ascendinator.set(0);
                break;

            case DEPLOY:
                if (_ascendinator.getPosition().getValueAsDouble() >= AscendinatorConstants.kPrepClimb) {
                    _ascendinator.set(0);
                    _readyToClimb = true;
                } else {
                    _ascendinator.set(0.8);
                }
                break;

            case ASCEND:
                if (_ascendinator.getPosition().getValueAsDouble() >= AscendinatorConstants.kEndClimb) {
                    _ascendinator.set(0);
                    _currentState = CageState.IDLE;
                } else {
                    _ascendinator.set(0.5);
                }
                break;
            default:
                _ascendinator.set(0);
                break;
        }
    }

    public InstantCommand setWantedState(CageState state){
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

    @Override
    public void periodic() {
        _hasCage = _debounceinator.calculate(!_detectinator.get());
        handleCurrentState();
    }

    public boolean hasCage() {
        return _hasCage;
    }
    
    public boolean isReady() {
        return _readyToClimb;
    }

    public CageState getCurrentState() {
        return _currentState;
    }
}
