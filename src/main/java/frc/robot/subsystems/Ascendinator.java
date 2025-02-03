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
    private TalonFX _primaryinatorAscendinator = new TalonFX(Constants.SubsystemConstants.kPrimaryinatorAscendinatorID);
    private TalonFX _secondaryinatorAscendinator = new TalonFX(
            Constants.SubsystemConstants.kSecondaryinatorAscendinatorID);
    private TalonFXConfiguration _configinator = new TalonFXConfiguration();

    private MotionMagicVoltage _motionMagic = new MotionMagicVoltage(0);

    private boolean _hasCage;

    private Debouncer _debounceinator;

    private DigitalInput _detectinator;

    private FeedbackConfigs FeedBackConfig = _configinator.Feedback;

    private MotionMagicConfigs _motionMagicinator = _configinator.MotionMagic;

    private Follower _follower;

    public Ascendinator() {
        Slot0Configs slot0 = _configinator.Slot0;
        slot0.kS = 0.25;
        slot0.kV = 0.12;
        slot0.kA = 0.01;
        slot0.kP = 60;
        slot0.kI = 0;
        slot0.kD = 0.5;

        FeedBackConfig.SensorToMechanismRatio = 3.5;

        MotionMagicConfigs MotionMagicinator = _configinator.MotionMagic;
        MotionMagicinator.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

        _follower = new Follower(16, false);
        _secondaryinatorAscendinator.setControl(_follower);

        _detectinator = new DigitalInput(Constants.SubsystemConstants.kAscendinatorDetectinatorChannel);

        _debounceinator = new Debouncer(Constants.SubsystemConstants.kDebouncinatorTime, DebounceType.kBoth);
    }

    public void handleCurrentState() {
        switch (_currentState) {
            case IDLE:
                _primaryinatorAscendinator.set(0);
                break;

            case DEPLOY:
                _primaryinatorAscendinator.setControl(_motionMagic.withPosition(0.25));
                break;

            case ASCEND:
                if (_primaryinatorAscendinator.getPosition().getValueAsDouble() > 0.5) {
                    setWantedState(CageState.IDLE);
                } else {
                    _primaryinatorAscendinator.set(1);
                }
                break;

            default:
                _primaryinatorAscendinator.set(0);
                break;
        }
    }

    @Override
    public void periodic() {
        _hasCage = _debounceinator.calculate(!_detectinator.get());
        handleCurrentState();
    }

    public boolean hasCage() {
        return _hasCage;
    }
}
