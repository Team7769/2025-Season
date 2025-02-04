package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.Constants;
import frc.robot.enums.ClawState;
import frc.robot.statemachine.StateBasedSubsystem;

public class Claw extends StateBasedSubsystem<ClawState> {

    private TalonFX _pivotinator;
    private TalonFXConfiguration _pivotConfig;
    private FeedbackConfigs _pivotFeedbackConfigs;
    private final MotionMagicVoltage _request = new MotionMagicVoltage(0);

    private TalonFX _topRollinator;
    private TalonFX _bottomRollinator;

    private DigitalInput _algaeDetectinator;
    private Debouncer _algaeDebouncinator;
    private boolean _hasAlgae;

    public Claw() {

        // TODO: Confirm/change all motor config values

        //// #region Pivot motor configs
        _pivotinator = new TalonFX(Constants.ClawConstants.kClawPivotinatorID);
        _pivotConfig = new TalonFXConfiguration();
        _pivotFeedbackConfigs = _pivotConfig.Feedback;
        _pivotFeedbackConfigs.SensorToMechanismRatio = 1;
        // 45 to 1 gear ratio

        MotionMagicConfigs _pivotMotionMagic = _pivotConfig.MotionMagic;
        _pivotMotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5)) // 5 (mechanism) rotations per second
                                                                                  // cruise
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to
                                                                                 // reach max vel
                // Take approximately 0.1 seconds to reach max accel
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

        Slot0Configs slot0 = _pivotConfig.Slot0;
        slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = 0.5; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output
        slot0.kG = 0;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = _pivotinator.getConfigurator().apply(_pivotConfig);
            if (status.isOK()) {
                break;
            }

            if (!status.isOK()) {
                System.out.println("Could not configure device. Error: " + status.toString());
            }
        }
        //// #endregion

        ///// #region Rollinator motor configs
        _topRollinator = new TalonFX(Constants.ClawConstants.kClawTopRollinatorID);

        _bottomRollinator = new TalonFX(Constants.ClawConstants.kClawBottomRollinatorID);

        // bottom Rollinator will follow the top Rollinator in the opposite direction
        _bottomRollinator.setControl(new Follower(Constants.ClawConstants.kClawTopRollinatorID, true));
        /// //#endregion
        /// //#region Detectinator config

        _algaeDetectinator = new DigitalInput(Constants.ClawConstants.kClawAlgaeDetectinatorChannel);
        _algaeDebouncinator = new Debouncer(Constants.ClawConstants.kClawAlgaeDebounceTime, DebounceType.kBoth);
        /// //#endregion

        _currentState = ClawState.IDLE;
        _previousState = ClawState.FLOOR_INTAKE;

    }

    public void handleCurrentState() {
        switch (_currentState) {
            case IDLE:

                _pivotinator.setControl(_request.withPosition(0));
                // _pivotinator.setPosition(Rotation.of(0));
                _topRollinator.set(0);
                break;
            case FLOOR_INTAKE:
                _pivotinator.setControl(_request.withPosition(0.375));
                _topRollinator.set(.5);
                break;
            case DEALGIFY:
                _pivotinator.setControl(_request.withPosition(0.25));
                _topRollinator.set(.5);
                break;
            case PREP_NET:
                _pivotinator.setControl(_request.withPosition(0.125));
                break;
            case PREP_PROCESSOR:
                _pivotinator.setControl(_request.withPosition(0.25));
                break;
            case SCORE:
                if (_previousState == ClawState.PREP_NET) {
                    _topRollinator.set(-.8);
                }
                if (_previousState == ClawState.PREP_PROCESSOR) {
                    _topRollinator.set(-.5);
                }
                break;
            default:
                _pivotinator.setControl(_request.withPosition(0));
                _topRollinator.set(0);
                break;
        }
    }

    @Override
    public void periodic() {
        handleCurrentState();
        _hasAlgae = _algaeDebouncinator.calculate(!_algaeDetectinator.get());
    }

    public boolean hasAlgae() {
        return _hasAlgae;
    }

}
