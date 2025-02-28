package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import frc.robot.enums.CalsificationinatorState;
import frc.robot.enums.ClawState;
import frc.robot.statemachine.StateBasedSubsystem;

public class Claw extends SubsystemBase {

    private TalonFX _pivotinator;
    private TalonFXConfiguration _pivotConfig;
    private FeedbackConfigs _pivotFeedbackConfigs;
    private final MotionMagicVoltage _request = new MotionMagicVoltage(0);

    private TalonFX _topRollinator;
    private TalonFX _bottomRollinator;

    private DigitalInput _algaeDetectinator;
    private Debouncer _algaeDebouncinator;
    private boolean _hasAlgae;

    private ClawState _targetClawState;
    private ClawState _currentState;
    private ClawState _previousState;

    private final VoltageOut voltage = new VoltageOut(0);
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysId_Claw", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> _pivotinator.setControl(voltage.withOutput(output)),
            null,
            this
        )
    );

    public Claw() {

        // TODO: Confirm/change all motor config values

        //// #region Pivot motor configs
        _pivotinator = new TalonFX(Constants.ClawConstants.kClawPivotinatorID);
        _pivotConfig = new TalonFXConfiguration();
        _pivotFeedbackConfigs = _pivotConfig.Feedback;
        _pivotFeedbackConfigs.RotorToSensorRatio = 1;
        _pivotFeedbackConfigs.SensorToMechanismRatio = 45;
        // 45 to 1 gear ratio

        MotionMagicConfigs _pivotMotionMagic = _pivotConfig.MotionMagic;
        _pivotMotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(2)) // 5 (mechanism) rotations per second
                                                                                  // cruise
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(2)) // Take approximately 0.5 seconds to
                                                                                 // reach max vel
                // Take approximately 0.1 seconds to reach max accel
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(0));

        Slot0Configs slot0 = _pivotConfig.Slot0;
        slot0.kS = 0.12; // Add 0.25 V output to overcome static friction
        slot0.kV = 2.5; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = 30; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output
        slot0.kG = .1;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;

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
        _pivotinator.setNeutralMode(NeutralModeValue.Brake);
        //// #endregion

        ///// #region Rollinator motor configs
        _topRollinator = new TalonFX(Constants.ClawConstants.kClawTopRollinatorID);
        _topRollinator.setNeutralMode(NeutralModeValue.Brake);
        var topConfig = new TalonFXConfiguration();
        topConfig.CurrentLimits.SupplyCurrentLimit = 70;
        topConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        topConfig.CurrentLimits.StatorCurrentLimit = 40;
        topConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        _topRollinator.getConfigurator().apply(topConfig);


        _bottomRollinator = new TalonFX(Constants.ClawConstants.kClawBottomRollinatorID);
        _bottomRollinator.setNeutralMode(NeutralModeValue.Brake);
        var bottomConfig = new TalonFXConfiguration();
        bottomConfig.CurrentLimits.SupplyCurrentLimit = 70;
        bottomConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        bottomConfig.CurrentLimits.StatorCurrentLimit = 40;
        bottomConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        _bottomRollinator.getConfigurator().apply(bottomConfig);

        // bottom Rollinator will follow the top Rollinator in the opposite direction
        _bottomRollinator.setControl(new Follower(Constants.ClawConstants.kClawTopRollinatorID, true));
        /// //#endregion
        /// //#region Detectinator config

        _algaeDetectinator = new DigitalInput(Constants.ClawConstants.kClawAlgaeDetectinatorChannel);
        _algaeDebouncinator = new Debouncer(Constants.ClawConstants.kClawAlgaeDebounceTime, DebounceType.kRising);
        /// //#endregion

        _currentState = ClawState.IDLE;
        _previousState = ClawState.FLOOR_INTAKE;
        _targetClawState = ClawState.IDLE;

    }

    public void handleCurrentState() {
        switch (_currentState) {
            case IDLE:
                _pivotinator.setControl(_request.withPosition(0));
                // _pivotinator.setPosition(Rotation.of(0));
                // if (_hasAlgae) {
                //     _topRollinator.set(-.05);
                // } else {                    
                //     _topRollinator.set(0);
                // }
                _topRollinator.set(-.2);
                break;
            case FLOOR_INTAKE:
                _pivotinator.setControl(_request.withPosition(0.375));
                _topRollinator.set(-.5);
                break;
            case DEALGIFY:
                _pivotinator.setControl(_request.withPosition(0.25));
                _topRollinator.set(-.5);
                break;
            case PREP_NET:
                _pivotinator.setControl(_request.withPosition(0));
                _topRollinator.set(-.2);
                break;
            case PREP_PROCESSOR:
                _pivotinator.setControl(_request.withPosition(0.18));
                _topRollinator.set(-.2);
                break;
            case SCORE:
                if (_previousState == ClawState.PREP_NET) {
                    _topRollinator.set(.7);
                }
                if (_previousState == ClawState.PREP_PROCESSOR) {
                    _topRollinator.set(.5);
                }
                break;
            case TARGET:
                _currentState = _targetClawState;
                break;
            default:
                _pivotinator.setControl(_request.withPosition(0));
                _topRollinator.set(0);
                break;
        }
    }

    public InstantCommand setWantedTargetState(){        
        return setWantedState(_targetClawState);
    }

    public InstantCommand setWantedState(ClawState state){
        return new InstantCommand(() -> {
            if(state == null)
            {
                return;
            }
            if (state != _currentState) {
                _previousState = _currentState;
                _currentState = state;

                switch (state){
                    case SCORE:
                        setHighCurrentLimit();
                        break;
                    default:
                        setLowCurrentLimit();
                        break;
                }
            }
        }, this);
    }

    private void setHighCurrentLimit()
    {
        var currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = 120;
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = 70;
        currentLimits.SupplyCurrentLimitEnable = true;

        _topRollinator.getConfigurator().apply(currentLimits);        
        _bottomRollinator.getConfigurator().apply(currentLimits);
    }

    private void setLowCurrentLimit()
    {
        var currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = 40;
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = 70;
        currentLimits.SupplyCurrentLimitEnable = true;

        _topRollinator.getConfigurator().apply(currentLimits);        
        _bottomRollinator.getConfigurator().apply(currentLimits);
    }

    @Override
    public void periodic() {
        handleCurrentState();
        _hasAlgae = _algaeDebouncinator.calculate(!_algaeDetectinator.get());
        SmartDashboard.putString("Claw Current State", _currentState.name());
        SmartDashboard.putBoolean("Claw Has Algae", _hasAlgae);
        SmartDashboard.putString("Claw Target", _targetClawState.name());
    }

    public boolean hasAlgae() {
        return _hasAlgae;
    }
    public boolean doesNotHaveAlgae() {
        return !_hasAlgae;
    }

    public void setTargetState(ClawState clawState)
    {
        _targetClawState = clawState;
    }

}
