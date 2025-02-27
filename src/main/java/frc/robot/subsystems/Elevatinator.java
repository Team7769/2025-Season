package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.ElevatinatorConstants;
import frc.robot.enums.CalsificationinatorState;
import frc.robot.enums.ElavatinatorState;
import frc.robot.statemachine.StateBasedSubsystem;

public class Elevatinator extends StateBasedSubsystem<ElavatinatorState> {
    private TalonFX _liftMotorinator;
    private VoltageOut voltageOut = new VoltageOut(0);
    private Slot0Configs _PIDConfiginator;
    private MotionMagicVoltage _requestinator;
    private double _manualPositioninator;
    private boolean _holdAlgaePosition = false;
    private MotionMagicConfigs _motionMagicConfiginator;
    private TalonFXConfiguration _talonFXConfiginator;
    public SysIdRoutine elevatorRoutine = new SysIdRoutine(new SysIdRoutine.Config(null, Volts.of(4), null,
        state -> SignalLogger.writeString("SysidElevatinator_State", state.toString())), 
        new Mechanism(output -> _liftMotorinator.setControl(voltageOut.withOutput(output)), null, this));

        private double _algaePosition = 0;
    //private Claw _claw;
    public Elevatinator() {
        //_claw = claw;
        _manualPositioninator = 0;
        _talonFXConfiginator = new TalonFXConfiguration();
        _talonFXConfiginator.Feedback.SensorToMechanismRatio = 1;
        _PIDConfiginator = _talonFXConfiginator.Slot0;
        _PIDConfiginator.withGravityType(GravityTypeValue.Elevator_Static);
        _PIDConfiginator.kG = 0.37009;
        _PIDConfiginator.kS = 0.060989;
        _PIDConfiginator.kV = 0.1204;
        _PIDConfiginator.kP = 4; 
        _PIDConfiginator.kI = 0; 
        _PIDConfiginator.kD = 0.05;
        _PIDConfiginator.kA = 0.0022182;
        _motionMagicConfiginator = _talonFXConfiginator.MotionMagic;
        _motionMagicConfiginator.MotionMagicCruiseVelocity = 400; // Target cruise velocity in rps
        _motionMagicConfiginator.MotionMagicAcceleration = 300; // Target acceleration in rps/s 
        _motionMagicConfiginator.MotionMagicJerk = 0; // Target jerk in rps/s/s
        _requestinator = new MotionMagicVoltage(0);
        _liftMotorinator = new TalonFX(ElevatinatorConstants.kLifinatorMotor);
        _currentState = ElavatinatorState.HOME;
        _previousState = ElavatinatorState.HOLD;
        _liftMotorinator.getConfigurator().apply(_talonFXConfiginator);
        _liftMotorinator.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setPositioninator(double positioninator) {
        _manualPositioninator = positioninator;
    }

    public void setAlgaePosition(int reefFace) {
        _algaePosition = reefFace % 2 == 0 ? ElevatinatorConstants.kL2Algae : ElevatinatorConstants.kL3Algae;
    }

    public void setHoldAlgaePosition(boolean value) {
        _holdAlgaePosition = value;
    }

    private void holdPositioninator() {
        if (_holdAlgaePosition) {
            _liftMotorinator.setControl(_requestinator.withPosition(_algaePosition));
        } else {
            _liftMotorinator.setControl(_requestinator.withPosition(_manualPositioninator));
        }
    }

    public double getPositioninator()
    {
        return _manualPositioninator;
    }

    public boolean isReady() {
        return (Math.abs(_manualPositioninator - _liftMotorinator.getPosition().getValueAsDouble()) < .5) && (_currentState != ElavatinatorState.HOME);
    }

    public InstantCommand zeroMotor() {
        return new InstantCommand(() -> _liftMotorinator.setControl(voltageOut.withOutput(0)), this);
    }

    private void handleCurrentStateinator(){
        switch (_currentState){
            case HOLD:
                holdPositioninator();
            break;
            case MANUAL:
                //setPositioninator(SmartDashboard.getNumber("Manual Position", 20));
                setPositioninator(30);

                holdPositioninator();
            break;
            case HOME:
                _liftMotorinator.setControl(_requestinator.withPosition(ElevatinatorConstants.kHumanPlayer));
                break;
            default:
            break;
        }
    }
    
    @Override
    public void periodic(){
        SmartDashboard.putString("Elevator Current state", _currentState.name());
        SmartDashboard.putNumber("Manual Position", _manualPositioninator);
        SmartDashboard.putNumber("Algae Position", _algaePosition);
        SmartDashboard.putBoolean("Hold Algae Position", _holdAlgaePosition);
        handleCurrentStateinator();
    }

    // public InstantCommand setWantedState(ElavatinatorState state){
    //     return new InstantCommand(() -> {
    //         if(state == null)
    //         {
    //             return;
    //         }
    //         if (state != _currentState) {
    //             _previousState = _currentState;
    //             _currentState = state;
    //         }
    //     }, this);
    // }
}
