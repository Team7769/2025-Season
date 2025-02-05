package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.AscendinatorConstants;
import frc.robot.enums.ElavatinatorState;
import frc.robot.statemachine.StateBasedSubsystem;

public class Elevatinator extends StateBasedSubsystem<ElavatinatorState>{
    private TalonFX _liftMotorinator;
    private Slot0Configs PIDConfiginator;
    private MotionMagicVoltage requestinator;
    private double manualPositioninator;
    private MotionMagicConfigs motionMagicConfiginator;
    private TalonFXConfiguration talonFXConfiginator;
    private SysIdRoutine elevatorRoutine;

    public Elevatinator() {
        sysidSetup();
        manualPositioninator = 0;
        talonFXConfiginator = new TalonFXConfiguration();
        PIDConfiginator = talonFXConfiginator.Slot0;
        PIDConfiginator.withGravityType(GravityTypeValue.Elevator_Static);
        PIDConfiginator.kG = 0;
        PIDConfiginator.kS = 0;
        PIDConfiginator.kV = 0;
        PIDConfiginator.kP = 0; 
        PIDConfiginator.kI = 0; 
        PIDConfiginator.kD = 0;
        motionMagicConfiginator = talonFXConfiginator.MotionMagic;
        motionMagicConfiginator.MotionMagicCruiseVelocity = 0; // Target cruise velocity in rps
        motionMagicConfiginator.MotionMagicAcceleration = 0; // Target acceleration in rps/s 
        motionMagicConfiginator.MotionMagicJerk = 0; // Target jerk in rps/s/s
        requestinator = new MotionMagicVoltage(0);
        _liftMotorinator = new TalonFX(AscendinatorConstants.kLifinatorMotor);
        _currentState = ElavatinatorState.HOLD;
        _previousState = ElavatinatorState.IDLE;
        _liftMotorinator.getConfigurator().apply(PIDConfiginator);
    }

    public void setPositioninator(double positioninator) {
        manualPositioninator = positioninator;
    }

    private void holdPositioninator() {
        _liftMotorinator.setControl(requestinator.withPosition(manualPositioninator));
    }

    private void handleCurrentStateinator(){
        switch (_currentState){
            case HOLD:
                holdPositioninator();
            break;
            case MANUAL:
                setPositioninator(SmartDashboard.getNumber("Manual Position", 0));
                holdPositioninator();
            break;
            default:
            break;
        }
    }
    
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Manual Position", manualPositioninator);
        handleCurrentStateinator();
    }

    private void sysidSetup() {
        elevatorRoutine = new SysIdRoutine(new SysIdRoutine.Config(null, Volts.of(4), null,
            state -> SignalLogger.writeString("SysidElevatinator_State", state.toString())), 
        new Mechanism(output -> _liftMotorinator.setVoltage(output.magnitude()), null, this));
    }
}
