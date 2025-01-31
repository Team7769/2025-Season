package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.enums.ElavatinatorState;
import frc.robot.statemachine.StateBasedSubsystem;

public class Elevatinator extends StateBasedSubsystem<ElavatinatorState>{
    private TalonFX _liftMotorinator;
    private Slot0Configs PIDConfiginator;
    private TrapezoidProfile profileinator;
    private TrapezoidProfile.State wantedPointinator;
    private TrapezoidProfile.State setPointinator;
    private PositionVoltage requestinator;

    public Elevatinator() {
        wantedPointinator = new TrapezoidProfile.State();
        setPointinator = new TrapezoidProfile.State();
        profileinator = new TrapezoidProfile(new TrapezoidProfile.Constraints(0, 0));
        PIDConfiginator = new Slot0Configs().withGravityType(GravityTypeValue.Elevator_Static);
        PIDConfiginator.kG = 0;
        PIDConfiginator.kS = 0;
        PIDConfiginator.kV = 0;
        PIDConfiginator.kP = 0; 
        PIDConfiginator.kI = 0; 
        PIDConfiginator.kD = 0;
        requestinator = new PositionVoltage(0).withSlot(0);

        _liftMotorinator = new TalonFX(DrivetrainConstants.kLifinatorMotor);
        _currentState = ElavatinatorState.HOLD;
        _previousState = ElavatinatorState.IDLE;
        _liftMotorinator.getConfigurator().apply(PIDConfiginator);
    }

    public void setPositioninator(double positioninator) {
        wantedPointinator.position = positioninator;
        setPointinator = profileinator.calculate(.02, setPointinator, wantedPointinator);
    }

    private void holdPositioninator() {
        requestinator.Position = setPointinator.position;
        requestinator.Velocity = setPointinator.velocity;
        _liftMotorinator.setControl(requestinator);
    }

    private void handleCurrentStateinator(){
        switch (_currentState){
            case IDLE:
            break;
            default:
                holdPositioninator();
            break;
        }
    }
    
    @Override
    public void periodic(){
        handleCurrentStateinator();
    }
}
