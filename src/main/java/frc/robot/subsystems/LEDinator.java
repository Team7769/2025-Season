package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix6.configs.CANdiConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.CageState;
import frc.robot.enums.LEDinatorState;
import frc.robot.enums.ReefLevel;
import frc.robot.statemachine.StateBasedSubsystem;

public class LEDinator extends SubsystemBase {
    private CANdle _candle;
    private CANdiConfiguration config;

    private Calsificationinator _calsificationator;

    private Animation WAITING_FOR_CORAL;
    private Animation WAITING_FOR_ALGAE;
    private Animation L1;
    private Animation L2;
    private Animation L3;
    private Animation L4;
    private Animation WAITING_FOR_CAGE;
    private Animation EPIC_CLIMB;
    private int numLEDS = 20;

    private Calsificationinator _calsificationinator;
    private Claw _claw;
    private Elevatinator _elevatinator;
    private Ascendinator _ascendinator;
    private LEDinatorState _currentState;
    private LEDinatorState _previousState;

    public LEDinator(Calsificationinator calsificationinator, Claw claw, Elevatinator elevatinator, Ascendinator ascendinator)
    {
        _candle = new CANdle(Constants.LEDinatorConstants.kLEDinatorID);
        //white
        WAITING_FOR_CORAL = new StrobeAnimation(255,255,255, 0,.15, numLEDS);
        //teal
        WAITING_FOR_ALGAE = new StrobeAnimation(10, 255, 194, 0, .15, numLEDS);
        //purple for all levels
        L1 = new StrobeAnimation(150, 38, 255, 0, .05, numLEDS / 4);
        L2 = new StrobeAnimation(150, 38, 255, 0, .05, (numLEDS * 2) / 4);
        L3 = new StrobeAnimation(150, 38, 255, 0, .05, (numLEDS * 3) / 4);
        L4 = new StrobeAnimation(150, 38, 255, 0, .05, numLEDS);
        //orange
        WAITING_FOR_CAGE = new StrobeAnimation(255,149,10,0,.15, numLEDS);

        //fire
        EPIC_CLIMB = new FireAnimation(.5, .5, numLEDS, .25, .1);

        _calsificationator = calsificationinator;
        _claw = claw;
        _elevatinator = elevatinator;
        _ascendinator = ascendinator;

        _currentState = LEDinatorState.CORAL;
        _previousState = LEDinatorState.ALGAE;
        
    }

    public void setWaitingForCoralAnimation()
    {
        _candle.clearAnimation(0);
        _candle.animate(WAITING_FOR_CORAL);
    }

    public void setHasCoralAnimation()
    {
        _candle.clearAnimation(0);
        _candle.setLEDs(255, 255, 255);
    }

    public void setWaitingForAlgaeAnimation()
    {
        _candle.clearAnimation(0);
        _candle.animate(WAITING_FOR_ALGAE);
    }

    public void setHasAlgaeAnimation()
    {
        _candle.clearAnimation(0);
        _candle.setLEDs(10, 255, 194);
    }

    public void setL1Animation()
    {
        _candle.clearAnimation(0);
        _candle.animate(L1);
    }

    public void setL2Animation()
    {
        _candle.clearAnimation(0);
        _candle.animate(L2);
    }

    public void setL3Animation()
    {
        _candle.clearAnimation(0);
        _candle.animate(L3);
    }

    public void setL4Animation()
    {
        _candle.clearAnimation(0);
        _candle.animate(L4);
    }

    public void setLevelAnimation()
    {
        var elevatinatorTarget = _elevatinator.getPositioninator();
        if(elevatinatorTarget == Constants.ElevatinatorConstants.kL4Coral)
        {
            setL4Animation();
        }
        else if(elevatinatorTarget == Constants.ElevatinatorConstants.kL3Coral)
        {
            setL3Animation();
        }
        else if(elevatinatorTarget == Constants.ElevatinatorConstants.kL2Coral)
        {
            setL2Animation();
        }
        else
        {
            setL1Animation();
        }
    }

    public void setReadyToScoreAnimation()
    {
        _candle.clearAnimation(0);
        _candle.setLEDs(0, 255, 0);
    }

    public void setProcessorAnimation()
    {
        _candle.clearAnimation(0);
        _candle.setLEDs(255, 64, 169);
    }

    public void setNetAnimation()
    {
        _candle.clearAnimation(0);
        _candle.setLEDs(10, 96, 255);
    }

    public void setWaitingForCageAnimation()
    {
        _candle.clearAnimation(0);
        _candle.animate(WAITING_FOR_CAGE);
    }

    public void setHasCageAnimation()
    {
        _candle.clearAnimation(0);
        _candle.setLEDs(255, 149, 10);
    }

    public void setEpicClimbAnimation()
    {
        _candle.clearAnimation(0);
        _candle.animate(EPIC_CLIMB);
    }

    public void handleCurrentState()
    {
        switch(_currentState)
        {
            case ALGAE:
            if(!_claw.hasAlgae())
            {
                setWaitingForAlgaeAnimation();
            }
            else
            {
                setHasAlgaeAnimation();
            }
                break;
            case CAGE:
            if(_ascendinator.getCurrentState() == CageState.ASCEND)
            {
                setEpicClimbAnimation();
            }
            else if(!_ascendinator.hasCage())
            {
                setWaitingForCageAnimation();
            }
            else
            {
                setHasCageAnimation();
            }
                break;
            default:
            //coral
            if(!_calsificationator.hasCoralinator())
            {
                setWaitingForCoralAnimation();
            }
            else
            {
                setLevelAnimation();
            }
                break;
        }
    }

    @Override
    public void periodic() {
        handleCurrentState();
    }




    
}
