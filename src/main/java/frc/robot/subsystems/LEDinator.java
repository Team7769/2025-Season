package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix6.configs.CANdiConfiguration;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.CageState;
import frc.robot.enums.ClawState;
import frc.robot.enums.LEDinatorState;
import frc.robot.enums.ReefLevel;
import frc.robot.statemachine.StateBasedSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDinator extends SubsystemBase {
    private CANdle _candle;
    private CANdle _partyLightinator;
    private CANdiConfiguration config;

    private Calsificationinator _calsificationator;

    private Animation WAITING_FOR_CORAL;
    private Animation WAITING_FOR_ALGAE;
    private Animation L1;
    private Animation L2;
    private Animation L3;
    private Animation L4;
    private Animation WAITING_FOR_CAGE;
    private Animation WOMPWOMP;
    private Animation IMPLOSION;
    private Animation DISCO_MODE;
    private Animation DISCO_MODE_PARTY;
    private Animation GREEN_DECORATION_LIGHTS;
    private Animation BLUE_DECORATION_LIGHTS;
    private int numLEDS = 60;
    private int numPartyLEDS = 60;

    private Calsificationinator _calsificationinator;
    private Claw _claw;
    private Elevatinator _elevatinator;
    private Ascendinator _ascendinator;
    private LEDinatorState _currentState;
    private LEDinatorState _previousState;

    public LEDinator(Calsificationinator calsificationinator, Claw claw, Elevatinator elevatinator, Ascendinator ascendinator)
    {
        _candle = new CANdle(Constants.LEDinatorConstants.kLEDinatorID);
        _partyLightinator = new CANdle(Constants.LEDinatorConstants.kLEDPartyLightinatorID);
        //white
        WAITING_FOR_CORAL = new StrobeAnimation(86, 1, 176, 0,.05, numLEDS);
        //teal
        WAITING_FOR_ALGAE = new StrobeAnimation(10, 255, 194, 0, .05, numLEDS);
        //purple for all levels
        L1 = new StrobeAnimation(86, 1, 176, 0, .05, numLEDS / 4);
        L2 = new StrobeAnimation(86, 1, 176, 0, .05, (numLEDS * 2) / 4);
        L3 = new StrobeAnimation(86, 1, 176, 0, .05, (numLEDS * 3) / 4);
        L4 = new StrobeAnimation(86, 1, 176, 0, .05, numLEDS);
        //orange
        //WAITING_FOR_CAGE = new StrobeAnimation(255,149,10,0,.15, numLEDS);
        WAITING_FOR_CAGE = new StrobeAnimation(120,0,0,0,.05, numLEDS);

        //fire
        WOMPWOMP = new FireAnimation(.5, .5, numLEDS, .25, .1);
        IMPLOSION = new StrobeAnimation(75, 0, 0, 0, 0.025, numPartyLEDS);
        //climb
        DISCO_MODE = new RainbowAnimation(.5, .5, numLEDS);
        DISCO_MODE_PARTY = new RainbowAnimation(.5, .5, numPartyLEDS);

        //green flow animation
        GREEN_DECORATION_LIGHTS = new ColorFlowAnimation(11, 252, 11, 255, .5, numPartyLEDS, Direction.Forward);
        BLUE_DECORATION_LIGHTS = new ColorFlowAnimation(0, 115, 255, 255,.5, numLEDS, Direction.Forward);

        _calsificationator = calsificationinator;
        _claw = claw;
        _elevatinator = elevatinator;
        _ascendinator = ascendinator;

        _currentState = LEDinatorState.CORAL;
        _previousState = LEDinatorState.ALGAE;

        setDecorationLights();
        
    }

    public void setWaitingForCoralAnimation()
    {
        //_candle.clearAnimation(0);
        _candle.animate(WAITING_FOR_CORAL);
    }

    public void setHasCoralAnimation()
    {
        //_candle.clearAnimation(0);
        _candle.setLEDs(86, 1, 176);
    }

    public void setWaitingForAlgaeAnimation()
    {
        //_candle.clearAnimation(0);
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
        //_candle.animate(L1);
        //_candle.setLEDs(150, 38, 255);
        _candle.setLEDs(86, 1, 176, 0, 0, (numLEDS / 4));
        _candle.setLEDs(0, 0, 0, 0, (numLEDS / 4) + 1, numLEDS);
    }

    public void setL2Animation()
    {
        _candle.clearAnimation(0);
        //_candle.animate(L2);
        _candle.setLEDs(86, 1, 176, 0, 0, (numLEDS * 2 / 4));
        _candle.setLEDs(0, 0, 0, 0, (numLEDS * 2 / 4) + 1, numLEDS);
    }

    public void setL3Animation()
    {
        _candle.clearAnimation(0);
        //_candle.animate(L3);
        _candle.setLEDs(86, 1, 176, 0, 0, (numLEDS * 3 / 4));
        _candle.setLEDs(0, 0, 0, 0, (numLEDS * 3/ 4) + 1, numLEDS);
    }

    public void setL4Animation()
    {
        _candle.clearAnimation(0);
        //_candle.animate(L4);
        //_candle.setLEDs(150, 38, 255);
        _candle.setLEDs(86, 1, 176, 0, 0, numLEDS);
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
        //_candle.clearAnimation(0);
        _candle.animate(WAITING_FOR_CAGE);
    }

    public void setHasCageAnimation()
    {
        _candle.clearAnimation(0);
        //_candle.setLEDs(255, 149, 10);
        _candle.setLEDs(10,255,0);
    }

    public void setEpicClimbAnimation()
    {
        //_candle.clearAnimation(0);
        _candle.animate(DISCO_MODE);
        //_partyLightinator.clearAnimation(0);
        _partyLightinator.animate(DISCO_MODE_PARTY);
    }

    public void setDecorationLights()
    {
        _partyLightinator.clearAnimation(0);
        _partyLightinator.animate(GREEN_DECORATION_LIGHTS);

        _candle.clearAnimation(0);
        _candle.animate(BLUE_DECORATION_LIGHTS);
    }

    public void setImplosionAnimation()
    {
        _candle.clearAnimation(0);
        _candle.animate(WOMPWOMP);

        _partyLightinator.clearAnimation(0);
        _partyLightinator.animate(IMPLOSION);

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
                if(_claw.getCurrentState() == ClawState.PREP_PROCESSOR)
                {
                    setProcessorAnimation();
                }
                else if(_claw.getCurrentState() == ClawState.PREP_NET)
                {
                    setNetAnimation();
                }
                else 
                {
                    setHasAlgaeAnimation();
                }
                
            }
                break;
            case CAGE:
            if(_ascendinator.getCurrentState() == CageState.ASCEND && _ascendinator.hasCage())
            {
                setEpicClimbAnimation();
            }
            else if(_ascendinator.getCurrentState() == CageState.ASCEND && !_ascendinator.hasCage())
            {
                setFallenAnimation();
            }
            else if(_ascendinator.getCurrentState() == CageState.DEPLOY && _ascendinator.hasCage())
            {
                setHasCageAnimation();
            }
            else if(_ascendinator.getCurrentState() == CageState.DEPLOY && !_ascendinator.hasCage())
            {
                setWaitingForCageAnimation();
            }
            else
            {
                setEpicClimbAnimation();
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
        SmartDashboard.putString("LEDinator State", _currentState.name());
    }

    @Override
    public void periodic() {
        handleCurrentState();
    }

        public InstantCommand setWantedState(LEDinatorState state){
        return new InstantCommand(() -> {
            if(state == null)
            {
                return;
            }
            if (state != _currentState) {
                _previousState = _currentState;
                _currentState = state;
                _candle.clearAnimation(0);
            }
        }, this);
    }




    
}
