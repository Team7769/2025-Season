package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.configs.CANdiConfiguration;

import frc.robot.Constants;

public class LEDinator {
    private CANdle _candle;
    private CANdiConfiguration config;

    private Calsificationinator _calsificationator;

    public LEDinator()
    {
        _candle = new CANdle(Constants.LEDinatorConstants.kLEDinatorID);
    }
    
}
