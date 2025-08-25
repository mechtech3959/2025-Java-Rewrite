package frc.robot.subsystems.LED;

import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.hardware.CANdle;

public class LedCANdleIO implements LedIO{
   CANdle leds = new CANdle(25, "CanBus");
   
   @Override 
   public void Start(){
    
   }
   @Override
   public void Stop(){
    leds.setControl(new EmptyAnimation(0));
   }
    
}
