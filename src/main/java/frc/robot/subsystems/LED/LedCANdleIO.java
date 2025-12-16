package frc.robot.subsystems.LED;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

public class LedCANdleIO implements LedIO {
   CANdle leds = new CANdle(25, "CanBus");

   @Override
   public void Start() {

   }

   @Override
   public void Stop() {

   }

   @Override
   // ONLY PUT IN CTRE ANIMATION TYPES
   public void changeAnimation(Animation type) {
      leds.animate(type);
   }

}
