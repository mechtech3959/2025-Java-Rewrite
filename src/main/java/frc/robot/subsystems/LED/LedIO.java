package frc.robot.subsystems.LED;

import org.w3c.dom.css.RGBColor;
import com.ctre.phoenix.led.Animation;



public interface LedIO {
    default void Start(){}
    default void Stop(){}
    default void changeAnimation(Animation type){}
    default void setOneColor(RGBColor color){}    
} 