package frc.robot.subsystems.LED;

import org.w3c.dom.css.RGBColor;

import com.ctre.phoenix6.signals.Animation0TypeValue;

public interface LedIO {
    default void Start(){}
    default void Stop(){}
    default void changeAnimation(Animation0TypeValue type){}
    default void setOneColor(RGBColor color){}    
} 