package frc.robot.commands;

import frc.robot.subsystems.LightSubsystem; 
import edu.wpi.first.wpilibj2.command.Command; 
import edu.wpi.first.units.measure.Distance; 
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Meters;

public class RainbowLightCmd extends Command { 
private final LightSubsystem light;
private final LEDPattern pattern;

public RainbowLightCmd(LightSubsystem light) {
    this.light = light;

    // declaring the subsystem dependency
    addRequirements(light);
    //equal to the space between LEDs
    Distance spacing = Meters.of(1.0 / 60);
    /*creates a rainbow pattern with full brightness and half saturation
    scrolls at a contant speed (1 m/s)
    NOTE, if you want to make a simple LED Pattern, modify this line only!!! (line 28)
    you can look at the LEDPattern interface: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/LEDPattern.html
    see the presentation for some examples and ideas*/
    pattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kBlueViolet, Color.kMagenta, Color.kGold).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), spacing);

}

@Override
public void execute() {
    //applies the pattern to the buffer
    //meaning, we're now telling the buffer which runs periodically to update color values as the pattern dictates
    pattern.applyTo(light.getBuffer());
}

@Override
public boolean isFinished() {
    //never finishes running until interrupted
    return false;
}

}