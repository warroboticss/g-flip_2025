package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LightSubsystem extends SubsystemBase {
    // PWM Port the LED Strip is connected to (you will need to update this to the PWM Port we used)
    private static final int kPort = 9;
    // Number of LEDs on the strip (this is the correct number, unless we cut the strip)
    // Note, you can calculate this, its equal to the number of LEDs per meter (60) * meters
    private static final int kLength = 300;

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;

    public LightSubsystem() {
        //initializing our LED controller and buffer
        m_led = new AddressableLED(kPort);
        m_buffer = new AddressableLEDBuffer(kLength);

        //setting the length of the strip, and buffer
        m_led.setLength(kLength);
        m_led.setData(m_buffer);
        m_led.start(); //Starting LED output

        //this sets the LEDs to be black on default
        //setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));
    }

    @Override
    public void periodic() {
        /*pushes buffer data to the strip periodically
        essentially, the buffer stores the data for the color value for every LED in our strip
        by updating it periodically, we can feed it new values and have our changes reflected in real life*/
        m_led.setData(m_buffer);
    }

    public AddressableLEDBuffer getBuffer() {
        return m_buffer;
        //returns the buffer so we can use it in our command
    }
    //simple command that allows us to set a pattern, we're using it above when we set our default command
    public Command runPattern(LEDPattern pattern) {
        return Commands.run(() -> pattern.applyTo(m_buffer), this);
    }
}
