package Team4450.Robot25.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.Map;
import java.util.function.DoubleSupplier;

import Team4450.Lib.Util;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * WPILib has an API for controlling WS2812, WS2812B, and WS2815 LEDs 
 * with their data pin connected via PWM. This class wraps that API in
 * a simpler, easy to use API. 
 * See <a target="_blank" href="https://docs.wpilib.org/en/latest/docs/software/hardware-apis/misc/addressable-leds.html"><b>WPILib</b></a> 
 * for more information.
 * See <a target="_blank" href="https://docs.thethriftybot.com/thrifty-12v-addressable-leds/wiring"><b>Wiring instructions</b></a> 
 * for more information.
 */
public class LEDStrip  extends SubsystemBase
{
    private AddressableLED          ledStrip;
    private AddressableLEDBuffer    ledBuffer;
    private LEDPattern              pattern;
    private boolean                 reverse;
    private double                  blink, breathe;
    private Dimensionless           brightness = Percent.of(100); 
    private Map<Double, Color>      maskSteps;
    private DoubleSupplier          progress;
    private int                     viewIndex;

    private ArrayList <AddressableLEDBufferView> bufferViews = new ArrayList<AddressableLEDBufferView>();
    private ArrayList <LEDPattern> viewPatterns = new ArrayList<LEDPattern>();
    
    // LED strip has a density of n LEDs per meter.
    private final                   Distance ledSpacing;

    /**
     * Create instance of LEDStrip to control an led strip wired to a 
     * PWM port.
     * @param port PWM port.
     * @param numberOfLeds Number of leds in strip.
     * @param ledDensity Number of leds per meter.
     */
    public LEDStrip(int port, int numberOfLeds, double ledDensity)
    {
        Util.consoleLog("port=%d  leds=%d", port, numberOfLeds);

        ledStrip = new AddressableLED(port);
        ledSpacing = Meters.of(1 / ledDensity);
        ledBuffer = new AddressableLEDBuffer(numberOfLeds);
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.start();
    }

    /**
     * Set brightness as percent of normal (100%).
     * @param percent Percent brightness.
     */
    public void setBrightness(int percent)
    {
        brightness = Percent.of(percent);
    }

    /**
     * Turn on/off blink for solid colors. Call before setting
     * color.
     * @param seconds Number of seconds on then off. Zero to stop.
     */
    public void setBlink(double seconds)
    {
        blink = seconds;
    }

    /**
     * Turn on/off breathing effect . Call before setting 
     * color/pattern.
     * @param seconds Number of seconds for a breath. Zero to stop.
     */
    public void setBreath(double seconds)
    {
        breathe = seconds;
    }
    
    /**
     * Toogles the direction of scrolling. Call before setting
     * color/pattern.
     * @param reversed True to reverse (right to left).
     */
    public void setReversed(boolean reversed)
    {
        reverse = reversed;
    }

    /**
     * Enables panning effect. Call before setting pattern. To pan
     * a solid color, call setColorPan().
     * @param panOn True to enable panning effect.
     */
    public void setPanEffect(boolean panOn)
    {
        if (panOn)
            maskSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);
        else
            maskSteps = null;
    }

    /**
     * Set a progress mask on a solid color driven by an external
     * function. Supplier function called on each period to update
     * the led strip. Call before setting color/pattern.
     * @param progress Progress supplier function or null to disable. 
     * values must be 0 to 1 representing % complete.
     */
    public void setProgress(DoubleSupplier progress)
    {
        this.progress = progress;
    }

    /**
     * The led strip can be segmented into "views" which are ranges of leds
     * on the strip. Colors/patterns can be set for individual views. This 
     * function creates a view with a range of leds. The views are numbered
     * 1,2, etc as they are created. Create views before setting any color/pattern.
     * Color/pattern is assigned to a view with the setViewIndex() function.
     * @param startIndex Starting led position (0 based).
     * @param endIndex Ending led position (0 based inclusive).
     */
    public void addView(int startIndex, int endIndex)
    {
        AddressableLEDBufferView bv = ledBuffer.createView(startIndex, endIndex);

        bufferViews.add(bv);

        viewPatterns.ensureCapacity(bufferViews.size());

        Util.consoleLog("bv size=%d  vp size=%d", bufferViews.size(), viewPatterns.size());
    } 

    /**
     * Sets the view to which a color/pattern is assigned. Call after setting
     * up views and before setting a color/pattern.
     * @param index Index of the view to apply next color/pattern to. (1 based).
     */
    public void setViewIndex(int index)
    {
        if (index < 0 || index > bufferViews.size()) throw new IndexOutOfBoundsException("View index out of range");

        viewIndex = index;
    }

    /**
     * Reset customizations to default values. Call before setting up a new
     * color/pattern. Does not reset views.
     */
    public void reset()
    {
        setBlink(0);
        setBreath(0);
        setBrightness(100);
        setPanEffect(false);
        setReversed(false);
        setProgress(null);
    }

    // Apply patterns to data buffer and send to strip.
    private void updateBuffer()
    {
        if (bufferViews.size() == 0) 
            pattern.applyTo(ledBuffer);
        else 
            for (int idx = 0; idx < viewPatterns.size(); idx++)
            {
                viewPatterns.get(idx).applyTo(bufferViews.get(idx));
            }

        // Write the data buffer to the LED strip
        ledStrip.setData(ledBuffer);
    }

    /**
     * Set strip to solid color with panning effect.
     * @param color The color to set. Black to turn strip off.
     * @param speed Speed of scrolling in m/s.
     */
    public void setColorPan(Color color, double speed)
    {
        pattern = LEDPattern.solid(color);
        
        setPanEffect(true);

        setPattern(pattern, speed);
    }

    /**
     * Set strip to solid color.
     * @param color The color to set. Black to turn strip off.
     */
    public void setColor(Color color)
    {
        pattern = LEDPattern.solid(color);
        
        setPattern(pattern, 0);
    }

    /**
     * Set the rainbow pattern.
     * @param speed Speed at which to scroll rainbow (m/s).
     */
    public void setRainbow(double speed)
    {
        setPattern(LEDPattern.rainbow(255, 255), speed);
    }

    /**
     * Set gradient pattern.
     * @param type Type of gradient.
     * @param startColor Starting color.
     * @param endColor Ending color.
     * @param speed Speed at which to scroll gradient (m/s).
     */
    public void setGradient(GradientType type, Color startColor, Color endColor, double speed)
    {
        setPattern(LEDPattern.gradient(type, startColor, endColor), speed);
    }

    /**
     * Set an led pattern.
     * @param pattern Patter selected from built in types.
     * @param speed Speed at which to scroll pattern (m/s).
     */
    public void setPattern(LEDPattern pattern, double speed)
    {
        this.pattern = pattern.atBrightness(brightness);
        
        if (blink != 0) this.pattern = this.pattern.blink(Seconds.of(blink));

        if (breathe != 0) this.pattern = this.pattern.breathe(Seconds.of(breathe));

        if (progress != null)
        {
            LEDPattern mask = LEDPattern.progressMaskLayer(progress);

            this.pattern = this.pattern.mask(mask);

            if (viewIndex != 0) savePattern(viewIndex, this.pattern);

            return;
        }

        if (speed == 0)
        {
            if (viewIndex != 0) savePattern(viewIndex, this.pattern);

            return;
        }

        // Create a new pattern that scrolls the rainbow pattern across the LED strip, 
        // moving at speed (m/s).

        if (maskSteps != null)
        {
            LEDPattern mask = LEDPattern.steps(maskSteps);

            this.pattern = this.pattern.mask(mask);
        }

        this.pattern = this.pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(speed), ledSpacing);

        if (reverse) this.pattern = this.pattern.reversed();

        if (viewIndex != 0) savePattern(viewIndex, this.pattern);
    }

    private void savePattern(int index, LEDPattern pattern)
    {
        if (index > viewPatterns.size())
            viewPatterns.add(index - 1, pattern);
        else
            viewPatterns.set(index - 1, pattern);
    }

    @Override
	public void periodic()
    {
        updateBuffer();
    }
}
