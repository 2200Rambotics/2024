package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ExtraMath;

public class LEDSubsystem extends SubsystemBase {
    AddressableLED ledStrip;
    AddressableLEDBuffer buffer;
    Timer timer;

    Strip fullStrip;
    Strip[] strips;

    LimelightSubsystem limelight;
    PowerDistribution pdp;
    ShooterSubsystem shooter;

    boolean[] conditions;
    int functionIndex = -1;

    AnalogInput micInput;

    public LEDSubsystem(LimelightSubsystem limelight, ShooterSubsystem shooter) {
        this.limelight = limelight;
        this.pdp = pdp;
        this.shooter = shooter;

        fullStrip = new Strip(0, 87);
        strips = new Strip[] {
                new Strip(0, 10), // FLStrip
                new Strip(21, 11), // LFStrip
                new Strip(22, 32), // LBStrip
                new Strip(43, 33), // BLStrip
                new Strip(44, 54), // BRStrip
                new Strip(65, 55), // RBStrip
                new Strip(66, 76), // RFStrip
                new Strip(87, 77), // FRStrip
        };

        int length = fullStrip.numLEDs;
        buffer = new AddressableLEDBuffer(length);
        ledStrip = new AddressableLED(Constants.LED_STRIP_ID);
        ledStrip.setLength(length);
        ledStrip.start();
        timer = new Timer();
        conditions = new boolean[3];

        micInput = new AnalogInput(0);
        micInput.setAverageBits(250);
    }

    private class Strip {
        // Both start and end are inclusive
        public final int start;
        public final int end;
        public final int direction;
        public final int numLEDs;

        public Strip(int start, int end) {

            this.start = start;
            this.end = end;

            numLEDs = Math.abs(start - end) + 1;

            if (start < end) {
                direction = 1;
            } else {
                direction = -1;
            }
        }
    }

    @Override
    public void periodic() {
        checkConditions();
        priorityCheck();
        switch (functionIndex) {
            case 0:
                setColour(fullStrip, Color.kWhite);
                break;
            case 1:
                limelightShotDisplay();
                break;
            case 2:
                vuMode();
                break;
            default:
                setColour(fullStrip, Color.kBlack);
                // displayVoltage();
                break;
        }
        ledStrip.setData(buffer);
    }

    /** Checks conditions for all LED methods. */
    public void checkConditions() {
        for (int i = 0; i < conditions.length; i++) {
            conditions[i] = false;
        }
        if (shooter.intakeBottom.getCurrent() > 5) {
            conditions[0] = true;
        }
        if (limelight.isAiming || limelight.readyToShoot) {
            conditions[1] = true;
        }
        if (DriverStation.isDisabled()) {
            conditions[2] = true;
        }
        // if (limelight1.resultLength() > 0) {
        //     conditions[0] = true;
        // }
    }

    /** Based on the conditions, decides which module to use. */
    public void priorityCheck() {
        functionIndex = -1;
        for (int i = 0; i < conditions.length; i++) {
            if (conditions[i]) {
                functionIndex = i;
                break;
            }
        }
    }

    /**  Clamps the index of the LED to "safely" set the LED to a buffer.
     * @param index The index of the strip.
     * @param color The desired colour of the index.
     */
    public void safeSetLED(int index, Color color) {
        int clampedIndex = ExtraMath.clamp(index, 0, buffer.getLength());
        buffer.setLED(clampedIndex, color);
    }

    /** Sets the strip to one static colour.
     * @param strip The strip to display the colour to.
     * @param color The desired colour of the strip.
     */
    public AddressableLEDBuffer setColour(Strip strip, Color color) {
        for (int i = strip.start; i != strip.end + strip.direction; i += strip.direction) {
            safeSetLED(i, color);
        }
        return buffer;
    }

    /** Given two colours, draws the first to a specific percentage of the strip length, filled in with the 2nd colour.
     * @param strip The strip to display to.
     * @param percentage The percentage of the colour to display.
     * @param color1 The foreground colour.
     * @param color2 The background colour.
    */
    public void twoColourProgressBar(Strip strip, double percentage, Color color1,
    Color color2) {
        percentage = ExtraMath.clamp(percentage, 0, 1);
        int numLEDs = (int) (strip.numLEDs * percentage);
        setColour(strip, color2);
        for (int i = strip.start; i != numLEDs * strip.direction + strip.start; i += strip.direction) {
            safeSetLED(i, color1);
        }
    }
    
    /** Draws cursors that vary in position and size depending on the location of notes */
    public void followNote() {
        // setColour(Color.kBlack, buffer, fullStrip);
        // for (int i = 0; i < limelight1.resultLength(); i++) {
        //     int size = (int) ExtraMath.rangeMap(limelight1.getTargets()[i].ta, 0, 1,
        //     fullStrip.start, fullStrip.end);
        //     Color color;
        // if (limelight1.getTargets()[i].className.equals("redbobot")) {
        // color = Color.kRed;
        // } else if (limelight1.getTargets()[i].className.equals("bluebobot")) {
        //     color = Color.kBlue;
        //     } else if (limelight1.resultLargestAreaTarget() == i) {
        //         color = Color.kWhite;
        // } else {
        //     color = Color.kOrangeRed;
        //     }
        // drawCursor(limelight1.getTargets()[i].tx, -29.8, 29.8, fullStrip,
        // showingBuffer, color, size);
        // }
    }
    
    /** Gets voltage from the PDP and displays it as a percentage */
    public void displayVoltage() {
        double voltage = pdp.getVoltage();
        final double minVoltage = 9;
        final double maxVoltage = 12;
        double percentageVoltage = (voltage - minVoltage) / (maxVoltage - minVoltage);
        Color color1 = Color.kGreen;
        Color color2 = Color.kBlack;
        twoColourProgressBar(fullStrip, percentageVoltage, color1, color2);
    }
    
    
    /** API for drawing a cursor on the LED strip
     * @param val The location of the cursor to be placed.
     * @param min The minimum value of the cursor range.
     * @param max The maximum value of the cursor range.
     * @param strip The strip to display to.
     * @param color The desired colour of the cursor.
     * @param size The width of the cursor in number of LEDs.
     */
    public void drawCursor(double val, double min, double max, Strip strip, Color color, int size) {
        int centerLED = (int) ExtraMath.rangeMap(val, min, max, strip.start, strip.end);
        int halfSize = (int) (size - 1) / 2;
        for (int i = centerLED - halfSize; i <= centerLED + halfSize; i++) {
            safeSetLED(i, color);
        }
    }
    
    /** Colours the entire strip orange when a note is visible */
    public void seeingNote() {
        setColour(fullStrip, Color.kOrangeRed);
    }

    /** Colours the LEDs yellow when the limelight is aiming itself onto the speaker.
     * Colours the LEDs green once the robot is ready to shoot. */
    public void limelightShotDisplay() {
        if (limelight.readyToShoot) {
            setColour(fullStrip, Color.kGreen);
        } else if (limelight.isAiming) {
            setColour(fullStrip, Color.kYellow);
        }
    }

    /** Displays a VU Meter to bounce along with the music. */
    public void vuMode() {
        for (int i = 0; i < strips.length; i++) {
            int micVal = (int) ExtraMath.rangeMap(micInput.getAverageVoltage(), 1, 1.05, 0, 10);
            // twoColourProgressBar(strips[i], showingBuffer, micVal, Color.kGreen, Color.kRed);
            for (int j = strips[i].start; j != micVal * strips[i].direction + strips[i].start; j += strips[i].direction) {
                if(ExtraMath.within(j, strips[i].start, 10)){
                    safeSetLED(j, Color.kRed);
                }
                if(ExtraMath.within(j, strips[i].start, 8)){
                    safeSetLED(j, Color.kYellow);
                }
                if(ExtraMath.within(j, strips[i].start, 6)){
                    safeSetLED(j, Color.kGreen);
                }
            }
            SmartDashboard.putNumber("Mic Input", micInput.getAverageVoltage());
        }
    }
}
