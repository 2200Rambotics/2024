package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ExtraMath;

public class LEDSubsystem extends SubsystemBase implements Runnable {
    AddressableLED ledStrip;
    AddressableLEDBuffer buffer;
    Timer timer;

    Strip fullStrip;
    Strip[] strips;
    Strip[] halfStrips;

    LimelightSubsystem limelight;
    PowerDistribution pdp;
    ShooterSubsystem shooter;

    boolean[] conditions;
    int functionIndex = -1;

    AnalogInput micInput;
    // double micVal = 5;
    double volumeLow = 5;
    double volumeHigh = 0;

    int[] cursorPositions = { 0, 1, 2 };
    boolean sirenState = false;
    public boolean exciteMode = false;

    Color BetterRed = new Color(75, 0, 0);
    Color BetterBlue = new Color(0, 0, 75);
    Color BetterWhite = Color.kViolet;

    public LEDSubsystem(LimelightSubsystem limelight, ShooterSubsystem shooter) {
        this.limelight = limelight;
        this.pdp = pdp;
        this.shooter = shooter;

        fullStrip = new Strip(0, 87);

        strips = new Strip[] {
                new Strip(0, 10), // Front Left
                new Strip(21, 11), // Left Front
                new Strip(22, 32), // Left Back
                new Strip(43, 33), // Back Left
                new Strip(44, 54), // Back Right
                new Strip(65, 55), // Right Back
                new Strip(66, 76), // Right Front
                new Strip(87, 77), // Front Right
        };

        halfStrips = new Strip[] {
                new Strip(0, 5), // Front Left Bottom
                new Strip(6, 10), // Front Left Top
                new Strip(15, 11), // Left Front Top
                new Strip(21, 15), // Left Front Bottom
                new Strip(22, 27), // Left Back Bottom
                new Strip(27, 32), // Left Back Top
                new Strip(37, 33), // Back Left Top
                new Strip(43, 38), // Back Left Bottom
                new Strip(44, 49), // Back Right Bottom
                new Strip(50, 54), // Back Right Top
                new Strip(59, 55), // Right Back Top
                new Strip(65, 60), // Right Back Bottom
                new Strip(66, 71), // Right Front Bottom
                new Strip(72, 77), // Right Front Top
                new Strip(81, 77), // Front Right Top
                new Strip(87, 82), // Front Right Bottom
        };

        int length = fullStrip.numLEDs;
        buffer = new AddressableLEDBuffer(length);
        ledStrip = new AddressableLED(Constants.LED_STRIP_ID);
        ledStrip.setLength(length);
        ledStrip.start();
        timer = new Timer();
        conditions = new boolean[4];

        micInput = new AnalogInput(0);
        micInput.setAverageBits(250);
        
        new Thread(this, "LED Thread").start();
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
    public void run() {
        while (true) {
            synchronized (this) {
                checkConditions();
                priorityCheck();
                switch (functionIndex) {
                    case 0:
                        sirenMode();
                        break;
                    case 1:
                        setColour(fullStrip, Color.kOrangeRed);
                        break;
                    case 2:
                        limelightShotDisplay();
                        break;
                    case 3:
                        // cursorMode();
                        vuMode();
                        // sirenMode();
                        break;
                    default:
                        // setColour(fullStrip, Color.kBlack);
                        // displayVoltage();
                        break;
                }
                ledStrip.setData(buffer);
            }
            try {
                if (exciteMode) {
                    Thread.sleep(100);
                } else {
                    Thread.sleep(20);
                }
            } catch (InterruptedException iex) {
            }
        }
    }

    /** Checks conditions for all LED methods. */
    public void checkConditions() {
        synchronized (this) {
            for (int i = 0; i < conditions.length; i++) {
                conditions[i] = false;
            }
            if(exciteMode){
                conditions[0] = true;
            }
            if (shooter.intakeBottom.getCurrent() > 5) {
                conditions[1] = true;
            }
            if (limelight.isAiming || limelight.readyToShoot) {
                conditions[2] = true;
            }
            if (DriverStation.isDisabled()) {
                conditions[3] = true;
            }
            // if (limelight1.resultLength() > 0) {
            // conditions[0] = true;
            // }
        }
    }

    /** Based on the conditions, decides which module to use. */
    public void priorityCheck() {
        synchronized (this) {
            functionIndex = -1;
            for (int i = 0; i < conditions.length; i++) {
                if (conditions[i]) {
                    functionIndex = i;
                    break;
                }
            }
        }
    }

    /**
     * Clamps the index of the LED to "safely" set the LED to a buffer.
     * 
     * @param index The index of the strip.
     * @param color The desired colour of the index.
     */
    public void safeSetLED(int index, Color color) {
        synchronized (this) {
            int clampedIndex = ExtraMath.clamp(index, 0, buffer.getLength());
            buffer.setLED(clampedIndex, color);
        }
    }

    /**
     * Sets the strip to one static colour.
     * 
     * @param strip The strip to display the colour to.
     * @param color The desired colour of the strip.
     */
    public AddressableLEDBuffer setColour(Strip strip, Color color) {
        synchronized (this) {
            for (int i = strip.start; i != strip.end + strip.direction; i += strip.direction) {
                safeSetLED(i, color);
            }
            return buffer;
        }
    }

    /**
     * Given two colours, draws the first to a specific percentage of the strip
     * length, filled in with the 2nd colour.
     * 
     * @param strip      The strip to display to.
     * @param percentage The percentage of the colour to display.
     * @param color1     The foreground colour.
     * @param color2     The background colour.
     */
    public void twoColourProgressBar(Strip strip, double percentage, Color color1,
            Color color2) {
        synchronized (this) {
            percentage = ExtraMath.clamp(percentage, 0, 1);
            int numLEDs = (int) (strip.numLEDs * percentage);
            setColour(strip, color2);
            for (int i = strip.start; i != numLEDs * strip.direction + strip.start; i += strip.direction) {
                safeSetLED(i, color1);
            }
        }
    }

    /**
     * Draws cursors that vary in position and size depending on the location of
     * notes
     */
    public void followNote() {
        synchronized (this) {
            // setColour(Color.kBlack, buffer, fullStrip);
            // for (int i = 0; i < limelight1.resultLength(); i++) {
            // int size = (int) ExtraMath.rangeMap(limelight1.getTargets()[i].ta, 0, 1,
            // fullStrip.start, fullStrip.end);
            // Color color;
            // if (limelight1.getTargets()[i].className.equals("redbobot")) {
            // color = Color.kRed;
            // } else if (limelight1.getTargets()[i].className.equals("bluebobot")) {
            // color = Color.kBlue;
            // } else if (limelight1.resultLargestAreaTarget() == i) {
            // color = Color.kWhite;
            // } else {
            // color = Color.kOrangeRed;
            // }
            // drawCursor(limelight1.getTargets()[i].tx, -29.8, 29.8, fullStrip,
            // showingBuffer, color, size);
            // }
        }
    }

    /** Gets voltage from the PDP and displays it as a percentage */
    public void displayVoltage() {
        synchronized (this) {
            double voltage = pdp.getVoltage();
            final double minVoltage = 9;
            final double maxVoltage = 12;
            double percentageVoltage = (voltage - minVoltage) / (maxVoltage - minVoltage);
            Color color1 = Color.kGreen;
            Color color2 = Color.kBlack;
            twoColourProgressBar(fullStrip, percentageVoltage, color1, color2);
        }
    }

    /**
     * API for drawing a cursor on the LED strip
     * 
     * @param val   The location of the cursor to be placed.
     * @param min   The minimum value of the cursor range.
     * @param max   The maximum value of the cursor range.
     * @param strip The strip to display to.
     * @param color The desired colour of the cursor.
     * @param size  The width of the cursor in number of LEDs.
     */
    public void drawCursor(double val, double min, double max, Strip strip, Color color, int size) {
        synchronized (this) {
            int centerLED = (int) ExtraMath.rangeMap(val, min, max, strip.start, strip.end);
            int halfSize = (int) (size - 1) / 2;
            for (int i = centerLED - halfSize; i <= centerLED + halfSize; i++) {
                safeSetLED(i, color);
            }
        }
    }

    /** Colours the entire strip orange when a note is visible */
    public void seeingNote() {
        synchronized (this) {
            setColour(fullStrip, Color.kOrangeRed);
        }
    }

    /**
     * Colours the LEDs yellow when the limelight is aiming itself onto the speaker.
     * Colours the LEDs green once the robot is ready to shoot.
     */
    public void limelightShotDisplay() {
        synchronized (this) {
            if (limelight.readyToShoot) {
                setColour(fullStrip, Color.kGreen);
            } else if (limelight.isAiming) {
                setColour(fullStrip, Color.kYellow);
            }
        }
    }

    /** Draws a fun cursor that follows the LEDs in sequence */
    public void cursorMode() {
        synchronized (this) {
            if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
                setColour(fullStrip, BetterRed);
            } else {
                setColour(fullStrip, BetterBlue);
            }
            for (int i = 0; i < cursorPositions.length; i++) {
                safeSetLED(cursorPositions[i], BetterWhite);
                cursorPositions[i]++;
                if (cursorPositions[i] == 88) {
                    cursorPositions[i] = 0;
                }
            }
        }
    }

    public void sirenMode() {
        int[] set1 = {0,3,5,6,8,11,13,14};
        int[] set2 = {1,2,4,7,9,10,12,15};
        if (sirenState) {
            for(int i = 0; i < 8; i++){
                setColour(halfStrips[set1[i]], BetterBlue);
                setColour(halfStrips[set2[i]], BetterRed);
            }
        } else {
            for(int i = 0; i < 8; i++){
                setColour(halfStrips[set1[i]], BetterRed);
                setColour(halfStrips[set2[i]], BetterBlue);
            }
        }
        sirenState = !sirenState;
    }

    /** Displays a VU Meter to bounce along with the music. */
    public void vuMode() {
        synchronized (this) {
            for (int i = 0; i < strips.length; i++) {
                setColour(strips[i], Color.kBlack);
                if(micInput.getAverageVoltage() < volumeLow){
                    volumeLow = micInput.getAverageVoltage();
                }
                if(micInput.getAverageVoltage() > volumeHigh){
                    volumeHigh = micInput.getAverageVoltage();
                }
                volumeLow += 0.001;
                volumeHigh -= 0.001;
                double micVal = (int) ExtraMath.rangeMap(micInput.getAverageVoltage(), volumeLow, volumeHigh, 0, 11.9);
                // double interval = (((int)(Math.random()*3))-1)*0.3;
                // micVal = micVal + interval;
                micVal = ExtraMath.clamp(micVal, 0, 11.9);
                for (int j = strips[i].start; j != (int) micVal * strips[i].direction
                        + strips[i].start; j += strips[i].direction) {
                    if (ExtraMath.within(j, strips[i].start, 11)) {
                        safeSetLED(j, Color.kRed);
                    }
                    if (ExtraMath.within(j, strips[i].start, 8)) {
                        safeSetLED(j, Color.kGold);
                    }
                    if (ExtraMath.within(j, strips[i].start, 6)) {
                        safeSetLED(j, Color.kGreen);
                    }
                }
                SmartDashboard.putNumber("Mic Input", micInput.getAverageVoltage());
            }
        }
    }

}
