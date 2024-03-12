package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.PIDMotor;

public class ClimberSubsystem extends SubsystemBase {
    static final double HEIGHT = 0.0;

    static final int LEFT_SERVO_CHANNEL = 7;
    static final int RIGHT_SERVO_CHANNEL = 8;
    public static final double LEFT_SERVO_ENGAGE_POS = 0.5;
    public static final double LEFT_SERVO_RELEASE_POS = 0.42;
    public static final double RIGHT_SERVO_ENGAGE_POS = 0.38;
    public static final double RIGHT_SERVO_RELEASE_POS = 0.5;
    static final double ENGAGE_BACKING_DISTANCE = 3.0;

    public double target = 0.0;
    boolean ratchetEngaged = false;
    boolean motorDisabled = false;
    public Timer climbTimer;

    public PIDMotor leftMotor = PIDMotor.makeMotor(Constants.CLIMBER_LEFT_ID, "Climber Left", 0.02, 0, 0, 0,
            ControlType.kPosition);
    public PIDMotor rightMotor = PIDMotor.makeMotor(Constants.CLIMBER_RIGHT_ID, "Climber Right", 0.02, 0, 0, 0,
            ControlType.kPosition);
    public Servo leftRatchetServo = new Servo(LEFT_SERVO_CHANNEL);
    public Servo rightRatchetServo = new Servo(RIGHT_SERVO_CHANNEL);

    public enum ClimbState {
        Max, Min, Mid, Stowed, Compact;

        public double height(){
        switch(this){
            case Max: return Constants.CLIMBER_HIGH_HEIGHT;
            case Mid: return Constants.CLIMBER_MID_HEIGHT;
            case Min: return Constants.CLIMBER_LOW_HEIGHT;
            case Stowed: return Constants.CLIMBER_STOWED_HEIGHT;
            case Compact: return Constants.CLIMBER_LOW_HEIGHT;
            default: return 0;
        }
    }
    }

    public ClimberSubsystem() {
        climbTimer = new Timer();
        leftMotor.setCurrentLimit(30);
        rightMotor.setCurrentLimit(30);
    }

    /**
     * Position of the climbers.
     * 
     * @return The average encoder position of the two climber motors.
     */
    public double position() {
        return ExtraMath.average(leftMotor.getPosition(), rightMotor.getPosition());
    }

    /**
     * Sets a new desired height for the climbers.
     * 
     * @param newHeight
     */
    public void setHeight(double newHeight) {
        target = newHeight;
    }

    /**
     * Engages the ratchet, which locks the climber mechanism.
     */
    public void engageRatchet() {
        ratchetEngaged = true;
    }

    /**
     * Releases the ratchet, which unlocks the climber mechanism.
     */
    public void disengageRatchet() {
        ratchetEngaged = false;
    }

    @Override
    public void periodic() {
        if(!motorDisabled){
            leftMotor.setTarget(target);
            rightMotor.setTarget(-target);
            climbTimer.stop();
            climbTimer.reset();
        } else{
            if(climbTimer.get() == 0){
                climbTimer.restart();
            }
            if(climbTimer.get() > 0 && climbTimer.get() < 5){
                leftMotor.setTarget(target);
                rightMotor.setTarget(-target);
            }
            if(climbTimer.get() >= 5){
                leftMotor.setPercentOutput(0);
                rightMotor.setPercentOutput(0);
            }
        }
        leftRatchetServo.setPosition(ratchetEngaged ? LEFT_SERVO_ENGAGE_POS : LEFT_SERVO_RELEASE_POS);
        rightRatchetServo.setPosition(ratchetEngaged ? RIGHT_SERVO_ENGAGE_POS : RIGHT_SERVO_RELEASE_POS);
    }

    /**
     * Toggles the ratchet between a locked and unlocked state.
     */
    public void toggleRatchet(){
        if(ratchetEngaged){
            ratchetEngaged = false;
        } else{
            ratchetEngaged = true;
        }
    }

    public void disableBrakeMode(){
      leftMotor.setIdleCoastMode();
      rightMotor.setIdleCoastMode();
    }
  
    public void enableBrakeMode(){
      leftMotor.setIdleBrakeMode();
      rightMotor.setIdleBrakeMode();
    }
  
    public void zeroEncoders(){
      leftMotor.resetEncoder();
      rightMotor.resetEncoder();
    }

    public void disableMotors(boolean disabled){
        motorDisabled = disabled;
    }
}
