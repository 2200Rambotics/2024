package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDMotor {
    String name;
    boolean initialized = false;

    ControlType controlType;

    double p, i, d, f;
    double target = 0.0;
    double finalTarget = 0;
    double pidfEpsilonFactor = 1.001;
    double maxV;
    double maxA;

    RelativeEncoder encoder;
    CANSparkMax motor;
    SparkPIDController controller;

    TrapezoidProfile motorProfile;
    TrapezoidProfile.Constraints motorConstraints;
    TrapezoidProfile.State motorCurrentState;
    TrapezoidProfile.State motorTargetState;
    Timer motorTimer;
    private final int sleepTime = 20;

    private PIDMotor(int deviceID, String name, double p, double i, double d, double f, ControlType type, double maxV, double maxA) {
        this.name = name;
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.controlType = type;
        this.maxV = maxV;
        this.maxA = maxA;

        motor = new CANSparkMax(deviceID, MotorType.kBrushless);
        controller = motor.getPIDController();
        encoder = motor.getEncoder();

        motorConstraints = new Constraints(maxV, maxA);
        motorCurrentState = new TrapezoidProfile.State(0, 0);
        motorTargetState = new TrapezoidProfile.State(0, 0);
        motorProfile = new TrapezoidProfile(motorConstraints, motorTargetState, motorCurrentState);
        motorTimer = new Timer();
    }

    /**
     * Constructs and initializes a new PIDMotor.
     * 
     * @param deviceID         The CAN ID for this motor.
     * @param name             This motor's name.
     * @param p                The initial proportional coefficient for this motor.
     * @param i                The initial integral coefficient for this motor.
     * @param d                The initial derivative coefficient for this motor.
     * @param f                The initial feed-forward coefficient for this motor.
     * @param type             The control type of this motor.
     * @return The constructed PIDMotor.
     */
    public static PIDMotor makeMotor(int deviceID, String name, double p, double i, double d, double f, ControlType type) {
        return makeMotor(deviceID, name, p, i, d, f, type,10000,10000);
    }

    /**
     * Constructs and initializes a new PIDMotor.
     * 
     * @param deviceID         The CAN ID for this motor.
     * @param name             This motor's name.
     * @param p                The initial proportional coefficient for this motor.
     * @param i                The initial integral coefficient for this motor.
     * @param d                The initial derivative coefficient for this motor.
     * @param f                The initial feed-forward coefficient for this motor.
     * @param type             The control type of this motor.
     * @param maxV             The maximum velocity of the motor.
     * @param maxA             The maximum acceleration of the motor.
     * @return The constructed PIDMotor.
     */
    public static PIDMotor makeMotor(int deviceID, String name, double p, double i, double d, double f, ControlType type,
            double maxV, double maxA) {
        PIDMotor motor = new PIDMotor(deviceID, name, p, i, d, f, type, maxV, maxA);
        motor.init();
        System.out.println("Finished initializing " + name);
        return motor;
    }

    /**
     * Throws an exception if motor failed to initialize.
     */
    private void catchUninit() {
        if (!initialized) {
            new Exception("PIDMotor `" + name + "` has not been initialized! Call `init()` before using the motor!").printStackTrace();
        }
    }

    /**
     * The initialization of the motor, only ever called through the makeMotor function.
     */
    private void init() {
        if (!initialized) {

            motor.restoreFactoryDefaults();
            sleep();
            resetAll();
            sleep();
            // putPIDF();
            updatePIDF();
            sleep();
            initialized = true;
        }
        motor.setIdleMode(IdleMode.kBrake);
    }

    public void sleep() {
                    try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
    }

    public void setIdleCoastMode(){
        motor.setIdleMode(IdleMode.kCoast);
    }

    public void setIdleBrakeMode(){
        motor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Gets whether or not any of the PIDF values require updating.
     * 
     * @return Whether or not any of the PIDF values differ from the values in the motor controller.
     */
    public boolean pidfRequiresUpdate() {
        catchUninit();
        return (ExtraMath.withinFactor(controller.getP(), this.p, pidfEpsilonFactor)
                || ExtraMath.withinFactor(controller.getI(), this.i, pidfEpsilonFactor)
                || ExtraMath.withinFactor(controller.getD(), this.d, pidfEpsilonFactor)
                || ExtraMath.withinFactor(controller.getFF(), this.f, pidfEpsilonFactor));
    }

    /**
     * Puts this motor's PIDF values to the SmartDashboard.
     */
    public void putPIDF() {
        SmartDashboard.putNumber(name + " P", p);
        SmartDashboard.putNumber(name + " I", i);
        SmartDashboard.putNumber(name + " D", d);
        SmartDashboard.putNumber(name + " F", f);
    }

    /**
     * Puts the position and velocity values to the SmartDashboard.
     */
    public void putPV() {
        SmartDashboard.putNumber(name + " Position", getPosition());
        SmartDashboard.putNumber(name + " Velocity", getVelocity());
    }

    /**
     * Sets the PIDF values for this motor. Call `updatePIDF` to send the values to the motor
     * controller.
     * 
     * @param p The new proportional coefficient.
     * @param i The new integral coefficient.
     * @param d The new derivative coefficient.
     * @param f The new feed-forward coefficient.
     */
    public void setPIDF(double p, double i, double d, double f) {
        catchUninit();
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        updatePIDF();
    }

    /**
     * Gets the position of the encoder in degrees.
     * 
     * @return The position of the encoder in degrees.
     */
    public double getDegrees() {
        return encoder.getPosition() * 360;
    }

    /**
     * Fetches the PIDF values from the SmartDashboard. Call `updatePIDF` to send the values to the motor controller.
     */
    public void fetchPIDFFromDashboard() {
        catchUninit();
        setPIDF(SmartDashboard.getNumber(name + " P", p), SmartDashboard.getNumber(name + " I", i), SmartDashboard.getNumber(name + " D", d),
                SmartDashboard.getNumber(name + " F", f));
    }

    /**
     * Sends the PIDF values to the motor controller. Call when PIDF values are changed.
     */
    public void updatePIDF() {
        controller.setP(p);
        sleep();
        controller.setI(i);
        sleep();
        controller.setD(d);
        sleep();
        controller.setFF(f);
        sleep();
    }

    /**
     * Resets the encoder, making its current position 0.
     */
    public void resetEncoder() {
        encoder.setPosition(0);
    }

    public void follow(PIDMotor other, boolean inverted) {
        motor.follow(other.motor, inverted);
    }

    /**
     * Resets the controller's integral accumulation. Call this every time this motor is enabled.
     */
    public void resetIAccum() {
        controller.setIAccum(0);
    }

    /**
     * Resets the state of the motor. Call this every time this motor is enabled.
     */
    public void resetAll() {
        resetEncoder();
        sleep();
        resetIAccum();
        sleep();
    }

    // /**
    //  * Converts the units for this motor into encoder rotations.
    //  * 
    //  * @param units The units specified by this motor to convert.
    //  * @return The number of rotations that correspond to the given units.
    //  */
    // public double unitsToRotations(double units) {
    //     return units * rotationsPerUnit;
    // }

    // /**
    //  * Converts encoder rotations into units for this motor.
    //  * 
    //  * @param units The number of rotations to convert.
    //  * @return The units specified by this motor that correspond to the given rotations.
    //  */
    // public double rotationsToUnits(double rotations) {
    //     return rotations / rotationsPerUnit;
    // }

    /**
     * Sets the motor's target to a given unit value.
     */
    public void setTarget(double target) {
        this.target = target;
        controller.setReference(target, controlType);
    }

    /**
     * Sets the motor to a given speed as a fraction of the maximum output, overriding the PID controller.
     * 
     * @param speed A fraction from -1 to 1 specifying the power to set this motor to.
     */
    public void setPercentOutput(double speed) {
        catchUninit();
        motor.set(speed);
    }

    /**
     * Sets the control type for this motor.
     * 
     * @param ctype The control type to target.
     */
    public void setControlType(ControlType ctype) {
        catchUninit();
        this.controlType = ctype;
    }

    /**
     * Sets this motor to have inverse rotation.
     * 
     * @param state Whether or not to make this motor inverted.
     */
    public void setInverted(boolean state) {
        motor.setInverted(state);
    }

    /**
     * Gets the encoder's current position.
     * 
     * @return Position in number of rotations.
     */
    public double getPosition() {
        return encoder.getPosition();
    }

    /**
     * Gets whether the current position of the motor is within 10 revolutions of the target position.
     * 
     * @return Whether position at target.
     */
    public boolean atPosition() {
        return atPosition(10);
    }

    public boolean atPosition(double epsilon) {
        return ExtraMath.within(finalTarget, getPosition(), epsilon);
    }


    /**
     * Gets the encoder's current velocity.
     * 
     * @return Velocity in rotations per second.
     */
    public double getVelocity() {
        return encoder.getVelocity() / 60;
    }

    /**
     * Gets whether the current velocity of the motor is within the desired threshold RPM of the target velocity.
     * 
     * @param threshold The threshold of acceptable desired velocity.
     * @return Whether velocity is at desired target.
     */
    public boolean atVelocity(double threshold) {
        return ExtraMath.within(target, getVelocity(), threshold);
    }

    /**
     * Generates a trapezoidal path for the motor to follow.
     * 
     * @param targetP The intended target position of the motor.
     * @param targetV The intended target velocity of the motor.
     */
    public void generateTrapezoidPath(double targetP, double targetV) {
        generateTrapezoidPath(targetP, targetV, maxV);
    }

    /**
     * Generates a trapezoidal path for the motor to follow.
     * 
     * @param targetP The intended target position of the motor.
     * @param targetV The intended target velocity of the motor.
     */
    public void generateTrapezoidPath(double targetP, double targetV, double maxV) {
        Constraints newConstraints = new Constraints(maxV, maxA);
        this.target = targetP;
        this.finalTarget = targetP;

        // Try not creating new object
        motorCurrentState.position = getPosition();
        motorCurrentState.velocity = getVelocity();
        // motorCurrentState = new TrapezoidProfile.State(getPosition(), getVelocity());

        // Try not creating new object
        motorTargetState.position = targetP;
        motorTargetState.velocity = targetV;
        // motorTargetState = new TrapezoidProfile.State(targetP, targetV);

        motorProfile = new TrapezoidProfile(newConstraints, motorTargetState, motorCurrentState);
        motorTimer.restart();
    }

    /**
     * After generating a trapezoidal path, run this function periodically to update the target of the PIDMotor.
     */
    public void runTrapezoidPath() {
        TrapezoidProfile.State state = motorProfile.calculate(motorTimer.get());
        setTarget(state.position);
        // SmartDashboard.putNumber(name+" Trapezoid Output", state.position);
        // SmartDashboard.putNumber(name + " Position Error", getPosition() - state.position);
    }

    /**
     * Sets the current limit of the motor.
     * @param limit Current limit in amps.
     */
    public void setCurrentLimit(int limit){
        motor.setSmartCurrentLimit(limit);
    }

    public double getCurrent(){
        return motor.getOutputCurrent();
    }

    public void putCurrent(){
        SmartDashboard.putNumber(name+" Current", motor.getOutputCurrent());
    }
}
