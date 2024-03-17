package frc.robot.commands;

import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.drive.Telemetry;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.FloorIntakeSubsystem.FloorIntakeState;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem.IntakeState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeWhileShootAutoCmd extends Command {
    private final FloorIntakeSubsystem floorIntake;
    private final ShooterSubsystem shooter;
    private final ArmSubsystem arm;
    ArmPosition target = ArmPosition.Popcorn;
    LimelightSubsystem limelight;
    Telemetry logger;
    CommandSwerveDrivetrain drivetrain;

    boolean shoulderSetCheck = false;
    boolean wristSetCheck = false;
    boolean elevatorSetCheck;

    boolean isDone;
    Timer shooterTimer;

    double[][] wristPosition = {
            // { 8.3, 22 },
            // { 17, 28.5 },
            // { 33, 36.5 }
            { 8.3, 23 },
            { 17, 25 },
            { 33, 34 } // adjusted values for the competition robot
    };

    double[][] shooterSpeed = {
            { 8.3, 10500 },
            { 17, 10000 },
            { 33, 9500 }
    };
    LinearInterpolation wrist;
    LinearInterpolation shooterRPM;

    public IntakeWhileShootAutoCmd(FloorIntakeSubsystem floorIntake, ShooterSubsystem shooter, ArmSubsystem arm,
            LimelightSubsystem limelight, Telemetry logger) {
        this.floorIntake = floorIntake;
        this.shooter = shooter;
        this.arm = arm;
        this.logger = logger;
        this.limelight = limelight;
        addRequirements(floorIntake, shooter, arm);
    }

    /**
     * Sets the target position of the arm to the Popcorn position.
     */
    @Override
    public void initialize() {
        // arm.unsafeSetPosition(target);

        // shooter.shooterState = ShooterState.SpinFixed;
        // shooter.shooterV = speed;
        wrist = new LinearInterpolation(wristPosition);
        shooterRPM = new LinearInterpolation(shooterSpeed);
        limelight.setPipeline(0);
        // shooterTimer = new Timer();
        // isDone = false;
        // drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Once the robot reaches the Intake position, run the floorIntake and shooter
     * intake motors to either take in or spit out a note.
     */
    @Override
    public void execute() {
        // SmartDashboard.putBoolean("Is Shoulder at
        // Position",arm.leftShoulderMotor.atPosition());
        // SmartDashboard.putBoolean("Is Wrist at
        // Position",arm.wristMotor.atPosition());
        // if (arm.leftShoulderMotor.atPosition() && arm.wristMotor.atPosition()) {

        // Intake Control
        shooter.shooterState = ShooterState.SpinLimelight;
        floorIntake.set(FloorIntakeState.Eat);
        shooter.intakeState = IntakeState.ShootNow;
        // shooter.intakeState = IntakeState.ShootNow;

        if (shooter.shooterV < 100)
            shooter.shooterV = 8000;

        // }
        if (limelight.tagTv) {
            // limelight.limelightRotation = true;
            // System.out.println("limelight rotation on");
            double x = wrist.interpolate(limelight.tagTy);
            x = x + -1.5 * logger.getVelocityX();
            // SmartDashboard.putNumber("vel x", logger.getVelocityX());
            // SmartDashboard.putNumber("Calculated Wrist Position:",
            // wrist.interpolate(tag.ty));
            arm.safeManualLimelightSetPosition(0, x, 0, false);
            shooter.shooterV = shooterRPM.interpolate(limelight.tagTy);

            // System.out.println(shooter.isShooterAtVelocity());

            // SmartDashboard.putNumber("Tag X", tag.tx);
        }
    }

    @Override
    public void end(boolean interrupted) {
        floorIntake.set(FloorIntakeState.Spit);

        // shooter.shooterState = ShooterState.Idle;
        // shooter.spinDownShooters();

        limelight.limelightRotation = false;
        shooter.shooterState = ShooterState.Idle;
        shooter.spinDownShooters();
        arm.isTrapezoidal = true;
        arm.unsafeSetPosition(ArmPosition.Stowed);

        shooter.intakeState = IntakeState.Idle;

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}