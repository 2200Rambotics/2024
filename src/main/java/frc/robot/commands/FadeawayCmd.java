package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.RobotContainer;
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

public class FadeawayCmd extends Command {
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

    final double[][] wristPosition = {
        // practice robot
            // { 8.3, 23 },
            // { 17, 25 },
            // { 33, 34 }
            // comp bot
            { 15.4, 24 },//back bumper on wing line
            { 19.4, 25.5   },//half way between wing line and game piece line
            { 24.4, 28.3 },//front bumper on game piece line
            { 33, 32.5 },//front bumper on starting line
            { 39.3, 37.5 }// centered on starting line
    };

    final double[][] shooterSpeed = {
        { 15, 10500 },
        { 40, 9500 }     
    };
    
    LinearInterpolation wrist;
    LinearInterpolation shooterRPM;

    public FadeawayCmd(FloorIntakeSubsystem floorIntake, ShooterSubsystem shooter, ArmSubsystem arm,
            LimelightSubsystem limelight, Telemetry logger) {
        this.floorIntake = floorIntake;
        this.shooter = shooter;
        this.arm = arm;
        this.logger = logger;
        this.limelight = limelight;
        addRequirements(floorIntake, shooter, arm);
        wrist = new LinearInterpolation(wristPosition);
        shooterRPM = new LinearInterpolation(shooterSpeed);
    }

    /**
     * Sets the target position of the arm to the Popcorn position.
     */
    @Override
    public void initialize() {
                    RobotContainer.speedMultiplier = 0.35;

        limelight.setPipeline(0);
        
    }

    /**
     * Once the robot reaches the Intake position, run the floorIntake and shooter
     * intake motors to either take in or spit out a note.
     */
    @Override
    public void execute() {
        limelight.limelightRotation = limelight.tagTv;
        

        // Intake Control
        shooter.shooterState = ShooterState.SpinLimelight;
        floorIntake.set(FloorIntakeState.Eat);
        shooter.intakeState = IntakeState.ShootNow;

        if (shooter.shooterV < 100)
            shooter.shooterV = 8000;

       
        if (limelight.tagTv) {
            double x = wrist.interpolate(limelight.tagTy);
            x = x + -1.5 * logger.getVelocityX();
            arm.safeManualLimelightSetPosition(0, x, 0, false);
            shooter.shooterV = shooterRPM.interpolate(limelight.tagTy);
            if(ExtraMath.within(limelight.tagTx, 10*logger.getVelocityY(), Constants.SHOOTER_ALLOWED_X_OFFSET)){
                limelight.readyToShoot = true;
                limelight.isAiming = false;
            } else {
                limelight.readyToShoot = false;
                limelight.isAiming = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        floorIntake.set(FloorIntakeState.Spit);
        RobotContainer.speedMultiplier = 1;


        limelight.limelightRotation = false;
        shooter.shooterState = ShooterState.Idle;
        shooter.spinDownShooters();
        arm.isTrapezoidal = true;
        arm.unsafeSetPosition(ArmPosition.Stowed);
        shooter.intakeState = IntakeState.Idle;
        limelight.isAiming = false;
        limelight.readyToShoot = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}