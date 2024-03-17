package frc.robot.commands;

import frc.robot.ExtraMath;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.drive.Telemetry;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.ShooterSubsystem.IntakeState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class LimelightAutoCmd extends Command {
    ArmSubsystem arm;
    double targetShoulderPosition;
    double targetWristPosition;
    double elevatorPosition;
    LimelightSubsystem limelight;
    ShooterSubsystem shooter;
    Telemetry logger;
    CommandSwerveDrivetrain drivetrain;

    boolean shoulderSetCheck = false;
    boolean wristSetCheck = false;
    boolean elevatorSetCheck;

    boolean isDone;
    Timer shooterTimer;

    
    double[][] wristPosition = {
        // practice robot
            // { 8.3, 23 },
            // { 17, 25 },
            // { 33, 34 }
            // comp bot
            { 16, 22.5 },//back bumper on wing line
            { 20, 24.8   },//half way between wing line and game piece line
            { 24, 26.3 },//font bumper on game piece line
            { 30, 29.0 },//front bumper on starting line
            { 40, 34.5 }// centered on starting line

            // { 16, 22.0 },//back bumper on wing line
            // { 20, 24.3   },//half way between wing line and game piece line
            // { 24, 25.8 },//font bumper on game piece line
            // { 30, 28.5 },//front bumper on starting line
            // { 40, 34.0 }// centered on starting line
    };

    double[][] shooterSpeed = {
            { 15, 10500 },
            { 40, 9500 }     
    };
    
    LinearInterpolation wrist;
    LinearInterpolation shooterRPM;

    public LimelightAutoCmd(ArmSubsystem arm, ShooterSubsystem shooter, LimelightSubsystem limelight, Telemetry logger,
            CommandSwerveDrivetrain drivetrain) {
        this.arm = arm;
        this.shooter = shooter;
        this.limelight = limelight;
        this.logger = logger;
        this.drivetrain = drivetrain;
        addRequirements(arm, shooter);
    }

    @Override
    public void initialize() {
        wrist = new LinearInterpolation(wristPosition);
        shooterRPM = new LinearInterpolation(shooterSpeed);
        limelight.setPipeline(0);
        shooterTimer = new Timer();
        isDone = false;
        drivetrain.registerTelemetry(logger::telemeterize);

        // arm.safeManualLimelightSetPosition(0, wrist.interpolate(tag.ty), 0, true);
    }

    @Override
    public void execute() {
        if (limelight.tagTv) {
            //limelight.limelightRotation = true;
            // System.out.println("limelight rotation on");
            double x = wrist.interpolate(limelight.tagTy);
            x = x + -1.5 * logger.getVelocityX();
            // SmartDashboard.putNumber("vel x", logger.getVelocityX());
            // SmartDashboard.putNumber("Calculated Wrist Position:",
            // wrist.interpolate(tag.ty));
            arm.safeManualLimelightSetPosition(0, x, 0, false);
            shooter.shooterV = shooterRPM.interpolate(limelight.tagTy);
            shooter.shooterState = ShooterState.SpinLimelight;

            // System.out.println(shooter.isShooterAtVelocity());
            if (ExtraMath.within(limelight.tagTx, 0, 8) && shooterTimer.get() == 0 /*&& shooter.isShooterAtVelocity()*/) {
                //limelight.limelightRotation = false;
                System.out.println("Limelight rotation off");
                shooterTimer.restart();
            }
            if (shooterTimer.get() > 0.4 && shooterTimer.get() < 0.7) {
                shooter.intakeState = IntakeState.ShootNow;
            }
            if (shooterTimer.get() > 0.7) {
                isDone = true;
            }
            // SmartDashboard.putNumber("Tag X", tag.tx);
        }
    }

    @Override
    public void end(boolean interrupted) {
        limelight.limelightRotation = false;
        shooter.shooterState = ShooterState.Idle;
        shooter.spinDownShooters();
        arm.isTrapezoidal = true;
        arm.unsafeSetPosition(ArmPosition.Stowed);

        shooter.intakeState = IntakeState.Idle;
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }
}