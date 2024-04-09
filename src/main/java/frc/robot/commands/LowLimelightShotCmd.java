package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.RobotContainer;
import frc.robot.drive.Telemetry;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class LowLimelightShotCmd extends Command {
    ArmSubsystem arm;
    double targetShoulderPosition;
    double targetWristPosition;
    double elevatorPosition;
    LimelightSubsystem limelight;
    ShooterSubsystem shooter;
    Telemetry logger;

    boolean shoulderSetCheck = false;
    boolean wristSetCheck = false;
    boolean elevatorSetCheck;

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
    Timer sniperTimer;

    public LowLimelightShotCmd(ArmSubsystem arm, ShooterSubsystem shooter, LimelightSubsystem limelight, Telemetry logger) {
        this.arm = arm;
        this.shooter = shooter;
        this.limelight = limelight;
        this.logger = logger;
        addRequirements(arm, shooter);

        wrist = new LinearInterpolation(wristPosition);
        shooterRPM = new LinearInterpolation(shooterSpeed);
        sniperTimer = new Timer();
    }

    @Override
    public void initialize() {
        limelight.setPipeline(0);
        shooter.okToShoot = false;
        sniperTimer.restart();
        
        // arm.safeManualLimelightSetPosition(0, wrist.interpolate(tag.ty), 0, true);
    }

    @Override
    public void execute() {
        limelight.limelightRotation = limelight.tagTv;
        if(limelight.limelightRotation){
            var speedMultiplier = ExtraMath.rangeMap(sniperTimer.get(), 0.25, 0.75, 1, 0.35);
            speedMultiplier = ExtraMath.clamp(speedMultiplier, 0.35, 1);
            RobotContainer.speedMultiplier = speedMultiplier;
            double x = wrist.interpolate(limelight.tagTy);
            x = x + -2*logger.getVelocityX();
            // SmartDashboard.putNumber("vel y", logger.getVelocityY());
            // SmartDashboard.putNumber("Calculated Wrist Position:", wrist.interpolate(tag.ty));
            arm.safeManualLimelightSetPosition(0, x, 0, false);
            shooter.shooterV = shooterRPM.interpolate(limelight.tagTy);
            shooter.shooterState = ShooterState.SpinLimelight;
            if(ExtraMath.within(limelight.tagTx, 10*logger.getVelocityY(), Constants.SHOOTER_ALLOWED_X_OFFSET)){
                // limelight.limelightRotation = false;
                shooter.okToShoot = true;
                limelight.readyToShoot = true;
                limelight.isAiming = false;
            } else {
                shooter.okToShoot = false;
                limelight.readyToShoot = false;
                limelight.isAiming = true;

            }
            // SmartDashboard.putNumber("Tag X", tag.tx);
        } else {
            arm.unsafeSetPosition(ArmPosition.SubWoofer);
            shooter.shooterV = Constants.SUBWOOFER_SHOOT_SPEED;
            shooter.shooterState = ShooterState.SpinFixed;
            shooter.okToShoot = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.speedMultiplier = 1;
        limelight.limelightRotation = false;
        shooter.okToShoot = true;
        shooter.shooterState = ShooterState.Idle;
        shooter.spinDownShooters();
        arm.isTrapezoidal = true;
        arm.unsafeSetPosition(ArmPosition.Stowed);
        limelight.isAiming = false;
        limelight.readyToShoot = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}