package frc.robot.drive;

import java.sql.Driver;
import java.util.Optional;
import java.util.function.Supplier;

import javax.management.ImmutableDescriptor;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    public boolean brake = false;
    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();
    public double currentAngleTarget = 0;

    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    private boolean hasAppliedOperatorPerspective = false;

    public LimelightSubsystem limelight = null;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
        configurePathPlanner();

        if (Utils.isSimulation()) {
            startSimThread();
        }

    }

    public Optional<Rotation2d> getRotationTargetOverride() {
        // Some condition that should decide if we want to override rotation
        if (limelight.limelightRotation) {
            // Return an optional containing the rotation override (this should be a field
            // relative rotation)
            Rotation2d targetRelativeToRobot = Rotation2d.fromDegrees(-limelight.tagTx);
            Rotation2d currentRotation = getState().Pose.getRotation();
            Rotation2d absolute = new Rotation2d(targetRelativeToRobot.getRadians() + currentRotation.getRadians());
            // SmartDashboard.putNumber("Current Rotation", currentRotation.getDegrees());
            // SmartDashboard.putNumber("Absolute Rotation", absolute.getDegrees());
            return Optional.of(absolute);
        } else {
            // return an empty optional when we don't want to override the path's rotation
            return Optional.empty();
        }

    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
                new HolonomicPathFollowerConfig(new PIDConstants(5, 0, 0),
                        new PIDConstants(10, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                ()-> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, // Change this if the path needs to be flipped on red vs blue // blue is false
                this); // Subsystem for requirements
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic(){
        if(!hasAppliedOperatorPerspective || DriverStation.isDisabled()){
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation : BlueAlliancePerspectiveRotation);
                    hasAppliedOperatorPerspective = true;
            });
        }
    }

    private boolean getRobotTeamState(){
        // SmartDashboard.putData("", DriverStation.getAlliance());
        switch (DriverStation.getRawAllianceStation()) {
            case Red1:
            case Red2:
            case Red3:
                return true;
            default:
                return false;
                
        }
    }
}
