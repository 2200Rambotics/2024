package frc.robot.drive;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import edu.wpi.first.math.util.Units;

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
public class GoodTunerConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.2)
        .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(3).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 80.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 4.5;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5714285714285716;

    private static final double kDriveGearRatio = 6.746031746031747;
    private static final double kSteerGearRatio = 21.428571428571427;
    private static final double kWheelRadiusInches = 2;

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = "DriveTrain";
    private static final int kPigeonId = 18;


    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);
    static boolean isPractice = false;
    static boolean hasInitialized = false;
    public static void initCompetition() {
        if (hasInitialized) return;
        hasInitialized = true;

        kFrontLeftEncoderOffset = kCompetitionFrontLeftEncoderOffset;
        kFrontRightEncoderOffset = kCompetitionFrontRightEncoderOffset;
        kBackLeftEncoderOffset = kCompetitionBackLeftEncoderOffset;
        kBackRightEncoderOffset = kCompetitionBackRightEncoderOffset;
        isPractice = false;

        commonInit();
    }
    public static void initPractice() {
        if (hasInitialized) return;
        hasInitialized = true;

        kFrontLeftEncoderOffset = kPracticeFrontLeftEncoderOffset;
        kFrontRightEncoderOffset = kPracticeFrontRightEncoderOffset;
        kBackLeftEncoderOffset = kPracticeBackLeftEncoderOffset;
        kBackRightEncoderOffset = kPracticeBackRightEncoderOffset;
        isPractice = true;

        commonInit();
    }
    static void commonInit() {
        FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
        FrontRight = ConstantCreator.createModuleConstants(
                kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
        BackLeft = ConstantCreator.createModuleConstants(
                kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
        BackRight = ConstantCreator.createModuleConstants(
                kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

        DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants, FrontLeft,
            FrontRight, BackLeft, BackRight);
    }

    // Front Left
    private static final int kFrontLeftDriveMotorId = 10;
    private static final int kFrontLeftSteerMotorId = 11;
    private static final int kFrontLeftEncoderId = 19;
    private static final double kPracticeFrontLeftEncoderOffset = 0.4111328125;
    private static final double kCompetitionFrontLeftEncoderOffset = -0.399169921875;
    private static double kFrontLeftEncoderOffset = kCompetitionFrontLeftEncoderOffset;

    private static final double kFrontLeftXPosInches = 10.375;
    private static final double kFrontLeftYPosInches = 10.375;

    // Front Right
    private static final int kFrontRightDriveMotorId = 12;
    private static final int kFrontRightSteerMotorId = 13;
    private static final int kFrontRightEncoderId = 20;
    private static final double kPracticeFrontRightEncoderOffset = 0.340576171875;
    private static final double kCompetitionFrontRightEncoderOffset = 0.34912109375;
    private static double kFrontRightEncoderOffset = kCompetitionFrontRightEncoderOffset;

    private static final double kFrontRightXPosInches = 10.375;
    private static final double kFrontRightYPosInches = -10.375;

    // Back Left
    private static final int kBackLeftDriveMotorId = 14;
    private static final int kBackLeftSteerMotorId = 15;
    private static final int kBackLeftEncoderId = 21;
    private static final double kPracticeBackLeftEncoderOffset = 0.40673828125;
    private static final double kCompetitionBackLeftEncoderOffset = 0.414306640625;
    private static double kBackLeftEncoderOffset = kCompetitionBackLeftEncoderOffset;

    private static final double kBackLeftXPosInches = -10.375;
    private static final double kBackLeftYPosInches = 10.375;

    // Back Right
    private static final int kBackRightDriveMotorId = 16;
    private static final int kBackRightSteerMotorId = 17;
    private static final int kBackRightEncoderId = 22;
    private static final double kPracticeBackRightEncoderOffset = -0.0341796875;
    private static final double kCompetitionBackRightEncoderOffset = -0.001708984375;
    private static double kBackRightEncoderOffset = kCompetitionBackRightEncoderOffset;

    private static final double kBackRightXPosInches = -10.375;
    private static final double kBackRightYPosInches = -10.375;


    private static SwerveModuleConstants FrontLeft;
    private static SwerveModuleConstants FrontRight;
    private static SwerveModuleConstants BackLeft;
    private static SwerveModuleConstants BackRight;

    public static CommandSwerveDrivetrain DriveTrain;
}