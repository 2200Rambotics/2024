package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AmpStowCmd;
import frc.robot.commands.ClimberPositionCmd;
import frc.robot.commands.CycleLEDModeCmd;
import frc.robot.commands.FloorToShooterCmd;
import frc.robot.commands.PreloadCmd;
import frc.robot.commands.FloorIntakeCmd;
import frc.robot.commands.LowLimelightShotCmd;
import frc.robot.commands.PopcornAutoCmd;
import frc.robot.commands.SetArmPositionCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.SignalNoteCmd;
import frc.robot.commands.SourceShotCmd;
import frc.robot.commands.SubwooferAutoCmd;
import frc.robot.commands.ZeroArmCommand;
import frc.robot.commands.SpinUpShooterCmd;
import frc.robot.commands.IntakeFromSourceCmd;
import frc.robot.commands.FadeawayAutoCmd;
import frc.robot.commands.FadeawayCmd;
import frc.robot.commands.LimelightAutoCmd;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.drive.Telemetry;
import frc.robot.drive.TunerConstants;
import frc.robot.input.AnalogTrigger;
import frc.robot.input.DPadButton;
import frc.robot.input.Keybind;
import frc.robot.input.AnalogTrigger.Axis;
import frc.robot.input.DPadButton.DPad;
import frc.robot.input.Keybind.Button;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DashboardSubsystem;
import frc.robot.subsystems.DigitalIOSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PigeonSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.ClimberSubsystem.ClimbState;
import frc.robot.subsystems.FloorIntakeSubsystem.FloorIntakeState;
import frc.robot.subsystems.ArmSubsystem;

public class RobotContainer {
    // The following is swerve auto-generated code
    private final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    public static double MaxAngularRate = 1.5 * Math.PI; // 1.0 of a rotation per second max angular velocity

    // set up the pigeon
    public PigeonSubsystem pGryo = new PigeonSubsystem();

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

    // Swerve Field Centric
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1 * speedMultiplier).withRotationalDeadband(MaxAngularRate * 0.)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Swerve Field Centric Facing Angle
    private final SwerveRequest.FieldCentricFacingAngle look = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1 * speedMultiplier)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Swerve Brake
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake(); // Brake Mode

    // Swerve Point
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt(); // Wheel Point Mode

    // Set up Telemetry
    final Telemetry logger = new Telemetry(MaxSpeed);

    // Set up LimeLight
    public LimelightSubsystem backLimelight;

    // Set up Subsystems
    public ClimberSubsystem climber;
    public PowerDistribution pdp;
    public FloorIntakeSubsystem floorIntake;
    public ShooterSubsystem shooter;
    public ArmSubsystem arm;
    public DigitalIOSubsystem digitalio;
    public LEDSubsystem led;

    public boolean savedAllianceRed;
    boolean shouldStayDegree;
    Rotation2d stayDegree;

    public DashboardSubsystem dashboard;
    public final CommandXboxController driverController = new CommandXboxController(
            Constants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController codriverController = new CommandXboxController(
            Constants.CODRIVER_CONTROLLER_PORT);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        pdp = new PowerDistribution(Constants.PDP_ID, ModuleType.kCTRE);
        backLimelight = new LimelightSubsystem("limelight-back");
        arm = new ArmSubsystem();
        floorIntake = new FloorIntakeSubsystem();
        shooter = new ShooterSubsystem();
        climber = new ClimberSubsystem();
        digitalio = new DigitalIOSubsystem(arm, shooter, floorIntake, climber);
        dashboard = new DashboardSubsystem(arm, shooter, climber, floorIntake);
        led = new LEDSubsystem(backLimelight, shooter, pdp, arm);

        drivetrain.limelight = backLimelight;
        shouldStayDegree = false;
        stayDegree = new Rotation2d(0);
        
        look.HeadingController = new PhoenixPIDController(3.7, 0.00, 0.);
        look.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        NamedCommands.registerCommand("limelight",
                new LimelightAutoCmd(arm, shooter, backLimelight, logger, drivetrain));
        NamedCommands.registerCommand("subwoofer", new SubwooferAutoCmd(arm, shooter));
        NamedCommands.registerCommand("intake", new FloorToShooterCmd(floorIntake, shooter, arm, true));
        NamedCommands.registerCommand("preload", new PreloadCmd(shooter, arm));
        NamedCommands.registerCommand("popcorn",
                new PopcornAutoCmd(floorIntake, shooter, arm, Constants.POPCORN_SPEED));
        NamedCommands.registerCommand("fadeaway",
                new FadeawayAutoCmd(floorIntake, shooter, arm,
                        backLimelight, logger));

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /** Right Trigger */
    AnalogTrigger intakeDriverKeybind;

    /** Left Bumper */
    Keybind resetFieldCentricKeybind;

    /** Dpad Up */
    Keybind snapTo0Keybind;
    /** Dpad Left */
    Keybind snapTo90Keybind;
    /** Dpad Down */
    Keybind snapTo180Keybind;
    /** Dpad Right */
    Keybind snapTo270Keybind;

    /** Right Dpad */
    DPadButton cycleLEDModeKeybind;

    /** B Button */
    Keybind snapToNoteKeybind;
    double angleFromNote = 0;
    Keybind zeroArmKeybind;

    /** Left Trigger */
    AnalogTrigger signalNote;

    // codriver keybinds
    /** Dpad Up */
    DPadButton climberMaxKeybind;
    /** Dpad Down */
    DPadButton climberMinKeybind;
    /** Dpad Left */
    DPadButton climberMidKeybind;
    /** Dpad Right */
    DPadButton climberStowKeybind;
    /** Back Button */
    Keybind climberCompactKeybind;

    /** Amp & Amp2 - X */
    Keybind ampKeybind;

    /** Intake & Source - A */
    Keybind intakeKeybind;

    /** Speaker High & Low, Podium High & Low - Y */
    Keybind shootPositionKeybind;

    /** B Button */
    Keybind subwooferKeybind;

    /** Right Trigger */
    AnalogTrigger shootTrigger;

    /** Left Trigger */
    AnalogTrigger spitTrigger;

    // Modifiers
    /** Left Bumper */
    Trigger modifyArm;
    /** Right Bumper */
    Trigger fixedArm;

    Keybind dubiousSpit;

    Keybind spinUpTrap;

    AnalogTrigger secretShoot;
    Keybind secretAim;

    public static double speedMultiplier = 1;

    private void configureBindings() {
        modifyArm = codriverController.leftBumper();
        fixedArm = codriverController.rightBumper();

        // Driver Controller - Keybind Initialization
        snapTo0Keybind = new Keybind(driverController.getHID(), Button.Y);
        snapTo90Keybind = new Keybind(driverController.getHID(), Button.X);
        snapTo180Keybind = new Keybind(driverController.getHID(), Button.A);
        snapTo270Keybind = new Keybind(driverController.getHID(), Button.B);

        cycleLEDModeKeybind = new DPadButton(driverController.getHID(), DPad.Right);

        zeroArmKeybind = new Keybind(driverController.getHID(), Button.Start);
        signalNote = new AnalogTrigger(driverController.getHID(), Axis.LT, 0.5);

        // snapToNoteKeybind = new Keybind(driverController.getHID(),
        // Button.RightBumper);

        resetFieldCentricKeybind = new Keybind(driverController.getHID(), Button.LeftBumper);
        intakeDriverKeybind = new AnalogTrigger(driverController.getHID(), Axis.RT, 0.5);
        secretShoot = new AnalogTrigger(driverController.getHID(), Axis.LT, 0.5);
        // secretAim = new Keybind(driverController.getHID(), Button.RightBumper);
        dubiousSpit = new Keybind(driverController.getHID(), Button.RightBumper);

        // Codriver Controller - Keybind Initialization
        climberMaxKeybind = new DPadButton(codriverController.getHID(), DPad.Up);
        climberMinKeybind = new DPadButton(codriverController.getHID(), DPad.Down);
        climberMidKeybind = new DPadButton(codriverController.getHID(), DPad.Left);
        climberStowKeybind = new DPadButton(codriverController.getHID(), DPad.Right);
        climberCompactKeybind = new Keybind(codriverController.getHID(), Button.Back);
        ampKeybind = new Keybind(codriverController.getHID(), Button.X);
        intakeKeybind = new Keybind(codriverController.getHID(), Button.A);
        shootPositionKeybind = new Keybind(codriverController.getHID(), Button.Y);
        subwooferKeybind = new Keybind(codriverController.getHID(), Button.B);
        shootTrigger = new AnalogTrigger(codriverController.getHID(), Axis.RT, 0.5);
        spitTrigger = new AnalogTrigger(codriverController.getHID(), Axis.LT, 0.5);
        spinUpTrap = new Keybind(codriverController.getHID(), Button.Start);
        // bind driver controls to commands
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> {

                    if (DriverStation.isTeleopEnabled()) {
                        double rate;

                        SmartDashboard.putNumber("Stay Degree", stayDegree.getDegrees());
                        SmartDashboard.putBoolean("Should stay", shouldStayDegree);
                        

                        if (backLimelight.limelightRotation && backLimelight.tagTv) {
                            rate = -0.026 * MaxAngularRate
                                    * (backLimelight.tagTx + (-10 * logger.getVelocityY()));
                            shouldStayDegree = false;
                        } else {

                            if (ExtraMath.within(driverController.getRightX(), 0, 0.1)
                                    && !shouldStayDegree) {
                                shouldStayDegree = true;
                                stayDegree = Rotation2d.fromDegrees(pGryo.Y).rotateBy(allianceBasedRotation());

                            } else if (!ExtraMath.within(driverController.getRightX(), 0, 0.1)) {
                                shouldStayDegree = false;
                            }

                            rate = ExtraMath.deadzone(-driverController.getRightX() * MaxAngularRate, 0.1);
                        }
                        if (shouldStayDegree) {

                            return look
                                    .withVelocityX(driverGetX())
                                    .withVelocityY(driverGetY())
                                    .withTargetDirection(stayDegree);
                        } else {
                            return drive
                                    .withVelocityX(driverGetX())
                                    .withVelocityY(driverGetY())
                                    .withRotationalRate(rate);
                        }
                    }else{
                        return drive
                                    .withVelocityX(0)
                                    .withVelocityY(0)
                                    .withRotationalRate(0);
                    }

                }));

        driverController.back().and(driverController.start()).and(driverController.a()).whileTrue(
                drivetrain.applyRequest(() -> {
                    return point.withModuleDirection(rotation_0degree);
                }));

        snapTo0Keybind.trigger().whileTrue(drivetrain.applyRequest(() -> {
            stayDegree = rotation_0degree;
            return look
                    .withTargetDirection(rotation_0degree)
                    .withVelocityX(driverGetX())
                    .withVelocityY(driverGetY());
        }));
        snapTo90Keybind.trigger().whileTrue(drivetrain.applyRequest(() -> {
            stayDegree = rotation_90degree;
            return look
                    .withTargetDirection(rotation_90degree)
                    .withVelocityX(driverGetX())
                    .withVelocityY(driverGetY());
        }));
        snapTo180Keybind.trigger().whileTrue(drivetrain.applyRequest(() -> {
            stayDegree = rotation_180degree;
            return look
                    .withTargetDirection(rotation_180degree)
                    .withVelocityX(driverGetX())
                    .withVelocityY(driverGetY());
        }));
        snapTo270Keybind.trigger().whileTrue(drivetrain.applyRequest(() -> {
            stayDegree = rotation_neg90degree;
            return look
                    .withTargetDirection(rotation_neg90degree)
                    .withVelocityX(driverGetX())
                    .withVelocityY(driverGetY());
        }));

        driverController.back().whileTrue(drivetrain.applyRequest(() -> brake));

        // reset the field-centric heading on left bumper press
        resetFieldCentricKeybind.trigger().onTrue(drivetrain.runOnce(() -> {
            // Pose2d tempPose = new Pose2d(0, 0, allianceBasedRotation());
            // drivetrain.seedFieldRelative(tempPose);
            pGryo.zeroYaw(savedAllianceRed);

            // angleOffset = Rotation2d.fromDegrees(pGryo.Y);
            stayDegree = rotation_0degree;

        }));

        drivetrain.registerTelemetry(logger::telemeterize);

        dubiousSpit.trigger().whileTrue(new FloorIntakeCmd(floorIntake, FloorIntakeState.Spit, 0));
        cycleLEDModeKeybind.trigger().onTrue(new CycleLEDModeCmd(led));

        intakeDriverKeybind.trigger().whileTrue(new FloorToShooterCmd(floorIntake, shooter, arm, true));
        intakeDriverKeybind.trigger().onFalse(new PreloadCmd(shooter, arm));
        intakeDriverKeybind.trigger().onFalse(new FloorIntakeCmd(floorIntake, FloorIntakeState.Stop, 1));

        zeroArmKeybind.trigger().whileTrue(new ZeroArmCommand(arm));
        signalNote.trigger().whileTrue(new SignalNoteCmd(led));

        // bind codriver controls to commands
        subwooferKeybind.trigger().whileTrue(new SetArmPositionCmd(arm, ArmPosition.SubWoofer));
        subwooferKeybind.trigger()
                .whileTrue(new SpinUpShooterCmd(shooter, Constants.SUBWOOFER_SHOOT_SPEED, false));

        ampKeybind.trigger().and(modifyArm).whileTrue(new SetArmPositionCmd(arm, ArmPosition.Amp2));
        ampKeybind.trigger().and(modifyArm.negate()).whileTrue(new SetArmPositionCmd(arm, ArmPosition.Amp));
        ampKeybind.trigger().and(modifyArm.negate())
                .whileTrue(new SpinUpShooterCmd(shooter, Constants.AMP_SHOOT_SPEED, false));
        ampKeybind.trigger().and(modifyArm.negate()).onFalse(new AmpStowCmd(arm));

        intakeKeybind.trigger().and(modifyArm.negate()).and(fixedArm.negate()).whileTrue(new FloorToShooterCmd(floorIntake, shooter, arm, true));
        intakeKeybind.trigger().and(modifyArm).and(fixedArm.negate()).whileTrue(new IntakeFromSourceCmd(arm, shooter, Constants.SOURCE_INTAKE_SPEED));
        intakeKeybind.trigger().and(modifyArm).and(fixedArm).whileTrue(new SourceShotCmd(arm, shooter, Constants.SOURCE_SHOT_SHOOTER_SPEED));
        intakeKeybind.trigger().and(modifyArm.negate()).and(fixedArm).whileTrue(new FadeawayCmd(floorIntake, shooter, arm, backLimelight, logger));
        intakeKeybind.trigger().onFalse(new PreloadCmd(shooter, arm));
        intakeKeybind.trigger().onFalse(new FloorIntakeCmd(floorIntake, FloorIntakeState.Stop, 0));

        // y button! (main speaker shot)
        shootPositionKeybind.trigger().and(modifyArm.negate()).and(fixedArm.negate())
                .whileTrue(new LowLimelightShotCmd(arm, shooter, backLimelight, logger));
        // shootPositionKeybind.trigger().and(modifyArm).and(fixedArm.negate())
        // .whileTrue(new SetArmPositionCmd(arm, ArmPosition.SpeakerHigh));
        shootPositionKeybind.trigger().and(modifyArm.negate()).and(fixedArm)
                .whileTrue(new SetArmPositionCmd(arm, ArmPosition.PodiumLow));
        shootPositionKeybind.trigger().and(modifyArm.negate()).and(fixedArm)
                .whileTrue(new SpinUpShooterCmd(shooter, Constants.PODIUM_LOW_SPEED, false));
        shootPositionKeybind.trigger().and(modifyArm).and(fixedArm)
                .whileTrue(new SetArmPositionCmd(arm, ArmPosition.PodiumHigh));
        shootPositionKeybind.trigger().and(modifyArm).and(fixedArm)
                .whileTrue(new SpinUpShooterCmd(shooter, Constants.PODIUM_HIGH_SPEED, false));

        // non positional
        shootTrigger.trigger().whileTrue(new ShootCmd(arm, shooter, 0));
        shootTrigger.trigger().whileTrue(new FloorIntakeCmd(floorIntake, FloorIntakeState.Eat, 0));
        spitTrigger.trigger().and(modifyArm.negate()).and(fixedArm.negate())
                .whileTrue(new FloorIntakeCmd(floorIntake, FloorIntakeState.Spit, 0));
        spitTrigger.trigger().and(modifyArm).and(fixedArm.negate()).whileTrue(new ShootCmd(arm, shooter, 2));
        spitTrigger.trigger().and(modifyArm.negate()).and(fixedArm).whileTrue(new ShootCmd(arm, shooter, 1));

        climberMaxKeybind.trigger().onTrue(new ClimberPositionCmd(climber, arm, ClimbState.Max));
        climberMinKeybind.trigger().onTrue(new ClimberPositionCmd(climber, arm, ClimbState.Min));
        climberMidKeybind.trigger().onTrue(new ClimberPositionCmd(climber, arm, ClimbState.Mid));
        climberStowKeybind.trigger().onTrue(new ClimberPositionCmd(climber, arm, ClimbState.Stowed));
        climberCompactKeybind.trigger().onTrue(new ClimberPositionCmd(climber, arm, ClimbState.Compact));
        spinUpTrap.trigger().whileTrue(new SpinUpShooterCmd(shooter, Constants.TRAP_SHOOT_SPEED, true));

    }

    private double driverGetX() {
        int direction = -1;
        return direction * driverController.getLeftY() * MaxSpeed * speedMultiplier;
    }

    private double driverGetY() {
        int direction = -1;
        return direction * driverController.getLeftX() * MaxSpeed * speedMultiplier;
    }

    final Rotation2d rotation_0degree = Rotation2d.fromDegrees(0);
    final Rotation2d rotation_180degree = Rotation2d.fromDegrees(180);
    final Rotation2d rotation_90degree = Rotation2d.fromDegrees(90);
    final Rotation2d rotation_neg90degree = Rotation2d.fromDegrees(-90);

    public Rotation2d allianceBasedRotation() {
        savedAllianceRed=true;
        if (savedAllianceRed) {
            return rotation_180degree;
        }
        return rotation_0degree;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}