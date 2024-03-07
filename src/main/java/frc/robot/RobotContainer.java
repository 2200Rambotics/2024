package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimberLockCmd;
import frc.robot.commands.ClimberPositionCmd;
import frc.robot.commands.FloorToShooterCmd;
import frc.robot.commands.PreloadCmd;
import frc.robot.commands.FloorIntakeCmd;
import frc.robot.commands.LowLimelightShotCmd;
import frc.robot.commands.PopcornAutoCmd;
import frc.robot.commands.SetArmPositionCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.SubwooferAutoCmd;
import frc.robot.commands.ZeroArmCommand;
import frc.robot.commands.SpinUpShooterCmd;
import frc.robot.commands.IntakeFromSourceCmd;
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
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    public static double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                     // driving in open loop
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.FieldCentricFacingAngle look = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1)// .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                     // driving in open loop

    final Telemetry logger = new Telemetry(MaxSpeed);

    public PigeonSubsystem pigeon;
    public LEDSubsystem led;
    public LimelightSubsystem backLimelight;
    public LimelightSubsystem leftLimelight;
    public LimelightSubsystem rightLimelight;
    public ClimberSubsystem climber;
    public PowerDistribution pdp;
    public FloorIntakeSubsystem floorIntake;
    public ShooterSubsystem shooter;
    public ArmSubsystem arm;
    public DigitalIOSubsystem digitalio;

    boolean shouldStayDegree;
    Rotation2d stayDegree, angleOffset;
    public DashboardSubsystem dashboard;
    public final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController codriverController = new CommandXboxController(
            Constants.CODRIVER_CONTROLLER_PORT);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        pigeon = new PigeonSubsystem();
        // pdp = new PowerDistribution(Constants.PDP_ID, ModuleType.kCTRE);
        backLimelight = new LimelightSubsystem("limelight-back");
        leftLimelight = new LimelightSubsystem("limelight-left");
        rightLimelight = new LimelightSubsystem("limelight-right");
        led = new LEDSubsystem(pdp);
        drivetrain.limelight = backLimelight;

        shouldStayDegree = false;
        stayDegree = new Rotation2d(0);
        angleOffset = drivetrain.getState().Pose.getRotation();

        arm = new ArmSubsystem();
        floorIntake = new FloorIntakeSubsystem();
        shooter = new ShooterSubsystem();
        climber = new ClimberSubsystem();
        digitalio = new DigitalIOSubsystem(arm, shooter, floorIntake, climber);
        dashboard = new DashboardSubsystem(arm, shooter, climber, floorIntake);

        look.HeadingController = new PhoenixPIDController(3.7, 0.00, 0.);
        look.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        drivetrain.seedFieldRelative();

        NamedCommands.registerCommand("limelight",
                new LimelightAutoCmd(arm, shooter, backLimelight, logger, drivetrain, drive));
        NamedCommands.registerCommand("subwoofer", new SubwooferAutoCmd(arm, shooter));
        NamedCommands.registerCommand("intake", new FloorToShooterCmd(floorIntake, shooter, arm, true));
        NamedCommands.registerCommand("preload", new PreloadCmd(shooter, arm));
        NamedCommands.registerCommand("popcorn", new PopcornAutoCmd(floorIntake, shooter, arm, Constants.POPCORN_SPEED));

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();

        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

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

    /** B Button */
    Keybind snapToNoteKeybind;
    double angleFromNote = 0;
    Keybind zeroArmKeybind;

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

        zeroArmKeybind = new Keybind(driverController.getHID(), Button.Start);

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
                    double rate;

                    SmartDashboard.putNumber("Stay Degree", stayDegree.getDegrees());
                    SmartDashboard.putNumber("Logger Angle", drivetrain.getState().Pose.getRotation().getDegrees());
                    SmartDashboard.putNumber("Angle Offset", angleOffset.getDegrees());

                    if (backLimelight.limelightRotation && backLimelight.tagTv) {
                        rate = -0.026 * MaxAngularRate * (backLimelight.tagTx + (-10 * logger.getVelocityY()));
                        shouldStayDegree = false;
                    } else {

                        if (ExtraMath.within(driverController.getRightX(), 0, 0.1) && !shouldStayDegree) {
                            shouldStayDegree = true;
                            stayDegree = Rotation2d.fromDegrees(drivetrain.getState().Pose.getRotation().getDegrees() - angleOffset.getDegrees());

                        } else if (!ExtraMath.within(driverController.getRightX(), 0, 0.1)) {
                            shouldStayDegree = false;
                        }

                        rate = -driverController.getRightX() * MaxAngularRate;
                    }

                    if (shouldStayDegree) {

                        return look
                                .withVelocityX(-driverController.getLeftY() * MaxSpeed * speedMultiplier) 
                                .withVelocityY(-driverController.getLeftX() * MaxSpeed * speedMultiplier) 
                                .withTargetDirection(stayDegree);

                    } else{
                        return drive
                                .withVelocityX(-driverController.getLeftY() * MaxSpeed * speedMultiplier)
                                .withVelocityY(-driverController.getLeftX() * MaxSpeed * speedMultiplier) 
                                .withRotationalRate(rate);
                    }
                }));

        driverController.back().and(driverController.start()).and(driverController.a()).whileTrue(
                drivetrain.applyRequest(() -> {
                    return point.withModuleDirection(new Rotation2d(0));
                }));

        snapTo0Keybind.trigger().whileTrue(drivetrain.applyRequest(() -> {
            stayDegree = Rotation2d.fromDegrees(0);
            return look
                    .withTargetDirection(Rotation2d.fromDegrees(0))
                    .withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed); // Drive left with negative X (left)
        }));
        snapTo90Keybind.trigger().whileTrue(drivetrain.applyRequest(() -> {
            stayDegree = Rotation2d.fromDegrees(90);
            return look
                    .withTargetDirection(Rotation2d.fromDegrees(90))
                    .withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed); // Drive left with negative X (left)
        }));
        snapTo180Keybind.trigger().whileTrue(drivetrain.applyRequest(() -> {
            stayDegree = Rotation2d.fromDegrees(180);
            return look
                    .withTargetDirection(Rotation2d.fromDegrees(180))
                    .withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed); // Drive left with negative X (left)
        }));
        snapTo270Keybind.trigger().whileTrue(drivetrain.applyRequest(() -> {
            stayDegree = Rotation2d.fromDegrees(-90);
            return look
                    .withTargetDirection(Rotation2d.fromDegrees(-90))
                    .withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed); // Drive left with negative X (left)
        }));

        // snapToNoteKeybind.trigger().whileTrue(drivetrain.applyRequest(() -> {

        // if(ang)
        // return look
        // .withTargetDirection(Rotation2d.fromDegrees(angleFromNote))
        // .withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with
        // negative Y (forward)
        // .withVelocityY(-driverController.getLeftX() * MaxSpeed); // Drive left with
        // negative X (left)
        // }));

        // not sure how brake will be toggled...
        driverController.back().whileTrue(drivetrain.applyRequest(() -> brake));

        // reset the field-centric heading on left bumper press
        resetFieldCentricKeybind.trigger().onTrue(drivetrain.runOnce(() -> {
            drivetrain.seedFieldRelative();
            angleOffset = drivetrain.getState().Pose.getRotation();
            stayDegree = new Rotation2d(0);


        }));

        drivetrain.registerTelemetry(logger::telemeterize);

        // snapTo0Keybind.trigger().whileTrue(new SnapToDegreeCmd(pigeon, 0));
        // snapTo180Keybind.trigger().whileTrue(new SnapToDegreeCmd(pigeon, 180));
        // snapToAmpKeybind.trigger().whileTrue(
        // new SnapToDegreeCmd(pigeon, () ->
        // DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? 90 :
        // -90));
        // snapToSLKeybind.trigger().whileTrue(new SnapToDegreeCmd(pigeon, -45));
        // snapToSRKeybind.trigger().whileTrue(new SnapToDegreeCmd(pigeon, 45));
        // snapToNoteKeybind.trigger().whileTrue(new SnapToDegreeCmd(pigeon,
        // limelight1));

        // secretShoot.trigger().whileTrue(new ShootCmd(arm, shooter, true));
        // secretAim.trigger().whileTrue(new LowLimelightShotCmd(arm, shooter,
        // limelight1, logger));
        dubiousSpit.trigger().whileTrue(new FloorIntakeCmd(floorIntake, FloorIntakeState.Spit, 0));

        intakeDriverKeybind.trigger().whileTrue(new FloorToShooterCmd(floorIntake, shooter, arm, true));
        intakeDriverKeybind.trigger().onFalse(new PreloadCmd(shooter, arm));
        intakeDriverKeybind.trigger().onFalse(new FloorIntakeCmd(floorIntake, FloorIntakeState.Stop, 1));

        zeroArmKeybind.trigger().whileTrue(new ZeroArmCommand(arm));

        // bind codriver controls to commands
        subwooferKeybind.trigger().whileTrue(new SetArmPositionCmd(arm, ArmPosition.SubWoofer));
        subwooferKeybind.trigger().whileTrue(new SpinUpShooterCmd(shooter, Constants.SUBWOOFER_SHOOT_SPEED, false));

        ampKeybind.trigger().and(modifyArm).whileTrue(new SetArmPositionCmd(arm, ArmPosition.Amp2));
        ampKeybind.trigger().and(modifyArm.negate()).whileTrue(new SetArmPositionCmd(arm, ArmPosition.Amp));
        ampKeybind.trigger().and(modifyArm.negate())
                .whileTrue(new SpinUpShooterCmd(shooter, Constants.AMP_SHOOT_SPEED, true));

        intakeKeybind.trigger().and(modifyArm)
                .whileTrue(new IntakeFromSourceCmd(arm, shooter, Constants.SOURCE_INTAKE_SPEED));
        intakeKeybind.trigger().onFalse(new PreloadCmd(shooter, arm));
        intakeKeybind.trigger().onFalse(new FloorIntakeCmd(floorIntake, FloorIntakeState.Stop, 0));
        intakeKeybind.trigger().and(modifyArm.negate())
                .whileTrue(new FloorToShooterCmd(floorIntake, shooter, arm, true));

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
        shootTrigger.trigger().whileTrue(new ShootCmd(arm, shooter, true));
        spitTrigger.trigger().and(modifyArm.negate())
                .whileTrue(new FloorIntakeCmd(floorIntake, FloorIntakeState.Spit, 0));
        spitTrigger.trigger().and(modifyArm).whileTrue(new ShootCmd(arm, shooter, false));

        climberMaxKeybind.trigger().onTrue(new ClimberPositionCmd(climber, arm, ClimbState.Max));
        climberMinKeybind.trigger().onTrue(new ClimberPositionCmd(climber, arm, ClimbState.Min));
        climberMidKeybind.trigger().onTrue(new ClimberPositionCmd(climber, arm, ClimbState.Mid));
        climberStowKeybind.trigger().onTrue(new ClimberPositionCmd(climber, arm, ClimbState.Stowed));
        climberCompactKeybind.trigger().onTrue(new ClimberPositionCmd(climber, arm, ClimbState.Compact));
        spinUpTrap.trigger().whileTrue(new SpinUpShooterCmd(shooter, 4500, true));

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}