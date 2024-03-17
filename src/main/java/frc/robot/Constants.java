package frc.robot;

import frc.robot.subsystems.DashboardSubsystem.DashState;

public final class Constants {
    private Constants() {}

    // The port for discerning between the practice and competition bot
    public static final int BOT_DISCRIMINATION_PORT = 0;

    // SmartDashboard State
    public static final DashState DASH_STATE = DashState.Temp;

    // Climber motor IDs
    public static final int CLIMBER_LEFT_ID = 30;
    public static final int CLIMBER_RIGHT_ID = 52;

    // Floor intake motor ID
    public static final int FLOOR_INTAKE_ID = 32;

    // Pigeon ID
    public static final int PIGEON_ID = 18;

    // Shooter motor IDs + allowed offset
    public static final int SHOOTER_TOP_ID = 46;
    public static final int SHOOTER_BOTTOM_ID = 47;
    public static final int INTAKE_TOP_ID = 44;
    public static final int INTAKE_BOTTOM_ID = 45;
    public static final double SHOOTER_ALLOWED_X_OFFSET = 4;
    
    // Shoulder motor IDs and max
    public static final int SHOULDER_LEFT_MOTOR_ID = 40;
    public static final int SHOULDER_RIGHT_MOTOR_ID = 41;
    public static final double SHOULDER_ENCODER_MAX = -119;
    
    // Wrist motor ID and max
    public static final int WRIST_MOTOR_ID = 43;
    public static final double WRIST_ENCODER_MAX = 150;

    // Elevator motor ID and max
    public static final int ELEVATOR_MOTOR_ID = 42;
    public static final double ELEVATOR_ENCODER_MAX = -92;
    
    // Arm Positions Constants (in encoder revolutions)

    // Max Vs
    public static final double WRIST_DEFAULT_MAXV = 150;
    public static final double ELEVATOR_DEFAULT_MAXV = 120;

    // Stowed
    public static final double SHOULDER_STOWED_POSITION = 0;
    public static final double WRIST_STOWED_POSITION = 0;
    public static final double WRIST_STOWED_MAXV = WRIST_DEFAULT_MAXV;
    public static final double ELEVATOR_STOWED_POSITION = 0;
    public static final double ELEVATOR_STOWED_MAXV = ELEVATOR_DEFAULT_MAXV;

    // Intake
    public static final double SHOULDER_INTAKE_POSITION = 0;
    public static final double WRIST_INTAKE_POSITION = 64;
    public static final double WRIST_INTAKE_MAXV = WRIST_DEFAULT_MAXV;
    public static final double ELEVATOR_INTAKE_POSITION = 0;
    public static final double ELEVATOR_INTAKE_MAXV = ELEVATOR_DEFAULT_MAXV;
    
    // Popcorn
    public static final double SHOULDER_POPCORN_POSITION = 0;
    public static final double WRIST_POPCORN_POSITION = 50;
    public static final double WRIST_POPCORN_MAXV = WRIST_DEFAULT_MAXV;
    public static final double ELEVATOR_POPCORN_POSITION = 0;
    public static final double ELEVATOR_POPCORN_MAXV = ELEVATOR_DEFAULT_MAXV;
    public static final double POPCORN_SPEED = 6000;

    // Source
    public static final double SHOULDER_SOURCE_POSITION = -100;
    public static final double WRIST_SOURCE_POSITION = 70;
    public static final double WRIST_SOURCE_MAXV = WRIST_DEFAULT_MAXV;
    public static final double ELEVATOR_SOURCE_POSITION = 0;
    public static final double ELEVATOR_SOURCE_MAXV = ELEVATOR_DEFAULT_MAXV;
    public static final double SOURCE_INTAKE_SPEED = 5000;

    // Source Shot
    public static final double SHOULDER_SOURCE_SHOT_POSITION = -100;
    public static final double WRIST_SOURCE_SHOT_POSITION = 150;
    public static final double WRIST_SOURCE_SHOT_MAXV = WRIST_DEFAULT_MAXV;
    public static final double ELEVATOR_SOURCE_SHOT_POSITION = 0;
    public static final double ELEVATOR_SOURCE_SHOT_MAXV = ELEVATOR_DEFAULT_MAXV;
    public static final double SOURCE_SHOT_SHOOTER_SPEED = 9000;


    // Speaker High - Limelight
    public static final double SHOULDER_SPEAKER_HIGH_POSITION = 0;
    public static final double WRIST_SPEAKER_HIGH_POSITION = 0;
    public static final double WRIST_SPEAKER_HIGH_MAXV = WRIST_DEFAULT_MAXV;
    public static final double ELEVATOR_SPEAKER_HIGH_POSITION = 0;
    public static final double ELEVATOR_SPEAKER_HIGH_MAXV = ELEVATOR_DEFAULT_MAXV;


    // Speaker Low - Limelight
    public static final double SHOULDER_SPEAKER_LOW_POSITION = 0;
    public static final double WRIST_SPEAKER_LOW_POSITION = 0;
    public static final double WRIST_SPEAKER_LOW_MAXV = WRIST_DEFAULT_MAXV;
    public static final double ELEVATOR_SPEAKER_LOW_POSITION = 0;
    public static final double ELEVATOR_SPEAKER_LOW_MAXV = ELEVATOR_DEFAULT_MAXV;


    // Amp - Shooting Upwards
    // public static final double SHOULDER_AMP_POSITION = -50;
    // public static final double WRIST_AMP_POSITION = 115;
    // public static final double WRIST_AMP_MAXV = WRIST_DEFAULT_MAXV;
    // public static final double ELEVATOR_AMP_POSITION = -13;
    // public static final double ELEVATOR_AMP_MAXV = ELEVATOR_DEFAULT_MAXV;
    // public static final double AMP_SHOOT_SPEED = 5500;

    // Amp - Shooting Down
    public static final double SHOULDER_AMP_POSITION = SHOULDER_ENCODER_MAX;
    public static final double WRIST_AMP_POSITION = 25;
    public static final double WRIST_AMP_MAXV = WRIST_DEFAULT_MAXV;
    public static final double ELEVATOR_AMP_POSITION = ELEVATOR_ENCODER_MAX;
    public static final double ELEVATOR_AMP_MAXV = ELEVATOR_DEFAULT_MAXV;
    public static final double AMP_SHOOT_SPEED = 5500;

    // Amp Partial Stow
    public static final double SHOULDER_AMP_STOW_POSITION = -70;
    public static final double WRIST_AMP_STOW_POSITION = 25;
    public static final double WRIST_AMP_STOW_MAXV = WRIST_DEFAULT_MAXV;
    public static final double ELEVATOR_AMP_STOW_POSITION = ELEVATOR_ENCODER_MAX;
    public static final double ELEVATOR_AMP_STOW_MAXV = ELEVATOR_DEFAULT_MAXV;
    public static final double AMP_STOW_SHOOT_SPEED = 5500;

    // Amp2 - Shooting out the back
    public static final double SHOULDER_AMP_DOWN_POSITION = -82;
    public static final double WRIST_AMP_DOWN_POSITION = 60;
    public static final double WRIST_AMP_DOWN_MAXV = WRIST_DEFAULT_MAXV;
    public static final double ELEVATOR_AMP_DOWN_POSITION = -92;
    public static final double ELEVATOR_AMP_DOWN_MAXV = 60;


    // Trap
    public static final double TRAP_SHOOT_SPEED = 6500;


    // Subwoofer
    public static final double SHOULDER_SUBWOOFER_POSITION = 0;
    public static final double WRIST_SUBWOOFER_POSITION = 41.5;
    public static final double WRIST_SUBWOOFER_MAXV = WRIST_DEFAULT_MAXV;
    public static final double ELEVATOR_SUBWOOFER_POSITION = 0;
    public static final double ELEVATOR_SUBWOOFER_MAXV = ELEVATOR_DEFAULT_MAXV;
    public static final double SUBWOOFER_SHOOT_SPEED = 8500;

    
    // Podium High
    public static final double SHOULDER_PODIUM_HIGH_POSITION = -114;
    public static final double WRIST_PODIUM_HIGH_POSITION = 78.5;
    public static final double WRIST_PODIUM_HIGH_MAXV = WRIST_DEFAULT_MAXV;
    public static final double ELEVATOR_PODIUM_HIGH_POSITION = -60;
    public static final double ELEVATOR_PODIUM_HIGH_MAXV = ELEVATOR_DEFAULT_MAXV;
    public static final double PODIUM_HIGH_SPEED = 9000;

    // Podium Low
    public static final double SHOULDER_PODIUM_LOW_POSITION = 0;
    public static final double WRIST_PODIUM_LOW_POSITION = 30;
    public static final double WRIST_PODIUM_LOW_MAXV = WRIST_DEFAULT_MAXV;
    public static final double ELEVATOR_PODIUM_LOW_POSITION = 0;
    public static final double ELEVATOR_PODIUM_LOW_MAXV = ELEVATOR_DEFAULT_MAXV;
    public static final double PODIUM_LOW_SPEED = 9000;

    // Climber High
    public static final double SHOULDER_CLIMBER_HIGH_POSITION = -119;
    public static final double WRIST_CLIMBER_HIGH_POSITION = 0;
    public static final double WRIST_CLIMBER_HIGH_MAXV = WRIST_DEFAULT_MAXV;
    public static final double ELEVATOR_CLIMBER_HIGH_POSITION = 0;
    public static final double ELEVATOR_CLIMBER_HIGH_MAXV = ELEVATOR_DEFAULT_MAXV;
    
    // Climber Mid
    public static final double SHOULDER_CLIMBER_MID_POSITION = -114;
    public static final double WRIST_CLIMBER_MID_POSITION = 145;
    public static final double WRIST_CLIMBER_MID_MAXV = 75;
    public static final double ELEVATOR_CLIMBER_MID_POSITION = 0;
    public static final double ELEVATOR_CLIMBER_MID_MAXV = ELEVATOR_DEFAULT_MAXV;

    // Climber Low
    public static final double SHOULDER_CLIMBER_LOW_POSITION = -114;
    public static final double WRIST_CLIMBER_LOW_POSITION = 151;
    public static final double WRIST_CLIMBER_LOW_MAXV = 75;
    public static final double ELEVATOR_CLIMBER_LOW_POSITION = -92;
    public static final double ELEVATOR_CLIMBER_LOW_MAXV = ELEVATOR_DEFAULT_MAXV;

    // Climber Stowed
    public static final double SHOULDER_CLIMBER_STOWED_POSITION = 0;
    public static final double WRIST_CLIMBER_STOWED_POSITION = 0;
    public static final double WRIST_CLIMBER_STOWED_MAXV = WRIST_DEFAULT_MAXV;
    public static final double ELEVATOR_CLIMBER_STOWED_POSITION = 0;
    public static final double ELEVATOR_CLIMBER_STOWED_MAXV = ELEVATOR_DEFAULT_MAXV;

    // Climber Compact
    public static final double SHOULDER_CLIMBER_COMPACT_POSITION = -114;
    public static final double WRIST_CLIMBER_COMPACT_POSITION = 0;
    public static final double WRIST_CLIMBER_COMPACT_MAXV = WRIST_DEFAULT_MAXV;
    public static final double ELEVATOR_CLIMBER_COMPACT_POSITION = -92;
    public static final double ELEVATOR_CLIMBER_COMPACT_MAXV = ELEVATOR_DEFAULT_MAXV;

    // Climber Heights
    public static final double CLIMBER_HIGH_HEIGHT = 400;
    public static final double CLIMBER_MID_HEIGHT = 110;
    public static final double CLIMBER_LOW_HEIGHT = -5;
    public static final double CLIMBER_STOWED_HEIGHT = 0;

    // Limelight Shooter Variable Height RPMs
    public static final double SHOOTER_LOW_RPM = 10000;
    public static final double SHOOTER_MID_RPM = 9000;
    public static final double SHOOTER_HIGH_RPM = 8000;

    // Power Distribution Panel ID
    public static final int PDP_ID = 1;

    // Xbox Controller Ports
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int CODRIVER_CONTROLLER_PORT = 1;

    // LED Strip IDs
    public static final int LED_STRIP_ID = 0;
}
