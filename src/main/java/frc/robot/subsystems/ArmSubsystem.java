package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import frc.robot.Constants;
import frc.robot.PIDMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  public PIDMotor leftShoulderMotor;
  public PIDMotor rightShoulderMotor;
  public PIDMotor wristMotor;
  public PIDMotor elevatorMotor;
  
  public ArmPosition target = ArmPosition.Stowed;
  public boolean isTrapezoidal = true;
  
  public enum ArmPosition {
    Stowed, Intake, Source, SourceShot, SpeakerHigh, SpeakerLow, Amp, Amp2, AmpStow, Trap,
    SubWoofer, PodiumHigh, PodiumLow, ClimberUp, ClimberMid,
    ClimberLow, ClimberStowed, ClimberCompact, Popcorn;
    
    public double shoulderPosition() {
      switch (this) {
        case Stowed: return Constants.SHOULDER_STOWED_POSITION;
        case Intake: return Constants.SHOULDER_INTAKE_POSITION;
        case Source: return Constants.SHOULDER_SOURCE_POSITION;
        case SourceShot: return Constants.SHOULDER_SOURCE_SHOT_POSITION;
        case SpeakerHigh: return Constants.SHOULDER_SPEAKER_HIGH_POSITION;
        case SpeakerLow: return Constants.SHOULDER_SPEAKER_LOW_POSITION;
        case Amp: return Constants.SHOULDER_AMP_POSITION;
        case Amp2: return Constants.SHOULDER_AMP_DOWN_POSITION;
        case AmpStow: return Constants.SHOULDER_AMP_STOW_POSITION;
        case SubWoofer: return Constants.SHOULDER_SUBWOOFER_POSITION;
        case PodiumHigh: return Constants.SHOULDER_PODIUM_HIGH_POSITION;
        case PodiumLow: return Constants.SHOULDER_PODIUM_LOW_POSITION;
        case ClimberUp: return Constants.SHOULDER_CLIMBER_HIGH_POSITION;
        case ClimberMid: return Constants.SHOULDER_CLIMBER_MID_POSITION;
        case ClimberLow: return Constants.SHOULDER_CLIMBER_LOW_POSITION;
        case ClimberStowed: return Constants.SHOULDER_CLIMBER_STOWED_POSITION;
        case ClimberCompact: return Constants.SHOULDER_CLIMBER_COMPACT_POSITION;
        case Popcorn: return Constants.SHOULDER_POPCORN_POSITION;
        default: return Constants.SHOULDER_STOWED_POSITION;
      }
    }
    
    public double wristPosition() {
      switch (this) {
        case Stowed: return Constants.WRIST_STOWED_POSITION;
        case Intake: return Constants.WRIST_INTAKE_POSITION;
        case Source: return Constants.WRIST_SOURCE_POSITION;
        case SourceShot: return Constants.WRIST_SOURCE_SHOT_POSITION;
        case SpeakerHigh: return Constants.WRIST_SPEAKER_HIGH_POSITION;
        case SpeakerLow: return Constants.WRIST_SPEAKER_LOW_POSITION;
        case Amp: return Constants.WRIST_AMP_POSITION;
        case Amp2: return Constants.WRIST_AMP_DOWN_POSITION;
        case AmpStow: return Constants.WRIST_AMP_STOW_POSITION;
        case SubWoofer: return Constants.WRIST_SUBWOOFER_POSITION;
        case PodiumHigh: return Constants.WRIST_PODIUM_HIGH_POSITION;
        case PodiumLow: return Constants.WRIST_PODIUM_LOW_POSITION;
        case ClimberUp: return Constants.WRIST_CLIMBER_HIGH_POSITION;
        case ClimberMid: return Constants.WRIST_CLIMBER_MID_POSITION;
        case ClimberLow: return Constants.WRIST_CLIMBER_LOW_POSITION;
        case ClimberStowed: return Constants.WRIST_CLIMBER_STOWED_POSITION;
        case ClimberCompact: return Constants.WRIST_CLIMBER_COMPACT_POSITION;
        case Popcorn: return Constants.WRIST_POPCORN_POSITION;
        default: return Constants.WRIST_STOWED_POSITION;
      }
    }

    public double elevatorPosition() {
      switch (this) {
        case Stowed: return Constants.ELEVATOR_STOWED_POSITION;
        case Intake: return Constants.ELEVATOR_INTAKE_POSITION;
        case Source: return Constants.ELEVATOR_SOURCE_POSITION;
        case SourceShot: return Constants.ELEVATOR_SOURCE_SHOT_POSITION;
        case SpeakerHigh: return Constants.ELEVATOR_SPEAKER_HIGH_POSITION;
        case SpeakerLow: return Constants.ELEVATOR_SPEAKER_LOW_POSITION;
        case Amp: return Constants.ELEVATOR_AMP_POSITION;
        case Amp2: return Constants.ELEVATOR_AMP_DOWN_POSITION;
        case AmpStow: return Constants.ELEVATOR_AMP_STOW_POSITION;
        case SubWoofer: return Constants.ELEVATOR_SUBWOOFER_POSITION;
        case PodiumHigh: return Constants.ELEVATOR_PODIUM_HIGH_POSITION;
        case PodiumLow: return Constants.ELEVATOR_PODIUM_LOW_POSITION;
        case ClimberUp: return Constants.ELEVATOR_CLIMBER_HIGH_POSITION;
        case ClimberMid: return Constants.ELEVATOR_CLIMBER_MID_POSITION;
        case ClimberLow: return Constants.ELEVATOR_CLIMBER_LOW_POSITION;
        case ClimberStowed: return Constants.ELEVATOR_CLIMBER_STOWED_POSITION;
        case ClimberCompact: return Constants.ELEVATOR_CLIMBER_COMPACT_POSITION;
        case Popcorn: return Constants.ELEVATOR_POPCORN_POSITION;
        default: return Constants.ELEVATOR_STOWED_POSITION;
      }
    }

    public double wristMaxV() {
      switch (this) {
        case Stowed: return Constants.WRIST_STOWED_MAXV;
        case Intake: return Constants.WRIST_INTAKE_MAXV;
        case Source: return Constants.WRIST_SOURCE_MAXV;
        case SourceShot: return Constants.WRIST_SOURCE_SHOT_MAXV;
        case SpeakerHigh: return Constants.WRIST_SPEAKER_HIGH_MAXV;
        case SpeakerLow: return Constants.WRIST_SPEAKER_LOW_MAXV;
        case Amp: return Constants.WRIST_AMP_MAXV;
        case Amp2: return Constants.WRIST_AMP_DOWN_MAXV;
        case AmpStow: return Constants.WRIST_AMP_STOW_MAXV;
        case SubWoofer: return Constants.WRIST_SUBWOOFER_MAXV;
        case PodiumHigh: return Constants.WRIST_PODIUM_HIGH_MAXV;
        case PodiumLow: return Constants.WRIST_PODIUM_LOW_MAXV;
        case ClimberUp: return Constants.WRIST_CLIMBER_HIGH_MAXV;
        case ClimberMid: return Constants.WRIST_CLIMBER_MID_MAXV;
        case ClimberLow: return Constants.WRIST_CLIMBER_LOW_MAXV;
        case ClimberStowed: return Constants.WRIST_CLIMBER_STOWED_MAXV;
        case ClimberCompact: return Constants.WRIST_CLIMBER_COMPACT_MAXV;
        case Popcorn: return Constants.WRIST_POPCORN_MAXV;
        default: return Constants.WRIST_STOWED_MAXV;
      }
    }

    public double elevatorMaxV() {
      switch (this) {
        case Stowed: return Constants.ELEVATOR_STOWED_MAXV;
        case Intake: return Constants.ELEVATOR_INTAKE_MAXV;
        case Source: return Constants.ELEVATOR_SOURCE_MAXV;
        case SourceShot: return Constants.ELEVATOR_SOURCE_SHOT_MAXV;
        case SpeakerHigh: return Constants.ELEVATOR_SPEAKER_HIGH_MAXV;
        case SpeakerLow: return Constants.ELEVATOR_SPEAKER_LOW_MAXV;
        case Amp: return Constants.ELEVATOR_AMP_MAXV;
        case Amp2: return Constants.ELEVATOR_AMP_DOWN_MAXV;
        case AmpStow: return Constants.ELEVATOR_AMP_STOW_MAXV;
        case SubWoofer: return Constants.ELEVATOR_SUBWOOFER_MAXV;
        case PodiumHigh: return Constants.ELEVATOR_PODIUM_HIGH_MAXV;
        case PodiumLow: return Constants.ELEVATOR_PODIUM_LOW_MAXV;
        case ClimberUp: return Constants.ELEVATOR_CLIMBER_HIGH_MAXV;
        case ClimberMid: return Constants.ELEVATOR_CLIMBER_MID_MAXV;
        case ClimberLow: return Constants.ELEVATOR_CLIMBER_LOW_MAXV;
        case ClimberStowed: return Constants.ELEVATOR_CLIMBER_STOWED_MAXV;
        case ClimberCompact: return Constants.ELEVATOR_CLIMBER_COMPACT_MAXV;
        case Popcorn: return Constants.ELEVATOR_POPCORN_MAXV;
        default: return Constants.ELEVATOR_STOWED_MAXV;
      }
    }
  
  @Override
  public String toString() {
      return String.format("Position(Shoulder: %f, Wrist: %f, Elevator: %f)", shoulderPosition(), wristPosition(), elevatorPosition());
    }
  }
  
  public ArmSubsystem() {
    leftShoulderMotor = PIDMotor.makeMotor(Constants.SHOULDER_LEFT_MOTOR_ID, "Shoulder Left", 0.05, 0, 0, 0,
        ControlType.kPosition, 120, 300);
    rightShoulderMotor = PIDMotor.makeMotor(Constants.SHOULDER_RIGHT_MOTOR_ID, "Shoulder Right", 0.05, 0, 0, 0,
        ControlType.kPosition, 120, 300);
    rightShoulderMotor.follow(leftShoulderMotor, true);
    wristMotor = PIDMotor.makeMotor(Constants.WRIST_MOTOR_ID, "Wrist", 0.05, 0, 0, 0, ControlType.kPosition, 150, 1000);
    elevatorMotor = PIDMotor.makeMotor(Constants.ELEVATOR_MOTOR_ID, "Elevator", 0.06, 0, 0, 0, ControlType.kPosition, Constants.ELEVATOR_DEFAULT_MAXV, 500);

    leftShoulderMotor.setCurrentLimit(30);
    rightShoulderMotor.setCurrentLimit(30);
    wristMotor.setCurrentLimit(30);
    elevatorMotor.setCurrentLimit(30);

    leftShoulderMotor.generateTrapezoidPath(0, 0);
    wristMotor.generateTrapezoidPath(0, 0);
    elevatorMotor.generateTrapezoidPath(0, 0);
  }

  @Override
  public void periodic() {
    if(isTrapezoidal && (!leftShoulderMotor.atPosition() || !wristMotor.atPosition() || !elevatorMotor.atPosition())) {
      leftShoulderMotor.runTrapezoidPath();
      wristMotor.runTrapezoidPath();
      elevatorMotor.runTrapezoidPath();
    }
  }

  /**
   * Generates a trapezoidal path towards the intended arm position.
   * @param target Target arm position.
   */
  public void unsafeSetPosition(ArmPosition target) {
    this.target = target;
    leftShoulderMotor.generateTrapezoidPath(target.shoulderPosition(), 0);
    wristMotor.generateTrapezoidPath(target.wristPosition(), 0, target.wristMaxV());
    SmartDashboard.putNumber("elevator target", target.elevatorPosition());
    elevatorMotor.generateTrapezoidPath(target.elevatorPosition(), 0, target.elevatorMaxV());
  }



  public void safeManualLimelightSetPosition(double shoulderEncoderCount, double wristEncoderCount, double elevatorEncoderCount, boolean isTrapezoidal){
    this.isTrapezoidal = isTrapezoidal;
    //if(Math.abs(shoulderEncoderCount) > Constants.SHOULDER_ENCODER_MAX)
    if(wristEncoderCount > Constants.WRIST_ENCODER_MAX){
      System.err.println("tried to set wrist past max");
    } else {
      wristMotor.setTarget(wristEncoderCount); 
      // elevatorMotor.setTarget(elevatorEncoderCount);
    }
  }

  public void disableBrakeMode(){
    leftShoulderMotor.setIdleCoastMode();
    rightShoulderMotor.setIdleCoastMode();
    wristMotor.setIdleCoastMode();
    elevatorMotor.setIdleCoastMode();
  }

  public void enableBrakeMode(){
    leftShoulderMotor.setIdleBrakeMode();
    rightShoulderMotor.setIdleBrakeMode();
    wristMotor.setIdleBrakeMode();
    elevatorMotor.setIdleBrakeMode();
  }

  public void zeroEncoders(){
    leftShoulderMotor.resetEncoder();
    rightShoulderMotor.resetEncoder();
    wristMotor.resetEncoder();
    elevatorMotor.resetEncoder();
  }

  public boolean armAtPostion(){
    return leftShoulderMotor.atPosition() && wristMotor.atPosition() && elevatorMotor.atPosition();
  }
}