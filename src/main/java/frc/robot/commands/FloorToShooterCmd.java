package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FlumperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.ShooterSubsystem.ShootState;
import edu.wpi.first.wpilibj2.command.Command;

public class FloorToShooterCmd extends Command {
  private final FlumperSubsystem flumperSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ArmSubsystem armSubsystem;
  boolean isForwards;

  public FloorToShooterCmd(FlumperSubsystem flumper, ShooterSubsystem shooter, ArmSubsystem arm, boolean isForwards) {
    flumperSubsystem = flumper;
    shooterSubsystem = shooter;
    armSubsystem = arm;
    this.isForwards = isForwards;
    addRequirements(flumper, shooter, arm);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    armSubsystem.unsafeSetPosition(ArmPosition.Intake);
    if(armSubsystem.checkShoulderPosition() && armSubsystem.checkWristPosition()){
      if(isForwards){
        flumperSubsystem.eat();
        shooterSubsystem.state = ShootState.Intake;
      } else{
        flumperSubsystem.spit();
        shooterSubsystem.state = ShootState.Reverse;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    flumperSubsystem.stop();
    shooterSubsystem.state = ShootState.Idle;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
