package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

// a: (0.83, -0.95)
// b: (-0.02, 0.4)
// SLOP; 1.35 / -0.85

// y = mx + b
// 0.4 = (1.35 / -0.85)(-0.02) + b
// 0.4 - b = (1.35 / -0.85)(-0.02)
// -b = (1.35 / -0.85)(-0.02) - 0.4
// b = 0.4 - (1.35 / -0.85)(-0.02)

// y = mx + b
// b = y - mx
// b = -0.95 - (1.35 / -0.85) * (0.83)


// m = 1.35 / -0.85
// b = 0.3682352941176472


public class LimelightSubsystem extends SubsystemBase implements Runnable {
  public boolean isAiming = false;
  public boolean readyToShoot = false;

  public boolean limelightRotation = false;
  String name;
  public double tagTx = 0.0;
  public double tagTy = 0.0;
  public boolean tagTv = false;

  public LimelightSubsystem(String name) {
    this.name = name;
    new Thread(this, "`" + name + "` Thread").start();
  }

  @Override
  public void periodic() {
    printDashboard();
  }

  String dashboardKey() {
    return "`" + name + "`";
  }

  // Prints various values of every target
  public void printDashboard() {
    synchronized (this) {
      SmartDashboard.putNumber(dashboardKey() + "TX", tagTx);
      SmartDashboard.putNumber(dashboardKey() + "TY", tagTy);
      // SmartDashboard.putBoolean(dashboardKey() + "TV", tagTv);
      // SmartDashboard.putBoolean(dashboardKey() + "Rotation Enabled", limelightRotation);
    }
  }

  public void setPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex(name, pipeline);
  }

  @Override
  public void run() {
    while (true) {
      synchronized (this) {
        tagTx = LimelightHelpers.getTX(name);
        tagTy = LimelightHelpers.getTY(name);
        tagTv = LimelightHelpers.getTV(name);
      }
      try {
        Thread.sleep(10);
      } catch (InterruptedException iex) {
      }
    }
  }
}
