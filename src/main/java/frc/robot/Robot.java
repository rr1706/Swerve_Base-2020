/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Autonomous;
import frc.robot.subsystems.IMU;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.Pair;
import frc.robot.utilities.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private SwerveDrivetrain driveTrain;

  private static final XboxController xbox1 = new XboxController(0);
  //public static final XboxController xbox2 = new XboxController(1);

  private double FWD;
  private double STR;
  private double RCW;

  private IMU imu = new IMU();

  @Override
  public void robotInit() {
    driveTrain = new SwerveDrivetrain();
    IMU.init();
    xbox1.setDeadband(0.09);
    //xbox2.setDeadband(0.09);
  }

  public void robotPeriodic() {
    driveTrain.printTest();
  }
  
  @Override
  public void autonomousInit() {
    Autonomous.init();
  }

  @Override
  public void autonomousPeriodic() {
    Autonomous.periodic();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    // Drive commands (-1.0 to 1.0)
    FWD = xbox1.LStickY() / 10.5 * DriverStation.getBatteryVoltage();
    STR = -xbox1.LStickX() / 10.5 * DriverStation.getBatteryVoltage();
    RCW = -xbox1.RStickX()/ 10.5 * DriverStation.getBatteryVoltage();

    driveTrain.printTest();

    if (xbox1.RStickButton()) {
      driveTrain.lock();
    } else {
      driveTrain.unlock();
    }


    if (xbox1.RB()) {
      driveTrain.drive(new Pair<>(STR, FWD), RCW);
    } else {
      driveTrain.drive(MathUtils.convertOrientation(imu.getAngle(), FWD, STR), RCW);
    }

    if (xbox1.Back()) {
      imu.reset(0.0);
    }

}

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    driveTrain.printTest();
    driveTrain.setTest(0.1, 0.05);
  }

}
