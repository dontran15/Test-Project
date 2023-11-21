// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.subsystems.swerve.SwerveModuleIO.ModuleData;

public class Swerve extends SubsystemBase {

  private final SwerveModuleIO[] modules = new SwerveModuleIO[4];
  private final ModuleData[] moduleData = new ModuleData[4];

  /** Creates a new ExampleSubsystem. */
  public Swerve() {

    if (Constants.Robot.isSim) {
      for (int i = 0; i < 4; i++) {
        modules[i] = new SwerveModuleSim();
        moduleData[i] = new ModuleData();
      }
    }

  }

  public Rotation2d getRotation2d(){
    return new Rotation2d();
  }

  public void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(states[i]);
    }
  }

  @Override
  public void periodic() {

    for (int i = 0; i < 4; i++) {
      modules[i].updateData(moduleData[i]);

    }

    // This method will be called once per scheduler run
    double[] realStates = new double[8];
    double[] theoreticalStates = new double[8];

    for (int i = 0; i < 8; i+=2) {
      realStates[i] = moduleData[i/2].turnPositionRad/Math.PI*180;
      realStates[i+1] = moduleData[i/2].drivePositionM;

      theoreticalStates[i] = moduleData[i/2].theoreticalState.angle.getDegrees();
      theoreticalStates[i+1] = moduleData[i/2].theoreticalState.speedMetersPerSecond;
    }
    
    SmartDashboard.putNumberArray("Real States", realStates);
    SmartDashboard.putNumberArray("Theoretical States", theoreticalStates);
  }


}