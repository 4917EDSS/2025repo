// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
// import com.ctre.phoenix6.swerve.SwerveModule;
// import com.ctre.phoenix6.swerve.SwerveRequest;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.DrivetrainSub;

// /*
//  * You should consider using the more terse Command factories API instead
//  * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
//  */
// public class DriveToPosFieldRelativeCmd extends Command {
//   private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
//       .withDriveRequestType(DriveRequestType.OpenLoopVoltage);;
//   private final DrivetrainSub m_drivetrainSub;
//   double targetX;
//   double targetY;
//   double power;
//   Translation2d[] modulePosArray;
//   Pose2d startingPos;
//   Pose2d currentPos;
//   double xDist;
//   double yDist;

//   /** Creates a new DriveToPosobotRelativeCmd. */
//   public DriveToPosFieldRelativeCmd(double targetX, double targetY, double targetRot, Double power,
//       DrivetrainSub drivetrainSub) {
//     m_drivetrainSub = drivetrainSub;
//     this.targetX = targetX;
//     this.targetY = targetY;
//     this.power = power;
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   public Pose2d calcPos(){
//     double xPos = 0;
//     double yPos = 0;
//     modulePosArray = m_drivetrainSub.getModuleLocations();
//     for(int i=0; i<4; i++){
//       xPos+=modulePosArray[i].getX();
//       yPos+=modulePosArray[i].getY();
//     }
//     xPos/=4;
//     yPos/=4;
//     return new Pose2d(xPos, yPos, m_drivetrainSub.getPose().getRotation());
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     startingPos = calcPos();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     currentPos = calcPos();
//     xDist = targetX + currentPos.getX();


//     double totalDist = Math.sqrt((targetX * targetX) + (targetY * targetY));
//     double xPower = targetX / totalDist;
//     double yPower = targetY / totalDist;

//     m_drivetrainSub.setControl(
//         driveRobotCentric.withVelocityX(xPower * Constants.DriveTrain.kMaxSpeed * power)
//             .withVelocityY(yPower * Constants.DriveTrain.kMaxSpeed * power)
//             .withRotationalRate(Constants.DriveTrain.kMaxAngularRate * 0.5));
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
