package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.sql.Driver;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystem.Drivetrain;
import frc.robot.subsystem.Drivetrain.AutoConstants;

public class RobotContainer {
    public SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 1.19, 0.0757);
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(0.55);
    private final Drivetrain robotdrive = new Drivetrain();

    public Command getAutonomouscCommand() {
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(feedforward, kDriveKinematics, 10);

        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

        String trajectoryJSON = "/home/lvuser/deploy/paths/Unnamed.wpilib.json";
        Path trajectoryPath;
        Trajectory Trajectory;
        try {
        trajectoryPath =
        Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        
        } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON,
        ex.getStackTrace());
                
        }
        
        
        Trajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                // new Translation2d(1,0.5)
                ),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(1, 0, new Rotation2d(0)),
                // Pass config
                config);

        RamseteCommand command = new RamseteCommand(Trajectory, robotdrive::getPose, new RamseteController(2, 0.7),
                feedforward, kDriveKinematics, robotdrive::getWheelSpeeds, new PIDController(0.214, 0, 0),
                new PIDController(0.214, 0, 0), robotdrive::tankDriveVolts, robotdrive);

                return command.andThen(() -> robotdrive.tankDriveVolts(0, 0));
    }
}
