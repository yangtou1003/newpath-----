package frc.robot.subsystem;

import java.sql.Driver;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends  SubsystemBase{ 
    public VictorSP leftMaster =new VictorSP(0);
    public VictorSP rightMaster =new VictorSP(2);

    public VictorSP leftslaver =new VictorSP(1);
    public VictorSP rightslaver =new VictorSP(3);

    public Encoder leftEncoder =new Encoder(1,0);
    public Encoder rightEncoder =new Encoder(2,3);

    public AHRS gyro = new AHRS(SPI.Port.kMXP);
    public Pose2d pose;
    public SpeedController leftmotor;
    public SpeedController rightmotor;
    public DifferentialDrive drive;
    
            
    public static final double kWheelDiameterMeters = Units.inchesToMeters(6);
    public static final double kEncoderPulsesPerRev = 360;//E4T
    public static final double kLeftMetersPerPulse = Math.PI * kWheelDiameterMeters / kEncoderPulsesPerRev;
    public static final double kRightMetersPerPulse = Math.PI * kWheelDiameterMeters / kEncoderPulsesPerRev;
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
      }

    private DifferentialDriveOdometry m_odometry;
    public Drivetrain(){
        leftmotor= new SpeedControllerGroup(leftMaster, leftslaver);
        rightmotor=new SpeedControllerGroup(rightMaster, rightslaver);
        
        leftmotor.setInverted(false);
        rightmotor.setInverted(false);

        drive= new DifferentialDrive(leftmotor,rightmotor);
        drive.setDeadband(0);

        leftEncoder.setDistancePerPulse(kLeftMetersPerPulse);
        rightEncoder.setDistancePerPulse(kRightMetersPerPulse);

        resetEncoders();

        m_odometry = new DifferentialDriveOdometry(getHeading());
    }

    @Override
    public void periodic(){
        // Get my gyro angle. We are negating the value because gyros return positive
        // values as the robot turns clockwise. This is not standard convention that is
        // used by the WPILib classes.
        var gyroAngle = Rotation2d.fromDegrees(-gyro.getAngle());
        
        // Update the pose
        pose = m_odometry.update(gyroAngle, leftEncoder.getDistance(), rightEncoder.getDistance());
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
      return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }
    
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, getHeading());
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftmotor.setVoltage(leftVolts);
        rightmotor.setVoltage(-rightVolts);
    }
    
    public double getAverageEncoderDistance() {
      return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
    }
    public Encoder getLeftEncoder() {
        return leftEncoder;
    }
    public Encoder getRightEncoder() {
        return rightEncoder;
    }
    

    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }
    public void zeroHeading() {
        gyro.reset();
    }


    public Rotation2d getHeading(){
        return Rotation2d.fromDegrees(-gyro.getAngle());
     }
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
      }

}
