package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;

import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;

@Autonomous

public class CreepyMovement2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
    
        ElapsedTime time = new ElapsedTime();
        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();
        
        Motor frontLeftMotor = new Motor (hardwareMap, "frontLeft");
        Motor frontRightMotor = new Motor (hardwareMap, "frontRight");
        Motor backLeftMotor = new Motor (hardwareMap, "backLeft");
        Motor backRightMotor = new Motor (hardwareMap, "backRight");
    
        MecanumDrive driveBase = new MecanumDrive (frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        
        Translation2d frontLeftMotorLocation = new Translation2d(0.216, 0.216);     
        Translation2d frontRightMotorLocation = new Translation2d(0.216, -0.216);
        Translation2d backLeftMotorLocation = new Translation2d(-0.216, 0.216);
        Translation2d backRightMotorLocation = new Translation2d(-0.216, -0.216);
        
        MecanumDriveKinematics kinematics = new MecanumDriveKinematics
        (
            frontLeftMotorLocation, frontRightMotorLocation,
            backLeftMotorLocation, backRightMotorLocation
        );
        
        
        
        // MecanumDriveOdometry odometry = new MecanumDriveOdometry
        // (
        //     kinematics, imu.getRotation2d()
        // );
        
        // Pose2d target = new Pose2d(0, 1, new Rotation2d());
        // Pose2d current = new Pose2d(0, 0, new Rotation2d());
        
        // waitForStart();
        
        // Waypoint p1 = new StartWaypoint(0, 0);
        // Waypoint p2 = new GeneralWaypoint(0, 1);
        
        // Path path = new Path(p1, p2);
        
        // waitForStart();
        
        // while(opModeIsActive() && !isStopRequested()) {
        //     MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds
        //     (
        //         frontLeftMotor.getRate(), frontRightMotor.getRate(),
        //         backLeftMotor.getRate(), backRightMotor.getRate()
        //     );
            
        //     current = odometry.updateWithTime(time.time(), imu.getRotation2d(), wheelSpeeds);
       
        //     double xDiff = current.getX() - target.getX();
        //     double yDiff = current.getY() - target.getY();

            
        //     driveBase.driveFieldCentric(xDiff, yDiff, 0, imu.getRotation2d().getDegrees());
            
        // }
    }
}