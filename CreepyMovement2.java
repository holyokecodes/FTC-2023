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

        
        double strafeSpeed;
        double fowardSpeed;
        double rotateSpeed;
        double heading;
        
        waitForStart();
        
       
        while(opModeIsActive() && !isStopRequested()) {
           
            
           
           
        
            requestOpModeStop();
        
            
        }    
    }
    
    public void driveFoward() {
      
        double startTime = getRuntime();

        double strafeSpeed = 0;
        double fowardSpeed = 1;
        double rotateSpeed = 0;
        double heading = 0;
            
      
    }
    
        public void drive(double strafeSpeed, double startTime, double fowardSpeed, double rotateSpeed, double heading ) {
    
        while(getRuntime() - startTime < time && opModeIsActive()) {
    
           this.driveBase.driveFieldCentric(strafeSpeed, fowardSpeed, rotateSpeed,heading);
        
           }  
    
    
    }
}
