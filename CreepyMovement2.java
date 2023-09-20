package org.firstinspires.ftc.teamcode;

//imports
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
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;


@Autonomous

public class CreepyMovement2 extends LinearOpMode {

    // Declare the global variable
    MecanumDrive driveBase;
    OpenCvCamera camera;
    RevIMU imu;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    
    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    
    @Override
    public void runOpMode() throws InterruptedException {

        //setup time
        ElapsedTime time = new ElapsedTime();

        //setup revIMU
        imu = new RevIMU(hardwareMap);
        imu.init();
        
        
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(864,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        
        


        //setup motors
        Motor frontLeftMotor = new Motor (hardwareMap, "frontLeft");
        Motor frontRightMotor = new Motor (hardwareMap, "frontRight");
        Motor backLeftMotor = new Motor (hardwareMap, "backLeft");
        Motor backRightMotor = new Motor (hardwareMap, "backRight");

        // Set the drivebase before using it
        driveBase = new MecanumDrive (frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

        //declare variables
        double strafeSpeed;
        double fowardSpeed;
        double rotateSpeed;
        double heading;

        //wait for the game to start
        waitForStart();
        
        boolean foundTag = false;
        int coneNumber = 0;

        
        while (opModeIsActive() && !foundTag) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if(currentDetections.size() != 0) {
                coneNumber = currentDetections.get(0).id;
                foundTag = true;
            }
        }
        
        
        switch(coneNumber) {
            case 1:
                driveFoward(1.4);
                rotateLeft(85);
                driveFoward(1.25);
                rotateRight(0);
                break;
            case 2:
                driveFoward(1.25);
                break;
            case 3:
                driveFoward(1.4);
                rotateRight(85);
                driveFoward(1.25);
                rotateLeft(0);
                break;
            default:
                break;
        }
            
            
        //stop the robot
        requestOpModeStop();

        
    }


    //setup methods

    public void driveFoward(double time) {

        double strafeSpeed = 0;
        double fowardSpeed = 1;
        double rotateSpeed = 0;
        double heading = 0;

        drive(strafeSpeed ,fowardSpeed, rotateSpeed, heading, time);

    }
    
    public void strafeLeft(double time) {

        double strafeSpeed = 1;
        double fowardSpeed = .75;
        double rotateSpeed = 0;
        double heading = 0;

        drive(strafeSpeed ,fowardSpeed, rotateSpeed, heading, time);

    }

    public void strafeRight(double time) {

        double strafeSpeed = -1;
        double fowardSpeed = .7;
        double rotateSpeed = 0;
        double heading = 0;

        drive(strafeSpeed ,fowardSpeed, rotateSpeed, heading, time);

    }
       
     public void rotateRight(double degrees) {

        double strafeSpeed = 0;
        double fowardSpeed = 0;
        double rotateSpeed = -0.75;
        double heading = 0;

        while(-imu.getHeading() <= degrees && opModeIsActive()) {
            driveBase.driveFieldCentric(strafeSpeed, fowardSpeed, rotateSpeed,heading, false);

        }
     }
    
    public void rotateLeft(double degrees) {

        double strafeSpeed = 0;
        double fowardSpeed = 0;
        double rotateSpeed = 0.75;
        double heading = 0;
        
        while(imu.getHeading() <= degrees && opModeIsActive()) {
            driveBase.driveFieldCentric(strafeSpeed, fowardSpeed, rotateSpeed,heading, false);
        }
    }
    
    public void drive(double strafeSpeed, double fowardSpeed, double rotateSpeed, double heading, double time ) {
        
        double startTime = getRuntime();
        
        while(getRuntime() - startTime < time && opModeIsActive()) {

            // this. is only needed if a parameter has the same name
            driveBase.driveFieldCentric(strafeSpeed, fowardSpeed, rotateSpeed,heading, false);

        }
    
    }
    
}
