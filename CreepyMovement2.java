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

@Autonomous

public class CreepyMovement2 extends LinearOpMode {

    // Declare the global variable
    MecanumDrive driveBase;

    @Override
    public void runOpMode() throws InterruptedException {

        //setup time
        ElapsedTime time = new ElapsedTime();

        //setup revIMU
        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();


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

        //run until the game is over
        while(opModeIsActive() && !isStopRequested()) {


            //stop the robot
            requestOpModeStop();

        }
    }


    //setup methods


    public void driveFoward() {

        double startTime = getRuntime();

        double strafeSpeed = 0;
        double fowardSpeed = 1;
        double rotateSpeed = 0;
        double heading = 0;


    }

    public void drive(double strafeSpeed, double startTime, double fowardSpeed, double rotateSpeed, double heading ) {

        while(getRuntime() - startTime < time && opModeIsActive()) {

            // this. is only needed if a parameter has the same name
            driveBase.driveFieldCentric(strafeSpeed, fowardSpeed, rotateSpeed,heading);

        }
    }
}
