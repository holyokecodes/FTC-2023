package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;

@TeleOp(name="TheNemo", group="Linear Opmode")

public class TheNemo extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();

        Motor frontLeftMotor = new Motor (hardwareMap, "frontLeft");
        Motor frontRightMotor = new Motor (hardwareMap, "frontRight");
        Motor backLeftMotor = new Motor (hardwareMap, "backLeft");
        Motor backRightMotor = new Motor (hardwareMap, "backRight");
        
        Motor armLifterMotor = new Motor (hardwareMap, "armLifter");
        armLifterMotor.setRunMode(Motor.RunMode.VelocityControl);
        
        // Setup mecanum
        MecanumDrive driveBase = new MecanumDrive (frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
            
        // Setup controllers
        GamepadEx controller1 = new GamepadEx(gamepad1);
        
        TriggerReader rightTrigger = new TriggerReader(controller1, GamepadKeys.Trigger.RIGHT_TRIGGER);
        
        ButtonReader leftBumper = new ButtonReader(controller1, GamepadKeys.Button.LEFT_BUMPER);
        ButtonReader rightBumper = new ButtonReader(controller1, GamepadKeys.Button.RIGHT_BUMPER);
        
        double speed = 1.0;
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
    
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            rightTrigger.readValue();
            
            if(rightTrigger.isDown()) {
                speed = 0.25;
            } else {
                //speed modifier
                speed = 1.0;
            }
            
            leftBumper.readValue();
            rightBumper.readValue();
            
            if(leftBumper.isDown()) {
                // Lower arm
                armLifterMotor.set(1);
                telemetry.addData("Left Bumper", leftBumper.isDown());
            } else if(rightBumper.isDown()) {
                // Raise arm
                armLifterMotor.set(-1);
                telemetry.addData("Right Bumper", rightBumper.isDown());
            } else {
                // Arm still
                armLifterMotor.set(0);
            }
                
            // Strafe, Forward, Rotate
            driveBase.driveFieldCentric(-controller1.getLeftX() * speed, controller1.getLeftY() * speed, -controller1.getRightX() * speed, -imu.getRotation2d().getDegrees(), false);
            
            telemetry.addData("Right Trigger", rightTrigger.isDown());
            telemetry.update();
        
        }
    }
}
