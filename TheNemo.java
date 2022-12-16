package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
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
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.ServoEx;


@TeleOp(name="TheNemo", group="Linear Opmode")

public class TheNemo extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    static final int MAX_ENCODER = 1000;
    static final int MIN_ENCODER = 0;
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();
        
        SimpleServo handMoverServo = new SimpleServo (hardwareMap, "handMover", 0, 180);

        Motor frontLeftMotor = new Motor (hardwareMap, "frontLeft");
        Motor frontRightMotor = new Motor (hardwareMap, "frontRight");
        Motor backLeftMotor = new Motor (hardwareMap, "backLeft");
        Motor backRightMotor = new Motor (hardwareMap, "backRight");
        Motor armLifterMotor = new Motor (hardwareMap, "armLifter");
        armLifterMotor.setRunMode(Motor.RunMode.PositionControl);
        armLifterMotor.setPositionCoefficient(0.05);
        armLifterMotor.setPositionTolerance(13.6);
        
        // Setup mecanum
        MecanumDrive driveBase = new MecanumDrive (frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
            
        // Setup controllers
        GamepadEx controller1 = new GamepadEx(gamepad1);
        
        TriggerReader rightTrigger = new TriggerReader(controller1, GamepadKeys.Trigger.RIGHT_TRIGGER);
        
        ButtonReader leftBumper = new ButtonReader(controller1, GamepadKeys.Button.LEFT_BUMPER);
        ButtonReader rightBumper = new ButtonReader(controller1, GamepadKeys.Button.RIGHT_BUMPER);
        ButtonReader xButton = new ButtonReader(controller1, GamepadKeys.Button.X);
        ButtonReader aButton = new ButtonReader(controller1, GamepadKeys.Button.A);
        ButtonReader yButton = new ButtonReader(controller1, GamepadKeys.Button.Y);
        
        
        double speed = 1.0;
        boolean handOpen = true;
        int openAngle = 100;
        int closeAngle = 151;
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        
    
        int targetPosition = 0;
        armLifterMotor.resetEncoder();
    
    
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            
            telemetry.addData("Servo Angle", handMoverServo.getAngle());
            
            
            rightTrigger.readValue();
            
            if(rightTrigger.isDown()) {
                speed = 0.25;
            } else {
                //speed modifier
                speed = 1.0;
            }
            
            xButton.readValue();
            if(xButton.wasJustPressed()) {
                handOpen = !handOpen; //switch true or false
                handMoverServo.turnToAngle(handOpen ? openAngle : closeAngle); //if true, first if false, second
            }
            
            aButton.readValue();
            yButton.readValue();
            
            double angle = handMoverServo.getAngle();
            
            if(yButton.isDown() && angle > openAngle) {
                handMoverServo.rotateByAngle(-.5);
            }
            
            if(aButton.isDown() && angle < closeAngle) {
                handMoverServo.rotateByAngle(.5);
            }
            
            
            
            
            telemetry.addData("handOpen",handOpen);
            telemetry.addData("handPosition",handMoverServo.getPosition());
             
            leftBumper.readValue();
            rightBumper.readValue();
            
            if(leftBumper.isDown()) {
                //Lower arm
                targetPosition += 1;
                //telemetry.addData("Left Bumper", leftBumper.isDown());
            } else if(rightBumper.isDown()) {
                //Raise arm
                targetPosition += -1;
                //telemetry.addData("Right Bumper", rightBumper.isDown());
            }
            
            if(targetPosition>MAX_ENCODER) {
                targetPosition = MAX_ENCODER;
            }
            
            if(targetPosition<MIN_ENCODER) {
                targetPosition = MIN_ENCODER;
            }
            
            armLifterMotor.setTargetPosition(targetPosition);
            
            if (armLifterMotor.atTargetPosition()){
                armLifterMotor.stopMotor();
            } else {
                armLifterMotor.set(0.75);
            }
            
         
            telemetry.addData("targetPosition", targetPosition);
            telemetry.addData("encoderPosition", armLifterMotor.getCurrentPosition());
                
            // Strafe, Forward, Rotate
            // driveBase.driveFieldCentric(-controller1.getLeftX() * speed, controller1.getLeftY() * speed, -controller1.getRightX() * speed, -imu.getRotation2d().getDegrees(), false);
            
            //telemetry.addData("Right Trigger", rightTrigger.isDown());
            telemetry.update();
        
        }
    }
}
