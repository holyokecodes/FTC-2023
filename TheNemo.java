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
    private final ElapsedTime runtime = new ElapsedTime();

    //declare encoder variables
    static final int MAX_ENCODER = 1000;
    static final int MIN_ENCODER = 0;

    @Override
    public void runOpMode() {


        // telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //set up revIMU
        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();

        //set up servos
        SimpleServo handMoverServo = new SimpleServo (hardwareMap, "handMover", 0, 180);

        //set up motors
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



        //declare variables
        double speed = 1.0;
        boolean handOpen = true;
        int openAngle = 100;
        int closeAngle = 151;
        int targetPosition = 0;
        double angle;

        // Wait for the game to start
        waitForStart();
        runtime.reset();

        //reset encoder
        armLifterMotor.resetEncoder();


        // run until the end of the match
        while (opModeIsActive()) {

            //speed modifier
            rightTrigger.readValue();

            if(rightTrigger.isDown()) {
                speed = 0.25;
            } else {
                speed = 1.0;
            }

            //hand opener and closer
            xButton.readValue();
            if(xButton.wasJustPressed()) {
                handOpen = !handOpen; //switch true or false
                handMoverServo.turnToAngle(handOpen ? openAngle : closeAngle); //if true, first if false, second
            }

            //precise hand mover
            aButton.readValue();
            yButton.readValue();
            
            //set angle to servo angle
            angle = handMoverServo.getAngle();

            if(yButton.isDown() && angle > openAngle) {
                handMoverServo.rotateByAngle(-.5);
            }

            if(aButton.isDown() && angle < closeAngle) {
                handMoverServo.rotateByAngle(.5);
            }


            //arm raiser
            leftBumper.readValue();
            rightBumper.readValue();

            if(leftBumper.isDown()) {
                //lower arm
                targetPosition += 1;

            } else if(rightBumper.isDown()) {
                //raise arm
                targetPosition += -1;

            }


            //arm lifter safety
            if (targetPosition > MAX_ENCODER) {
                targetPosition = MAX_ENCODER;
            }

            if (targetPosition<MIN_ENCODER) {
                targetPosition = MIN_ENCODER;
            }

            //actually move arm
            armLifterMotor.setTargetPosition(targetPosition);

            //brake motor
            if (armLifterMotor.atTargetPosition()){
                armLifterMotor.stopMotor();
            } else {
                armLifterMotor.set(0.75);
            }

            //strafe, forward, rotate robot
            driveBase.driveFieldCentric(-controller1.getLeftX() * speed, controller1.getLeftY() * speed, -controller1.getRightX() * speed, -imu.getRotation2d().getDegrees(), false);

            //setup telemetry
            telemetry.addData("targetPosition", targetPosition);
            telemetry.addData("encoderPosition", armLifterMotor.getCurrentPosition());
            telemetry.addData("handOpen",handOpen);
            telemetry.addData("handPosition",handMoverServo.getPosition());
            telemetry.addData("Servo Angle", handMoverServo.getAngle());
            telemetry.update();

        }
    }
}

