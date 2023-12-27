package org.firstinspires.ftc.teamcode;

import android.widget.Button;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="MainOpMode")
public class MainTeleOp extends LinearOpMode {

    private Motor fL, fR, bL, bR;
    private Motor slidesMotor;
    private ServoEx intakeServo, outtakeServo1, outtakeServo2;
    private MecanumDrive mec_drive;
    private GamepadEx driverController1, driverController2;
    @Override
    public void runOpMode() throws InterruptedException {
        driverController1 = new GamepadEx(gamepad1);
        driverController2 = new GamepadEx(gamepad2);

        TriggerReader triggerReaderL = new TriggerReader(driverController1, GamepadKeys.Trigger.LEFT_TRIGGER);
        TriggerReader triggerReaderR = new TriggerReader(driverController1, GamepadKeys.Trigger.RIGHT_TRIGGER);
        ButtonReader buttonReaderA = new ButtonReader(driverController1, GamepadKeys.Button.A);
        ButtonReader buttonReaderB = new ButtonReader(driverController1, GamepadKeys.Button.B);
        ButtonReader buttonReaderX = new ButtonReader(driverController1, GamepadKeys.Button.X);
        ButtonReader buttonReaderY = new ButtonReader(driverController1, GamepadKeys.Button.Y);

        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

//        slidesMotor = new Motor(hardwareMap, "motorOne");
//        slidesMotor.setRunMode(Motor.RunMode.RawPower);
////        ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
//        intakeServo = new SimpleServo(hardwareMap, "servo_name", 0, 90);
//        outtakeServo1 = new SimpleServo(hardwareMap, "servo_name", 0, 180);
//        outtakeServo2 = new SimpleServo(hardwareMap, "servo_name", 0, 180);

        mec_drive = new MecanumDrive(fL, fR, bL, bR);

        waitForStart();
        while(opModeIsActive()) {
            mec_drive.driveRobotCentric(driverController1.getLeftX() * 0.25, driverController1.getLeftY() * 0.25, driverController1.getRightX() * 0.25, false);

//            if (triggerReaderR.isDown()) {
////                slidesMotor.set(feedforward.calculate(leftVel, rightVel));
//                slidesMotor.set(0.3);
//            }
//            else if (triggerReaderL.isDown()) {
//                slidesMotor.set(-0.3);
//            }
//            else if (buttonReaderA.wasJustPressed()) {
//                intakeServo.turnToAngle(0);
//            }
//            else if (buttonReaderY.wasJustPressed()) {
//                intakeServo.turnToAngle(45);
//            }
//            else if (buttonReaderX.isDown()) {
//                outtakeServo1.rotateByAngle(-1);
//                outtakeServo2.rotateByAngle(-1);
//            }
//            else if (buttonReaderB.isDown()) {
//                outtakeServo1.rotateByAngle(1);
//                outtakeServo2.rotateByAngle(1);
//            }
        }
    }
}
