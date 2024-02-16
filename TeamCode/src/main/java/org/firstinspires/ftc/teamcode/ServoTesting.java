package org.firstinspires.ftc.teamcode;

import android.widget.Button;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="ServoTesting")
public class ServoTesting extends LinearOpMode {

    private SampleMecanumDrive drive;
    private Motor slidesMotor;
    private ServoEx clawL, clawR, swivelL, swivelR;
    private MecanumDrive mec_drive;
    private GamepadEx driverController1, driverController2;
    @Override
    public void runOpMode() throws InterruptedException {
        driverController1 = new GamepadEx(gamepad1);
//        ServoEx servo = new SimpleServo(hardwareMap, "clawL", 0, 360);
        ServoEx servo2 = new SimpleServo(hardwareMap, "clawR", 0, 360);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
//                servo.setPosition(1);
            }
            if (gamepad1.y) {
                servo2.rotateByAngle(15, AngleUnit.DEGREES);
            }
        }
    }
}
