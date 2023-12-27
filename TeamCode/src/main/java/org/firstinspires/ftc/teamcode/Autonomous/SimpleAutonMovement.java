package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="SimpleAuton")
public class SimpleAutonMovement extends LinearOpMode {

    private MecanumDrive mec_drive;
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx driverController1 = new GamepadEx(gamepad1);
        ButtonReader buttonReaderA = new ButtonReader(driverController1, GamepadKeys.Button.A);
        ButtonReader buttonReaderB = new ButtonReader(driverController1, GamepadKeys.Button.B);
        ButtonReader buttonReaderX = new ButtonReader(driverController1, GamepadKeys.Button.X);
        ButtonReader buttonReaderY = new ButtonReader(driverController1, GamepadKeys.Button.Y);

        Motor fL = new Motor(hardwareMap, "fL");
        Motor fR = new Motor(hardwareMap, "fR");
        Motor bL = new Motor(hardwareMap, "bL");
        Motor bR = new Motor(hardwareMap, "bR");
        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        mec_drive = new MecanumDrive(fL, fR, bL, bR);

        boolean blue = false;
        boolean red = false;
        boolean red1 = false;
        boolean blue1 = false;

        while (!isStarted() && !isStopRequested()) {

            telemetry.addLine("PRESS A");
            telemetry.update();
            //right
            if (gamepad1.a) {
                blue = true;
                red = false;
//                red1 = false;
//                blue1 = false;
            }

            //left
            else if (gamepad1.x) {
                red = true;
                blue = false;
//                red1 = false;
//                blue1 = false;
            }

            //forward left
//            else if (gamepad1.y) {
//                red1 = true;
//                blue = false;
//                red = false;
//                blue1 = false;
//            }
//
//            //forward right
//            else if (gamepad1.b) {
//                blue1 = true;
//                blue = false;
//                red = false;
//                red1 = false;
//            }
        }

        mec_drive.driveRobotCentric(driverController1.getLeftX(), driverController1.getLeftY(), driverController1.getRightX(), false);

        //right
        if (blue) {
            blueMovement();
        }

        //left
        else if (red) {
            blueMovement();
        }

    }

    public void blueMovement() {
        ElapsedTime elapsedTime = new ElapsedTime();
        while (elapsedTime.seconds() < 4) {
            mec_drive.driveRobotCentric(0, 0.5, 0, false);
        }
    }

    public void redMovement() {
        ElapsedTime elapsedTime = new ElapsedTime();
        while (elapsedTime.seconds() < 4) {
            mec_drive.driveRobotCentric(0, 0.5, 0, false);
        }
    }

    public void redMovement1() {
        ElapsedTime elapsedTime = new ElapsedTime();
        while (elapsedTime.seconds() < 0.5) {
            mec_drive.driveRobotCentric(0, 0.5, 0, false);
        }
        while (elapsedTime.seconds() < 3) {
            mec_drive.driveRobotCentric(-0.5, 0, 0.25, false);
        }
    }

    public void blueMovement1() {
        ElapsedTime elapsedTime = new ElapsedTime();
        while (elapsedTime.seconds() < 0.5) {
            mec_drive.driveRobotCentric(0, 0.5, 0, false);
        }
        while (elapsedTime.seconds() < 3) {
            mec_drive.driveRobotCentric(0.5, 0, -0.25, false);
        }
    }
}
