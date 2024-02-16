package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.LIFT_POSITION_COEFFICIENT;
import static org.firstinspires.ftc.teamcode.RobotConstants.LIFT_POSITION_TOLERANCE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LIFT_SWIVEL_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.LOWER_SWIVEL_POSITION;

import android.widget.Button;

import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
@TeleOp(name="MainOpMode")
public class MainTeleOp extends LinearOpMode {

    private SampleMecanumDrive drive;
    private Motor slidesMotor;
    private ServoEx clawL, clawR, swivelL, swivelR;
    private MecanumDrive mec_drive;
    private GamepadEx driverController1, driverController2;
    private double leftTrigger, rightTrigger;
    private Encoder slidesEncoder;
    private int liftPosition = 0;
    private int targetPosition = 0;
    private ColorSensor colorSensorL, colorSensorR;
    private DistanceSensor distanceSensorL, distanceSensorR;

    @Override
    public void runOpMode() throws InterruptedException {
        driverController1 = new GamepadEx(gamepad1);
        driverController2 = new GamepadEx(gamepad2);

//        TriggerReader triggerReaderL = new TriggerReader(driverController1, GamepadKeys.Trigger.LEFT_TRIGGER);
//        TriggerReader triggerReaderR = new TriggerReader(driverController1, GamepadKeys.Trigger.RIGHT_TRIGGER);
        ButtonReader buttonReaderA = new ButtonReader(driverController1, GamepadKeys.Button.A);
        ButtonReader buttonReaderB = new ButtonReader(driverController1, GamepadKeys.Button.B);
        ButtonReader buttonReaderX = new ButtonReader(driverController1, GamepadKeys.Button.X);
        ButtonReader buttonReaderY = new ButtonReader(driverController1, GamepadKeys.Button.Y);

        drive = new SampleMecanumDrive(hardwareMap);

        slidesMotor = new Motor(hardwareMap, "slidesMotor");
        slidesMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slidesMotor.resetEncoder();

        slidesMotor.setRunMode(Motor.RunMode.PositionControl);
        slidesMotor.setPositionCoefficient(LIFT_POSITION_COEFFICIENT);
        slidesMotor.setPositionTolerance(LIFT_POSITION_TOLERANCE);

        liftPosition = slidesMotor.getCurrentPosition();

        clawL = new SimpleServo(hardwareMap, "clawL", 0, 360);
        clawR = new SimpleServo(hardwareMap, "clawR", 90, 360);
        clawR.setInverted(true);
//        closeClaw();

//        colorSensorL = hardwareMap.get(ColorSensor.class, "colorSensorL");
//        distanceSensorL = hardwareMap.get(DistanceSensor.class, "colorSensorL");
        colorSensorR = hardwareMap.get(ColorSensor.class, "colorSensorR");
        distanceSensorR = hardwareMap.get(DistanceSensor.class, "colorSensorR");

        swivelL = new SimpleServo(hardwareMap, "swivelL", 0, 360);
        ServoImplEx swivelLImpl = hardwareMap.get(ServoImplEx.class, "swivelL");
        swivelLImpl.setPwmRange(new PwmControl.PwmRange(500, 2500));
        swivelR = new SimpleServo(hardwareMap, "swivelR", 0, 360);

        swivelR.setInverted(true);

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Ticks: ", slidesMotor.getCurrentPosition());
            telemetry.addData("Target Position ", targetPosition);
            telemetry.addData("Distance", distanceSensorR.getDistance(DistanceUnit.CM));
            telemetry.addData("Red", colorSensorR.red());
            telemetry.addData("Green", colorSensorR.green());
            telemetry.addData("Blue", colorSensorR.blue());
            telemetry.addData("Alpha", colorSensorR.alpha());
            telemetry.update();

            leftTrigger = driverController1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            rightTrigger = driverController1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

            if (leftTrigger > 0) {
                targetPosition -= 20;
            }
            else if (rightTrigger > 0) {
                targetPosition += 20;
            }

            if (targetPosition < 0) {
                targetPosition = 0;
            } else if (targetPosition > 2050) {
                targetPosition = 1950;
            }

            slidesMotor.setTargetPosition(targetPosition);

            if (slidesMotor.atTargetPosition()) {
                slidesMotor.set(0);
            } else {
                slidesMotor.set(1);
            }

            //--------------------------------------------------------------

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * 0.75,
                            -gamepad1.left_stick_x * 0.75,
                            -gamepad1.right_stick_x * 0.75
                    )
            );

            //-------------------------------------------------------------

            if (distanceSensorR.getDistance(DistanceUnit.CM) < 2.5 && isWhite(colorSensorR.red(), colorSensorR.blue(), colorSensorR.green())) {
                clawR.setPosition(0);
            }

            if (gamepad1.a) {
                openClaw();
            }
            else if (gamepad1.y) {
                closeClaw();
            }
            else if (gamepad1.left_bumper) {
                lowerSwivel();
            }
            else if (gamepad1.right_bumper) {
                liftSwivel();
            }
        }
    }

    private void openClaw(){
        clawL.setPosition(0.1);
        clawR.setPosition(0.1);
    }

    private void closeClaw(){
        clawL.setPosition(0.3);
        clawR.setPosition(0.3);
    }
    private void liftSwivel() {
        swivelR.setPosition(LIFT_SWIVEL_POSITION);
    }
    private void lowerSwivel() {
        swivelR.setPosition(LOWER_SWIVEL_POSITION);
    }

    private boolean isWhite(int r, int g, int b) {
        if (r >= 280 && r <= 420) {
            if (g >= 500 && g <= 700) {
                if (b >= 400 && b <= 700) {
                    return true;
                }
            }
        }
        return false;
    }
}
