package org.firstinspires.ftc.teamcode.drive.opmode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@Disabled
@TeleOp(name="Motor Test")
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Motor motor = new Motor(hardwareMap, "intake");
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        GamepadEx driverController1 = new GamepadEx(gamepad1);

        waitForStart();
        while (opModeIsActive()) {
            double leftTrigger = driverController1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            double rightTrigger = driverController1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

            if (leftTrigger > 0) {
                motor.set(leftTrigger * -0.25);
            }
            else if (rightTrigger > 0) {
                motor.set(rightTrigger * 0.25);
            }
            else {
                motor.set(0);
            }
        }
    }
}
