package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

@Config
public class SampleMecanumDrive extends MecanumDrive {


    public SampleMecanumDrive(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight) {
        super(frontLeft, frontRight, backLeft, backRight);
    }
}
