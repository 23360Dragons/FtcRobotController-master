package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoProgram extends OpMode {

    IMU _orient;
    DcMotor _rightFront;
    DcMotor _leftFront;
    DcMotor _rightBack;
    DcMotor _leftBack;
    Servo _intake;

    @Override
    public void init() {

    _intake = hardwareMap.get(Servo.class, "IntakeServo1");

    }

    @Override
    public void loop() {

        if (gamepad2.left_trigger!=0){

            _intake.setPosition(90);

        } else if (gamepad2.right_trigger!=0){

            _intake.setPosition(0);

        }

    }
}
