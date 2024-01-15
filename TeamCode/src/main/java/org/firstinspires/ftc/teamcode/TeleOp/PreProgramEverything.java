package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class PreProgramEverything extends OpMode {

    Servo arm1;
    Servo arm2;

    @Override
    public void init() {

        arm1 = hardwareMap.get(Servo.class, "Arm1");
        arm2 = hardwareMap.get(Servo.class, "Arm2");

    }

    @Override
    public void loop() {

        Double pos1 = arm1.getPosition();
        Double pos2 = arm2.getPosition();


    }
}