package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Low Side: RED", group="Robot")
public class Auton_Red_LowSide extends Auton_Blue_LowSide{
    @Override
    public String teamColor() {
        return "RED";
    }
}
