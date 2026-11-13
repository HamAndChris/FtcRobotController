package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="High Side: RED", group="Robot")
public class Auton_Red_HighSide extends Auton_Blue_HighSide{
    @Override
    public String teamColor() {
        return "RED";
    }
}
