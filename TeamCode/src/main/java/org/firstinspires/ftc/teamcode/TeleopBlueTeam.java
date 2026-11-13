package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop: BLUE", group = "Robot")
public class TeleopBlueTeam extends TeleopRedTeam
{
    // Everything is the same as red teleop, but it aims for blue APRIL tags.
    @Override
    public String teamColor() {
        return "BLUE";
    }
}
