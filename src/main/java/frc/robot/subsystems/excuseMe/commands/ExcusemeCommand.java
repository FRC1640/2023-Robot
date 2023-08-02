package frc.robot.subsystems.excuseMe.commands;

import java.io.OutputStream;
import java.net.Socket;
import java.nio.charset.StandardCharsets;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExcusemeCommand extends CommandBase {
    
    @Override
    public void initialize(){

        new Thread(() -> {
            try(Socket s = new Socket("10.16.40.146", 5000); OutputStream o = s.getOutputStream();){
                o.write("GET /play HTTP/1.1\r\n".getBytes(StandardCharsets.UTF_8));
            }
            catch(Exception e){

            }
        }).start();;
    }

    @Override
    public void execute() {
    }

    @Override 
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
