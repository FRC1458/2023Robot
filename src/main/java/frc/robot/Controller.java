package frc.robot;

public interface Controller {
     boolean lockWheels();
     boolean stateManual();
     boolean stateAlign();
     boolean stateBalance();
     boolean resetNavX();

     boolean resetArm();

     boolean armTop();
     boolean armMiddle();
     boolean armBottom();
     boolean fieldOriented();
     boolean clawOpen();
     boolean clawClose();
     boolean armExtend();
     boolean armRetract();
     boolean armUp();
     boolean armDown();

     double getSwerveR();
     double getSwerveX();
     double getSwerveY();

}
