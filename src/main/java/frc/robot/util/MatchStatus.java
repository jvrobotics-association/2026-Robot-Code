package frc.robot.util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class MatchStatus {

    // You can use Timer.getMatchTime(), which would give you an APPROXIMATE time remaining (countdown)
    // This timer starts as soon as teleop is enabled and counts up.


    private static Timer matchTime = new Timer();
    private static final int TRANSITION_TIME = 10;
    private static final int SHIFT_TIME = 25;
    private static final int END_GAME = 30;
    private static final int PREPARE_TIME = 10;

    private enum GameStates {
        NONE,
        TEST,
        AUTO,
        TRANSITION,
        SHIFT_1,
        SHIFT_2,
        SHIFT_3,
        SHIFT_4,
        END_GAME,
        ERR
    }

    private enum Hubs {
        NONE,
        BLUE,
        RED,
        BOTH,
        ERR
    }

    public static Hubs getAutoWinner() {
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()){
            return Hubs.NONE;
        } else if(gameData.charAt(0) == 'B'){
            return Hubs.BLUE;
        } else if (gameData.charAt(0) == 'R'){
            return Hubs.RED;
        }

        return Hubs.ERR;
    }

    public static void startTimer(){
        matchTime.reset();
        matchTime.start();
    }

    public static GameStates getCurrentShift(){
        if (DriverStation.isAutonomous()) {
            return GameStates.AUTO;
        } else if (DriverStation.isDisabled()){
            return GameStates.NONE;
        } else if (DriverStation.isTest()){
            return GameStates.TEST;
        } else if (matchTime.get() <= TRANSITION_TIME){
            return GameStates.TRANSITION;
        } else if (matchTime.get() <= SHIFT_TIME){
            return GameStates.SHIFT_1;
        } else if (matchTime.get() <= SHIFT_TIME*2){
            return GameStates.SHIFT_2;
        } else if (matchTime.get() <= SHIFT_TIME*3){
            return GameStates.SHIFT_3;
        } else if (matchTime.get() <= SHIFT_TIME*4){
            return GameStates.SHIFT_4;
        } else if (matchTime.get() > TRANSITION_TIME + SHIFT_TIME*4 && matchTime.get() < TRANSITION_TIME + SHIFT_TIME*4 + END_GAME){
            return GameStates.END_GAME;
        }

        return GameStates.ERR;
    }

    public static Hubs getActiveHub(){
        if(getCurrentShift() == GameStates.AUTO || getCurrentShift() == GameStates.TRANSITION ||getCurrentShift() == GameStates.END_GAME) {
            return Hubs.BOTH;
        } else if (getCurrentShift() == GameStates.SHIFT_1 || getCurrentShift() == GameStates.SHIFT_3){
            if (getAutoWinner() == Hubs.BLUE){
                return Hubs.RED;
            }
            return Hubs.BLUE;
        } else if (getCurrentShift() == GameStates.SHIFT_2 || getCurrentShift() == GameStates.SHIFT_4){
            if (getAutoWinner() == Hubs.BLUE){
                return Hubs.BLUE;
            }
            return Hubs.RED;
        }
        
        return Hubs.ERR;
    }
    
    // should we save value as member variable to cut down on API calls?
    public static Hubs getCurrentHub(){
        if(DriverStation.getAlliance().isEmpty()){
            return Hubs.NONE;
        } else if(DriverStation.getAlliance().get() == Alliance.Blue){
            return Hubs.BLUE;
        } else if(DriverStation.getAlliance().get() == Alliance.Red){
            return Hubs.RED;
        }
        return Hubs.ERR;
    }

    public static boolean isHubActive(){
        if(getCurrentHub() == Hubs.NONE || getCurrentHub() == Hubs.ERR || getActiveHub() == Hubs.ERR){
            return false;
        } else if (getActiveHub() == Hubs.BOTH){
            return true;
        } else if (getCurrentHub() == getActiveHub()){
            return true;
        }
        return false;
    }

    public static int timeTillActive(){
        if (isHubActive()){
            return 0;
        } 

        // if you're hub is inactive, it will ALWAYS be active next shift
        if(getCurrentShift() == GameStates.SHIFT_1){
            return ((TRANSITION_TIME + SHIFT_TIME*1) - (int)matchTime.get());
        } else if(getCurrentShift() == GameStates.SHIFT_2){
            return ((TRANSITION_TIME + SHIFT_TIME*2) - (int)matchTime.get());
        } else if(getCurrentShift() == GameStates.SHIFT_3){
            return ((TRANSITION_TIME + SHIFT_TIME*3) - (int)matchTime.get());
        } else if(getCurrentShift() == GameStates.SHIFT_4){
            return ((TRANSITION_TIME + SHIFT_TIME*4) - (int)matchTime.get());
        }


        return 0;
    }

    public static boolean approachingTimeToShoot(){
        return (timeTillActive() <= PREPARE_TIME);
    }

}
