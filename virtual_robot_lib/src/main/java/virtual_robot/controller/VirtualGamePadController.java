package virtual_robot.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.StackPane;
import javafx.scene.shape.Circle;

public class VirtualGamePadController {

    @FXML StackPane joyStickLeftPane;
    @FXML StackPane joyStickRightPane;
    @FXML Circle joyStickLeftHandle;
    @FXML Circle joyStickRightHandle;
    @FXML Label controllerLabel;
    @FXML Button btnRT;
    @FXML Button btnLT;
    @FXML Button btnRB;
    @FXML Button btnLB;
    @FXML Button btnU;
    @FXML Button btnD;
    @FXML Button btnR;
    @FXML Button btnL;
    @FXML Button btnSwitch;

    volatile float left_stick_x = 0;
    volatile float left_stick_y = 0;
    volatile float right_stick_x = 0;
    volatile float right_stick_y = 0;
    volatile float rightTrigger = 0;
    volatile float leftTrigger = 0;
    volatile boolean xPressed = false;
    volatile boolean yPressed = false;
    volatile boolean aPressed = false;
    volatile boolean bPressed = false;
    volatile boolean uPressed = false;
    volatile boolean dPressed = false;
    volatile boolean lPressed = false;
    volatile boolean rPressed = false;
    volatile boolean rbPressed = false;
    volatile boolean lbPressed = false;
    boolean alternate = false;

    VirtualRobotController virtualRobotController = null;

    void setVirtualRobotController(VirtualRobotController vrController){
        virtualRobotController = vrController;
    }

    @FXML
    private void handleJoystickDrag(MouseEvent arg){
        if (!virtualRobotController.getOpModeInitialized()) return;
        float x = (float)Math.max(10, Math.min(110, arg.getX()));
        float y = (float)Math.max(10, Math.min(110, arg.getY()));
        if (arg.getSource() == joyStickLeftPane){
            joyStickLeftHandle.setTranslateX(x-10);
            joyStickLeftHandle.setTranslateY(y-10);
            left_stick_x = (x - 60.0f) / 50.0f;
            left_stick_y = (y - 60.0f) / 50.0f;
        }
        else if (arg.getSource() == joyStickRightPane){
            joyStickRightHandle.setTranslateX(x-10);
            joyStickRightHandle.setTranslateY(y-10);
            right_stick_x = (x - 60.0f) / 50.0f;
            right_stick_y = (y - 60.0f) / 50.0f;
        }
    }

    @FXML
    private void handleGamePadButtonMouseEvent(MouseEvent arg){
        if (!virtualRobotController.getOpModeInitialized()) return;
        Button btn = (Button)arg.getSource();
        boolean result;
        float trigResult;


        if (arg.getEventType() == MouseEvent.MOUSE_EXITED || arg.getEventType() == MouseEvent.MOUSE_RELEASED) {
            result = false;
            trigResult = (float) 0.0;
        }
        else if (arg.getEventType() == MouseEvent.MOUSE_PRESSED) {
            result = true;
            trigResult = (float) 0.99;
        }
        else return;

        if (btn == btnRT) rightTrigger = trigResult;
        else if (btn == btnLT) leftTrigger = trigResult;
        else if (btn == btnRB) rbPressed = result;
        else if (btn == btnLB) lbPressed = result;

        if(alternate) {
            if (btn == btnL) lPressed = result;
            else if (btn == btnU) uPressed = result;
            else if (btn == btnD) dPressed = result;
            else if (btn == btnR) rPressed = result;
        } else {
            if (btn == btnL) xPressed = result;
            else if (btn == btnU) yPressed = result;
            else if (btn == btnD) aPressed = result;
            else if (btn == btnR) bPressed = result;
        }
        if (btn == btnSwitch && result) {
            if(alternate) {
                btnD.textProperty().setValue("A");
                btnU.textProperty().setValue("Y");
                btnL.textProperty().setValue("X");
                btnR.textProperty().setValue("B");
                alternate = false;
            } else {
                btnD.textProperty().setValue("D");
                btnU.textProperty().setValue("U");
                btnL.textProperty().setValue("L");
                btnR.textProperty().setValue("R");
                alternate = true;
            }
        }
    }

    void resetGamePad(){
        left_stick_y = 0;
        left_stick_x = 0;
        right_stick_x = 0;
        right_stick_y = 0;
        aPressed = false;
        bPressed = false;
        xPressed = false;
        yPressed = false;
        dPressed = false;
        uPressed = false;
        rPressed = false;
        lPressed = false;
        rbPressed = false;
        lbPressed = false;
        rightTrigger = 0;
        leftTrigger = 0;
        joyStickLeftHandle.setTranslateX(50);
        joyStickLeftHandle.setTranslateY(50);
        joyStickRightHandle.setTranslateX(50);
        joyStickRightHandle.setTranslateY(50);
    }

    public class ControllerState {

        public final float leftStickX;
        public final float leftStickY;
        public final float rightStickX;
        public final float rightStickY;
        public final boolean a;
        public final boolean b;
        public final boolean x;
        public final boolean y;
        public final boolean u;
        public final boolean d;
        public final boolean l;
        public final boolean r;
        public final boolean rb;
        public final boolean lb;
        public final float rt;
        public final float lt;


        public ControllerState() {
            leftStickX = VirtualGamePadController.this.left_stick_x;
            leftStickY = VirtualGamePadController.this.left_stick_y;

            rightStickX = VirtualGamePadController.this.right_stick_x;
            rightStickY = VirtualGamePadController.this.right_stick_y;

            a = VirtualGamePadController.this.aPressed;
            b = VirtualGamePadController.this.bPressed;
            x = VirtualGamePadController.this.xPressed;
            y = VirtualGamePadController.this.yPressed;

            u = VirtualGamePadController.this.uPressed;
            d = VirtualGamePadController.this.dPressed;
            l = VirtualGamePadController.this.lPressed;
            r = VirtualGamePadController.this.rPressed;

            rb = VirtualGamePadController.this.rbPressed;
            lb = VirtualGamePadController.this.lbPressed;

            rt = VirtualGamePadController.this.rightTrigger;
            lt = VirtualGamePadController.this.leftTrigger;
        }
    }

    ControllerState getState(){
        return new ControllerState();
    }

}
