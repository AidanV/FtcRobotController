package virtual_robot.controller;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import io.vertx.core.AbstractVerticle;
import io.vertx.core.Handler;
import io.vertx.core.Vertx;
import io.vertx.core.buffer.Buffer;
import io.vertx.core.http.HttpServer;
import io.vertx.core.http.ServerWebSocket;
import io.vertx.core.json.JsonObject;
import virtual_robot.controller.opmodes.BlueOpMode;
import virtual_robot.controller.robots.classes.MechanumBot;




/**
 * For internal use only. Main class for the JavaFX application.
 */
public class VirtualRobotApplication extends AbstractVerticle {

  static ServerWebSocket webSocket = null;



  public static void sendData(String data){
    webSocket.writeBinaryMessage(Buffer.buffer(data));
  }


  @Override
  public void start() {

    System.out.println("http://127.0.0.1:8080/index.html");
    HttpServer httpServer = vertx.createHttpServer();

    httpServer.requestHandler(req -> {
      System.out.println("got request...: ");
      System.out.println(req.uri());
      req.response().sendFile("/Users/vanduyneai/Documents/GitHub/2021FtcRobotController/virtual_robot_lib/src/main/java/FileStream/"+req.uri());

    }).webSocketHandler(ws -> {
      System.out.println("inside ws... ");
      System.out.println(ws.uri());
      webSocket = ws;
      vertx.setPeriodic(1000, new Handler<Long>() {

        @Override
        public void handle(Long event) {
          if(MechanumBot.controller != null) MechanumBot.controller.handler();
        }
      });

      ws.handler(data -> updateRobot(data.toJsonObject()));

    }).listen(8080);



  }


  public static void main(String[] args) {
    VirtualRobotController virtualRobotController = new VirtualRobotController();
    virtualRobotController.initialize();
    Vertx vertx = Vertx.vertx();
    vertx.deployVerticle(new VirtualRobotApplication());
  }


  public static void updateRobot(JsonObject msg){
    if(msg.getBoolean("isStarted")) {
//      OpMode.getVirtualRobotController().startOpMode();
    }
    JsonObject encoders =  msg.getJsonObject("encoders");

    BlueOpMode.bleft.setCurrentPosition(encoders.getInteger("bleftPosition"));
    BlueOpMode.fleft.setCurrentPosition(encoders.getInteger("fleftPosition"));
    BlueOpMode.bright.setCurrentPosition(encoders.getInteger("brightPosition"));
    BlueOpMode.fright.setCurrentPosition(encoders.getInteger("frightPosition"));
  }
}
