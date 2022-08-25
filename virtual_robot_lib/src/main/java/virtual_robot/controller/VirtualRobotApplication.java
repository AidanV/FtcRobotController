package virtual_robot.controller;

import java.net.*;
import java.io.*;
import java.time.Instant;
import java.util.*;

import io.vertx.core.AbstractVerticle;
import io.vertx.core.Handler;
import io.vertx.core.Vertx;
import io.vertx.core.buffer.Buffer;
import io.vertx.core.http.HttpServer;
import io.vertx.core.http.ServerWebSocket;
import io.vertx.core.json.JsonObject;
import io.vertx.ext.web.client.HttpRequest;
import virtual_robot.controller.robots.classes.MechanumBot;




/**
 * For internal use only. Main class for the JavaFX application.
 */
public class VirtualRobotApplication extends AbstractVerticle {



//    private static virtual_robot.controller.VirtualRobotController controllerHandle;

//    @Override
//    public void start(Stage primaryStage) throws Exception{
//        FXMLLoader loader = new FXMLLoader(getClass().getResource("virtual_robot.fxml"));
//        Parent root = (BorderPane)loader.load();
//        controllerHandle = loader.getController();
//        primaryStage.setTitle("Virtual Robot");
//        primaryStage.setScene(new Scene(root));
//        primaryStage.setResizable(false);
//        primaryStage.setOnShowing(new EventHandler<WindowEvent>() {
//            @Override
//            public void handle(WindowEvent event) {
//                controllerHandle.setConfig(null);
//            }
//        });
//        primaryStage.show();
//    }
//
//    @Override
//    public void stop() {
//        if (controllerHandle.executorService != null && !controllerHandle.executorService.isShutdown()) {
//            controllerHandle.executorService.shutdownNow();
//        }
//        if (controllerHandle.gamePadExecutorService != null && !controllerHandle.gamePadExecutorService.isShutdown()) {
//            controllerHandle.gamePadExecutorService.shutdownNow();
//        }
//        controllerHandle.gamePadHelper.quit();
//    }

//    public static VirtualRobotController getControllerHandle(){return controllerHandle;}

//
//    public static void main(String[] args)
//    {
//        VirtualRobotController controllerHandler = new VirtualRobotController();
//        // read arguments
////        if (args.length!=2) {
////            System.out.println("Usage: java FileServer <port> <wwwhome>");
////            System.exit(-1);
////        }
//        int port = 8000;//Integer.parseInt(args[0]);
//        String wwwhome = "/Users/vanduyneai/Documents/GitHub/2021FtcRobotController/virtual_robot_lib/src/main/java/FileStream";//args[1];
//
//        // open server socket
//        ServerSocket socket = null;
//        try {
//            socket = new ServerSocket(port);
//        } catch (IOException e) {
//            System.err.println("Could not start server: " + e);
//            System.exit(-1);
//        }
//        System.out.println("FileServer accepting connections on port " + port);
//
//        // request handler loop
//        while (true) {
////            controllerHandler.handler();//THIS LINE TO UPDATE EVERY FRAME
//            Socket connection = null;
//            try {
//                // wait for request
//                connection = socket.accept();
//                BufferedReader in = new BufferedReader(new InputStreamReader(connection.getInputStream()));
//                OutputStream out = new BufferedOutputStream(connection.getOutputStream());
//                PrintStream pout = new PrintStream(out);
//
//                // read first line of request (ignore the rest)
//                String request = in.readLine();
//                if (request==null)
//                    continue;
//                log(connection, request);
//                while (true) {
//                    String misc = in.readLine();
//                    if (misc==null || misc.length()==0)
//                        break;
//                }
//
//                // parse the line
//                if (!request.startsWith("GET") || request.length()<14 ||
//                        !(request.endsWith("HTTP/1.0") || request.endsWith("HTTP/1.1"))) {
//                    // bad request
//                    errorReport(pout, connection, "400", "Bad Request",
//                            "Your browser sent a request that " +
//                                    "this server could not understand.");
//                } else {
//                    String req = request.substring(4, request.length()-9).trim();
//                    if (req.indexOf("..")!=-1 ||
//                            req.indexOf("/.ht")!=-1 || req.endsWith("~")) {
//                        // evil hacker trying to read non-wwwhome or secret file
//                        errorReport(pout, connection, "403", "Forbidden",
//                                "You don't have permission to access the requested URL.");
//                    } else {
//                        String path = wwwhome + "/" + req;
//                        File f = new File(path);
//                        if (f.isDirectory() && !path.endsWith("/")) {
//                            // redirect browser if referring to directory without final '/'
//                            pout.print("HTTP/1.0 301 Moved Permanently\r\n" +
//                                    "Location: http://" +
//                                    connection.getLocalAddress().getHostAddress() + ":" +
//                                    connection.getLocalPort() + "/" + req + "/\r\n\r\n");
//                            log(connection, "301 Moved Permanently");
//                        } else {
//                            if (f.isDirectory()) {
//                                // if directory, implicitly add 'index.html'
//                                path = path + "index.html";
//                                f = new File(path);
//                            }
//                            try {
//                                // send file
//                                InputStream file = new FileInputStream(f);
//                                pout.print("HTTP/1.0 200 OK\r\n" +
//                                        "Content-Type: " + guessContentType(path) + "\r\n" +
//                                        "Date: " + new Date() + "\r\n" +
//                                        "Server: FileServer 1.0\r\n\r\n");
//                                sendFile(file, out); // send raw file
//                                log(connection, "200 OK");
//                            } catch (FileNotFoundException e) {
//                                // file not found
//                                errorReport(pout, connection, "404", "Not Found",
//                                        "The requested URL was not found on this server.");
//                            }
//                        }
//                    }
//                }
//                out.flush();
//            } catch (IOException e) { System.err.println(e); }
//            try {
//                if (connection != null) connection.close();
//            } catch (IOException e) { System.err.println(e); }
//        }
//    }
//
//    private static void log(Socket connection, String msg)
//    {
//        System.err.println(new Date() + " [" + connection.getInetAddress().getHostAddress() +
//                ":" + connection.getPort() + "] " + msg);
//    }
//
//    private static void errorReport(PrintStream pout, Socket connection,
//                                    String code, String title, String msg)
//    {
//        pout.print("HTTP/1.0 " + code + " " + title + "\r\n" +
//                "\r\n" +
//                "<!DOCTYPE HTML PUBLIC \"-//IETF//DTD HTML 2.0//EN\">\r\n" +
//                "<TITLE>" + code + " " + title + "</TITLE>\r\n" +
//                "</HEAD><BODY>\r\n" +
//                "<H1>" + title + "</H1>\r\n" + msg + "<P>\r\n" +
//                "<HR><ADDRESS>FileServer 1.0 at " +
//                connection.getLocalAddress().getHostName() +
//                " Port " + connection.getLocalPort() + "</ADDRESS>\r\n" +
//                "</BODY></HTML>\r\n");
//        log(connection, code + " " + title);
//    }
//
//    private static String guessContentType(String path)
//    {
//        if (path.endsWith(".html") || path.endsWith(".htm"))
//            return "text/html";
//        else if (path.endsWith(".txt") || path.endsWith(".java"))
//            return "text/plain";
//        else if (path.endsWith(".gif"))
//            return "image/gif";
//        else if (path.endsWith(".class"))
//            return "application/octet-stream";
//        else if (path.endsWith(".jpg") || path.endsWith(".jpeg"))
//            return "image/jpeg";
//        else
//            return "text/plain";
//    }
//
//    private static void sendFile(InputStream file, OutputStream out)
//    {
//        try {
//            byte[] buffer = new byte[1000];
//            while (file.available()>0)
//                out.write(buffer, 0, file.read(buffer));
//        } catch (IOException e) { System.err.println(e); }
//    }

  static ServerWebSocket webSocket = null;

  public static void sendData(String data){
    webSocket.writeBinaryMessage(Buffer.buffer(data));
  }


  @Override
  public void start() {




    System.out.println("some letters in front of the word starting...");
    HttpServer httpServer = vertx.createHttpServer();

    httpServer.requestHandler(req -> {
      System.out.println("got request...: ");
      System.out.println(req.uri());

      if (req.uri().equals("/ws.html")) {
        req.response()
          .sendFile("/Users/vanduyneai/Documents/GitHub/2021FtcRobotController/virtual_robot_lib/src/main/java/filestream/ws.html");
      }

      if (req.uri().equals("/index.html")) {
        req.response()
          .sendFile("/Users/vanduyneai/Documents/GitHub/2021FtcRobotController/virtual_robot_lib/src/main/java/FileStream/index.html");
      }

      if (req.uri().equals("/babylonScene.js")) {
        req.response()
          .sendFile("/Users/vanduyneai/Documents/GitHub/2021FtcRobotController/virtual_robot_lib/src/main/java/FileStream/babylonScene.js");
      }

      if (req.uri().equals("/webSocket.js")) {
        req.response()
          .sendFile("/Users/vanduyneai/Documents/GitHub/2021FtcRobotController/virtual_robot_lib/src/main/java/FileStream/webSocket.js");
      }

    }).webSocketHandler(ws -> {
      System.out.println("inside ws... ");
      System.out.println(ws.uri());
      webSocket = ws;
//      webSocketHandler = ws;
      vertx.setPeriodic(1000, new Handler<Long>() {

        @Override
        public void handle(Long event) {

//            virtualRobotController.handler();
          if(MechanumBot.controller != null) MechanumBot.controller.handler();
//          ws.writeBinaryMessage(Buffer.buffer(Date.from(Instant.now()).toString()));
        }
      });
      ws.handler(ws::writeBinaryMessage);
    }).listen(8080);



  }


  public static void main(String[] args) {
    VirtualRobotController virtualRobotController = new VirtualRobotController();
    virtualRobotController.initialize();
    Vertx vertx = Vertx.vertx();
    vertx.deployVerticle(new VirtualRobotApplication());
  }
}
