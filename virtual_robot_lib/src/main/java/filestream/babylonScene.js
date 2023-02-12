var canvas = document.getElementById("renderCanvas");



var sceneObjects = {}
const robot = {}
var isStarted = false


var createScene = function () {
    var scene = new BABYLON.Scene(engine);
//    JSONObject data = JSON.parse({
//                                 "employees":[
//                                   {"firstName":"John", "lastName":"Doe"},
//                                   {"firstName":"Anna", "lastName":"Smith"},
//                                   {"firstName":"Peter", "lastName":"Jones"}
//                                 ]
//                                 });

    var camera = new BABYLON.FreeCamera("camera1", new BABYLON.Vector3(0, 5, -10), scene);
		camera.setTarget(BABYLON.Vector3.Zero());
		camera.attachControl(canvas, true);
    var light = new BABYLON.HemisphericLight("light1", new BABYLON.Vector3(0, 1, 0), scene);
		light.intensity = 0.7;
    var ground = BABYLON.Mesh.CreateGround("ground1", 6, 6, 2, scene);

    sceneObjects.driveTrain = BABYLON.Mesh.CreateBox("driveTrain", 0.5, scene)
    sceneObjects.driveTrain.position = new BABYLON.Vector3(0, 0.5, 0)
//    sceneObjects.driveTrain.isVisible = false

    sceneObjects.bleft = BABYLON.Mesh.CreateBox("bleft", 0.5, scene)
    sceneObjects.bleft.position = new BABYLON.Vector3(-1, 0.0, -1)
//    sceneObjects.bleft.isVisible = false


    sceneObjects.fleft = BABYLON.Mesh.CreateBox("fleft", 0.5, scene)
    sceneObjects.fleft.position = new BABYLON.Vector3(-1, 0.0, 1)
//    sceneObjects.fleft.isVisible = false


    sceneObjects.bright = BABYLON.Mesh.CreateBox("bright", 0.5, scene)
    sceneObjects.bright.position = new BABYLON.Vector3(1, 0.0, -1)
//    sceneObjects.bright.isVisible = false


    sceneObjects.fright = BABYLON.Mesh.CreateBox("fright", 0.5, scene)
    sceneObjects.fright.position = new BABYLON.Vector3(1, 0.0, 1)
//    sceneObjects.fright.isVisible = false


//BABYLON.SceneLoader.ImportMesh("", './', "StrippedChassis.glb", scene, group => {
//    group.forEach((mesh, i) => {
//        if (mesh.name !== '__root__') {
//            console.log(mesh)
//            sceneObjects[mesh.name] = mesh;
////            mesh.name = 'FIELD_BLACK';
////            let blackMaterial = new StandardMaterial("blackField", scene);
////            blackMaterial.diffuseColor = new Color3.FromHexString("#3E211B");
////            mesh.material = blackMaterial;
////            mesh.isVisible = false;
//        }
//    })
//    });

//    BABYLON.SceneLoader.AppendAsync("/", "StrippedChassis.glb", scene);
//    scene = await BABYLON.SceneLoader.AppendAsync("./", "StrippedChassis.glb")
//    console.log(scene.getNodeByName("__root__"))
//    sceneObjects.root = scene.getNodeByName("__root__")
//    sceneObjects.root.rotation = new BABYLON.Vector3(Math.PI/2, 0, 0)
//    sceneObjects.root.scaling = new BABYLON.Vector3(4, 4, 4);
////        scene.getNodeByName("__root__").getChildren()[0].getChildren()[0]
//    sceneObjects.bright = scene.getNodeByName("__root__").getChildren()[0].getChildren()[0]
////        sceneObjects.root.rotation.z = 180
////        sceneObjects.root.position.x = 10
//    sceneObjects.driveTrain = sceneObjects.root//scene.getNodeByName("__root__").getChildren()[0].getChildren()[1]
//
//    createRobot()




//
//    sceneObjects.chassis = BABYLON.SceneLoader.Append("/", "StrippedChassis.glb", scene);
//    sceneObjects.chassis.rotation.x = Math.PI/2.0;

    return scene;
};

var engine = new BABYLON.Engine(canvas, true, { preserveDrawingBuffer: true, stencil: true });
var scene = null

engine.runRenderLoop(function () {
  document.getElementById('startButton').onclick = () => {
     isStarted = true
  }
	if (scene) {
		scene.render();

    for(let r in robot){
      robot[r].update(.1)//TODO make actual frame time
    }
    console.log(robot.bright.geometry.position)
    sendWebSocket(JSON.stringify({
    "isStarted":isStarted,
    "encoders":{
      "bleftPosition":(robot.bleft.internalState.actualPosition - robot.bleft.internalState.basePosition),
      "fleftPosition":(robot.fleft.internalState.actualPosition - robot.fleft.internalState.basePosition),
      "brightPosition":(robot.bright.internalState.actualPosition - robot.bright.internalState.basePosition),
      "frightPosition":(robot.fright.internalState.actualPosition - robot.fright.internalState.basePosition)
    }
    }))
	}
//  if(scene.getNodeByName("__root__")!= null){
//  console.log(scene.getNodeByName("__root__").getChildren()[0].getChildren())
//  }
});

// Resize
window.addEventListener("resize", function () {
	engine.resize();
});

const processWSData = (data) => {
  if(data["objectType"] === "motor"){
    robot[data["instance"]].processCommand(data["data"])
  }

}








const createRobot = () => {

  robot.bleft = motorFactory("bleft", sceneObjects.bleft)
  robot.fleft = motorFactory("fleft", sceneObjects.fleft)
  robot.bright = motorFactory("bright", sceneObjects.bright)
  robot.fright = motorFactory("fright", sceneObjects.fright)
  robot.driveTrain = driveTrainFactory(sceneObjects.driveTrain, [robot.bleft, robot.fleft, robot.bright, robot.fright])
}

const driveTrainFactory = (geometry, motors) => {
  const obj = {}
  obj.wheelCircumference = 4.0
  obj.position = {
    Vector3: new BABYLON.Vector3.Zero(),
    x: 0.0,
    y: 0.0
  }
  obj.geometry = geometry
  obj.headingRadians = 0
  obj.wlAverage = 2.0
  obj.tWR = [
    [0.25, 0.25, 0.25, 0.25],
    [-0.25, 0.25, -0.25, 0.25],
    [-0.25/ obj.wlAverage, -0.25/ obj.wlAverage, 0.25/ obj.wlAverage, 0.25/ obj.wlAverage],
    [-0.25, 0.25, 0.25, -0.25]]

//      obj.tWR = [
//        [-0.25, 0.25, -0.25, 0.25],
//        [0.25, 0.25, 0.25, 0.25],
//        [-0.25/ obj.wlAverage, -0.25/ obj.wlAverage, 0.25/ obj.wlAverage, 0.25/ obj.wlAverage],
//        [-0.25, 0.25, 0.25, -0.25]]

  obj.update = (frameTime) => {
      var deltaPos = {};
      var w = {};

      for (let i = 0; i < 4; i++) {
          deltaPos[i] = motors[i].internalState.velocity/frameTime;
          w[i] = deltaPos[i] * obj.wheelCircumference / motors[i].TICKS_PER_ROTATION;
//          if (i < 2) w[i] = -w[i];
      }

      var robotDeltaPos = [0.0,0.0,0.0,0.0]
      for (let i=0; i<4; i++){
          for (let j = 0; j<4; j++){
              robotDeltaPos[i] += obj.tWR[i][j] * w[j];
          }
      }

      var dxR = robotDeltaPos[0];
      var dyR = robotDeltaPos[1];
      var headingChange = robotDeltaPos[2];
      var avgHeading = obj.headingRadians + headingChange / 2.0;

      var sin = Math.sin(avgHeading);
      var cos = Math.cos(avgHeading);

      var dx = dxR * cos - dyR * sin
      var dy = dxR * sin + dyR * cos
      obj.position.x += dx
      obj.position.y += dy
      obj.headingRadians += headingChange

      obj.geometry.position.z = obj.position.x //= new BABYLON.Vector3(obj.position.x, 0.5, obj.position.y)
      obj.geometry.position.x = obj.position.y

//      for(let i = 0; i < 4; i++){
//        motors[i].geometry.position.z += dx;
//        motors[i].geometry.position.x += dy;
//      }
  //        if (x >  (halfFieldWidth - halfBotWidth)) x = halfFieldWidth - halfBotWidth;
  //        else if (x < (halfBotWidth - halfFieldWidth)) x = halfBotWidth - halfFieldWidth;
  //        if (y > (halfFieldWidth - halfBotWidth)) y = halfFieldWidth - halfBotWidth;
  //        else if (y < (halfBotWidth - halfFieldWidth)) y = halfBotWidth - halfFieldWidth;

      if (obj.headingRadians > Math.PI) obj.headingRadians -= 2.0 * Math.PI;
      else if (obj.headingRadians < -Math.PI) obj.headingRadians += 2.0 * Math.PI;

      obj.geometry.rotation.y = -obj.headingRadians

  }
  return obj
}

const motorFactory = (id, geometry) => {
  const obj = {}
  obj.id = id
  obj.inertia = 0//.95
  obj.directionForward = true;
  obj.REVERSED = false
  obj.MAX_TICKS_PER_SECOND = 100; //TODO find max ticks
  obj.COEFF_PROPORTIONATE = 10; //TODO find proportionate
  obj.TICKS_PER_ROTATION = 384.5;
  obj.geometry = geometry
  obj.geometry.parent = sceneObjects.driveTrain
  obj.internalState = {
    actualPosition: 0,
    velocity: 0.0,
    basePosition: 0,
    positionChange: 0
  }

  obj.controls = {
    power: 0.0,
    mode: "RUN_WITHOUT_ENCODERS"
  }

  obj.processCommand = (command) => {
    obj.controls.power = command["power"]
    obj.controls.mode = command["mode"]
  }

  obj.update = (frameTime) => {
      if (obj.controls.mode === "STOP_AND_RESET_ENCODER") return 0.0;
      else if (obj.controls.mode === "RUN_TO_POSITION"){
        console.log("RUN_TO_POSITION not implemented!")
      //TODO add RUN_TO_POSITION support
//          var targetSpeed = COEFF_PROPORTIONATE * (double)(targetPosition - getCurrentPosition())
//                  / obj.MAX_TICKS_PER_SECOND;
//          var absPower = Math.abs(power);
//          targetSpeed = Math.max(-absPower, Math.min(targetSpeed, absPower));
//          speed = speed + (1.0 - inertia) * (targetSpeed - speed);
      } else {
          obj.internalState.velocity = obj.internalState.velocity + (1.0 - obj.inertia) * (obj.controls.power - obj.internalState.velocity);
      }
      obj.internalState.positionChange = obj.internalState.velocity * obj.MAX_TICKS_PER_SECOND * frameTime;
//      positionChange *= (1.0 + systematicErrorFrac + randomErrorFrac * random.nextGaussian());
      if (obj.directionForward && obj.REVERSED ||
              !obj.directionForward && !obj.REVERSED) obj.internalState.positionChange = -obj.internalState.positionChange;
      obj.internalState.actualPosition += obj.internalState.positionChange;
//      obj.geometry.setPivotMatrix(BABYLON.Matrix.Translation(1))
//      obj.geometry.rotate(new BABYLON.Vector3(1,0,0)/*BABYLON.Axis.Y*/, Math.PI / 2, /*BABYLON.Space.LOCAL*/new BABYLON.Vector3(100,100,100));
      obj.geometry.rotation = new BABYLON.Vector3(obj.internalState.actualPosition*2*Math.PI/obj.TICKS_PER_ROTATION, 0, 0);
//      return positionChange;

  }
  return obj
}



  const sendWebSocket = webSocketSetup(processWSData, async () => {
    scene = await createScene()
    await createRobot()


  })




