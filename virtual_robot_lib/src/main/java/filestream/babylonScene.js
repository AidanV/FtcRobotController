var canvas = document.getElementById("renderCanvas");



var sceneObjects = {}
const robot = {}


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

    sceneObjects.bleft = BABYLON.Mesh.CreateBox("bleft", 0.5, scene)
    sceneObjects.bleft.position = new BABYLON.Vector3(-1, 0.5, -1)

    sceneObjects.fleft = BABYLON.Mesh.CreateBox("fleft", 0.5, scene)
    sceneObjects.fleft.position = new BABYLON.Vector3(-1, 0.5, 1)

    sceneObjects.bright = BABYLON.Mesh.CreateBox("bright", 0.5, scene)
    sceneObjects.bright.position = new BABYLON.Vector3(1, 0.5, -1)

    sceneObjects.fright = BABYLON.Mesh.CreateBox("fright", 0.5, scene)
    sceneObjects.fright.position = new BABYLON.Vector3(1, 0.5, 1)

    sceneObjects.driveTrain = BABYLON.Mesh.CreateBox("driveTrain", 0.5, scene)
    sceneObjects.driveTrain.position = new BABYLON.Vector3(0, 0.5, 0)


	var url;
	var fileName;


//-- BoomBox.gltf
	url = "https://raw.githubusercontent.com/BabylonJS/Babylon.js/master/Playground/scenes/BoomBox/";
	fileName = "BoomBox.gltf";

	BABYLON.SceneLoader.ImportMesh("", url, fileName, scene, function (newMeshes) {
		var mesh = newMeshes[0];
			mesh.position.copyFromFloats(0, 1, 0);
			mesh.scaling.copyFromFloats(100,100,100);

		camera.target = mesh;
		console.log(mesh)
	});
    return scene;
};

var engine = new BABYLON.Engine(canvas, true, { preserveDrawingBuffer: true, stencil: true });
var scene = createScene();

engine.runRenderLoop(function () {
	if (scene) {
		scene.render();
	}
	for(let r in robot){
	  robot[r].update(.1)
	}
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
      var avgHeading = 0//= obj.headingRadians + headingChange / 2.0;

      var sin = Math.sin(avgHeading);
      var cos = Math.cos(avgHeading);

      var dx = dxR * cos - dyR * sin
      var dy = dxR * sin + dyR * cos
      obj.position.x += dx
      obj.position.y += dy
      obj.headingRadians = 0//+= headingChange;

      obj.geometry.position.z = obj.position.x //= new BABYLON.Vector3(obj.position.x, 0.5, obj.position.y)
      obj.geometry.position.x = obj.position.y

      for(let i = 0; i < 4; i++){
        motors[i].geometry.position.z += dx;
        motors[i].geometry.position.x += dy;
      }
  //        if (x >  (halfFieldWidth - halfBotWidth)) x = halfFieldWidth - halfBotWidth;
  //        else if (x < (halfBotWidth - halfFieldWidth)) x = halfBotWidth - halfFieldWidth;
  //        if (y > (halfFieldWidth - halfBotWidth)) y = halfFieldWidth - halfBotWidth;
  //        else if (y < (halfBotWidth - halfFieldWidth)) y = halfBotWidth - halfFieldWidth;

      if (obj.headingRadians > Math.PI) obj.headingRadians -= 2.0 * Math.PI;
      else if (obj.headingRadians < -Math.PI) obj.headingRadians += 2.0 * Math.PI;
  }
  return obj
}

const motorFactory = (id, geometry) => {
  const obj = {}
  obj.id = id
  obj.inertia = 0.0
  obj.directionForward = true;
  obj.REVERSED = false
  obj.MAX_TICKS_PER_SECOND = 100; //TODO find max ticks
  obj.COEFF_PROPORTIONATE = 10; //TODO find proportionate
  obj.TICKS_PER_ROTATION = 384.5;
  obj.geometry = geometry
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
      obj.geometry.rotation.x  =  obj.internalState.actualPosition*2*Math.PI/obj.TICKS_PER_ROTATION;
//      return positionChange;

  }
  return obj
}


createRobot()
webSocketSetup(processWSData)
