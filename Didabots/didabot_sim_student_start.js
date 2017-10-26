/* Didabots - robot and GUI script.
 *
 * Copyright 2016 Harmen de Weerd
 * Copyright 2017 Johannes Keyser, James Cooke
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Description of robot(s), and attached sensor(s) used by InstantiateRobot()
RobotInfo = [
  {body: null,  // for MatterJS body, added by InstantiateRobot()
   color: "red",  // color of the robot marker
   init: {x: 50, y: 50, angle: 0},  // initial position and orientation
   sensors: [  // define an array of sensors on the robot
     // define right sensor
     {sense: senseDistance,  // function handle, determines type of sensor
      minVal: 0,  // minimum detectable distance, in pixels
      maxVal: 50,  // maximum detectable distance, in pixels
      attachAngle: Math.PI/4,  // where the sensor is mounted on robot body
      lookAngle: 0,  // direction the sensor is looking (relative to center-out)
      id: 'distR',  // a unique, arbitrary ID of the sensor, for printing/debugging
      parent: null,  // robot object the sensor is attached to, added by InstantiateRobot
      value: null  // sensor value, i.e. distance in pixels; updated by sense() function
     },
     // define left sensor
     {sense: senseDistance,
      minVal: 0,
      maxVal: 50,
      attachAngle: -Math.PI/4,
      lookAngle: 0,
      id: 'distL',
      parent: null,
      value: null}
   ]
  }
];

simInfo = {
  maxSteps: 20000,  // maximal number of simulation steps to run
  airDrag: 0.1,  // "air" friction of enviroment; 0 is vacuum, 0.9 is molasses
  boxFric: 0.005, //
  boxMass: 0.01,  // mass of boxes
  boxSize: 20,  // size of the boxes, in pixels
  robotSize: 2*7,  // robot radius, in pixels
  robotMass: 0.4, // robot mass (a.u)
  gravity: 0,  // constant acceleration in Y-direction
  bayRobot: null,  // currently selected robot
  baySensor: null,  // currently selected sensor
  bayScale: 3,  // scale within 2nd, inset canvas showing robot in it's "bay"
  doContinue: true,  // whether to continue simulation, set in HTML
  debugSensors: false,  // plot sensor rays and mark detected objects
  debugMouse: false,  // allow dragging any object with the mouse
  engine: null,  // MatterJS 2D physics engine
  world: null,  // world object (composite of all objects in MatterJS engine)
  runner: null,  // object for running MatterJS engine
  height: null,  // set in HTML file; height of arena (world canvas), in pixels
  width: null,  // set in HTML file; width of arena (world canvas), in pixels
  curSteps: 0  // increased by simStep()
};

robots = new Array();
sensors = new Array();

function init() {  // called once when loading HTML file
  const robotBay = document.getElementById("bayDidabot"),
        arena = document.getElementById("arenaDidabot"),
        height = arena.height,
        width = arena.width;
  simInfo.height = height;
  simInfo.width = width;

  /* Create a MatterJS engine and world. */
  simInfo.engine = Matter.Engine.create();
  simInfo.world = simInfo.engine.world;
  simInfo.world.gravity.y = simInfo.gravity;
  simInfo.engine.timing.timeScale = 1;

  /* Create walls and boxes, and add them to the world. */
  // note that "roles" are custom properties for rendering (not from MatterJS)
  function getWall(x, y, width, height) {
    return Matter.Bodies.rectangle(x, y, width, height,
                                   {isStatic: true, role: 'wall'});
  };
  const wall_lo = getWall(width/2, height-5, width-5, 5),
        wall_hi = getWall(width/2, 5, width-5, 5),
        wall_le = getWall(5, height/2, 5, height-15),
        wall_ri = getWall(width-5, height/2, 5, height-15);
  Matter.World.add(simInfo.world, [wall_lo, wall_hi, wall_le, wall_ri]);

  /* Add a bunch of boxes in a neat grid. */
  function getBox(x, y) {
    return Matter.Bodies.rectangle(x, y, simInfo.boxSize, simInfo.boxSize,
                                   {frictionAir: simInfo.airDrag,
                                    friction: simInfo.boxFric,
                                    mass: simInfo.boxMass,
                                    role: 'box'});
  };
  const startX = 100, startY = 100,
        nBoxX = 5, nBoxY = 5,
        gapX = 40, gapY = 30,
        stack = Matter.Composites.stack(startX, startY,
                                        nBoxX, nBoxY,
                                        gapX, gapY, getBox);
  Matter.World.add(simInfo.world, stack);

  /* Add debugging mouse control for dragging objects. */
  if (simInfo.debugMouse){
    const mouseConstraint = Matter.MouseConstraint.create(simInfo.engine,
                              {mouse: Matter.Mouse.create(arena),
                               // spring stiffness mouse ~ object
                               constraint: {stiffness: 0.5}});
    Matter.World.add(simInfo.world, mouseConstraint);
  }
  // Add the tracker functions from mouse.js
  addMouseTracker(arena);
  addMouseTracker(robotBay);

  /* Running the MatterJS physics engine (without rendering). */
  simInfo.runner = Matter.Runner.create({fps: 60,  // TODO: why weird effects?
                                         isFixed: false});
  Matter.Runner.start(simInfo.runner, simInfo.engine);
  // register function simStep() as callback to MatterJS's engine events
  Matter.Events.on(simInfo.engine, 'tick', simStep);

  /* Create robot(s). */
  setRobotNumber(1);  // requires defined simInfo.world
  loadBay(robots[0]);

};

function rotate(robot, torque=0) {
  /* Apply a torque to the robot to rotate it.
   *
   * Parameters
   *   torque - rotational force to apply to the body.
   */
  robot.body.torque = torque;
 };

function drive(robot, force=0) {
  /* Apply a force to the robot to move it.
   *
   * Parameters
   *   force - force to apply to the body.
   */
  const orientation = robot.body.angle,
        force_vec = Matter.Vector.create(force, 0),
        move_vec = Matter.Vector.rotate(force_vec, orientation);
  Matter.Body.applyForce(robot.body, robot.body.position , move_vec);
};


function senseDistance() {
  /* Distance sensor simulation based on ray casting. Called from sensor
   * object, returns nothing, updates a new reading into this.value.
   *
   * Idea: Cast a ray with a certain length from the sensor, and check
   *       via collision detection if objects intersect with the ray.
   *       To determine distance, run a Binary search on ray length.
   * Note: Sensor ray needs to ignore robot (parts), or start outside of it.
   *       The latter is easy with the current circular shape of the robots.
   * Note: Order of tests are optimized by starting with max ray length, and
   *       then only testing the maximal number of initially resulting objects.
   * Note: The sensor's "ray" could have any other (convex) shape;
   *       currently it's just a very thin rectangle.
   */

  const context = document.getElementById('arenaDidabot').getContext('2d');
  var bodies = Matter.Composite.allBodies(simInfo.engine.world);

  const robotAngle = this.parent.body.angle,
        attachAngle = this.attachAngle,
        rayAngle = robotAngle + attachAngle + this.lookAngle;

  const rPos = this.parent.body.position,
        rSize = simInfo.robotSize,
        startPoint = {x: rPos.x + (rSize+1) * Math.cos(robotAngle + attachAngle),
                      y: rPos.y + (rSize+1) * Math.sin(robotAngle + attachAngle)};

  function getEndpoint(rayLength) {
    return {x: rPos.x + (rSize + rayLength) * Math.cos(rayAngle),
            y: rPos.y + (rSize + rayLength) * Math.sin(rayAngle)};
  };

  function sensorRay(bodies, rayLength) {
    // Cast ray of supplied length and return the bodies that collide with it.
    const rayWidth = 1e-100,
          endPoint = getEndpoint(rayLength);
    rayX = (endPoint.x + startPoint.x) / 2,
    rayY = (endPoint.y + startPoint.y) / 2,
    rayRect = Matter.Bodies.rectangle(rayX, rayY, rayLength, rayWidth,
                                      {isSensor: true, isStatic: true,
                                       angle: rayAngle, role: 'sensor'});

    var collidedBodies = [];
    for (var bb = 0; bb < bodies.length; bb++) {
      var body = bodies[bb];
      // coarse check on body boundaries, to increase performance:
      if (Matter.Bounds.overlaps(body.bounds, rayRect.bounds)) {
        for (var pp = body.parts.length === 1 ? 0 : 1; pp < body.parts.length; pp++) {
          var part = body.parts[pp];
          // finer, more costly check on actual geometry:
          if (Matter.Bounds.overlaps(part.bounds, rayRect.bounds)) {
            const collision = Matter.SAT.collides(part, rayRect);
            if (collision.collided) {
              collidedBodies.push(body);
              break;
            }
          }
        }
      }
    }
    return collidedBodies;
  };

  // call 1x with full length, and check all bodies in the world;
  // in subsequent calls, only check the bodies resulting here
  var rayLength = this.maxVal;
  bodies = sensorRay(bodies, rayLength);

  // if some collided, search for maximal ray length without collisions
  if (bodies.length > 0) {
    var lo = 0,
        hi = rayLength;
    while (lo < rayLength) {
      if (sensorRay(bodies, rayLength).length > 0) {
        hi = rayLength;
      }
      else {
        lo = rayLength;
      }
      rayLength = Math.floor(lo + (hi-lo)/2);
    }
  }
  // increase length to (barely) touch closest body (if any)
  rayLength += 1;
  bodies = sensorRay(bodies, rayLength);

  if (simInfo.debugSensors) {  // if invisible, check order of object drawing
    // draw the resulting ray
    endPoint = getEndpoint(rayLength);
    context.beginPath();
    context.moveTo(startPoint.x, startPoint.y);
    context.lineTo(endPoint.x, endPoint.y);
    context.strokeStyle = this.parent.info.color;
    context.lineWidth = 0.5;
    context.stroke();
    // mark all objects's lines intersecting with the ray
    for (var bb = 0; bb < bodies.length; bb++) {
      var vertices = bodies[bb].vertices;
      context.moveTo(vertices[0].x, vertices[0].y);
      for (var vv = 1; vv < vertices.length; vv += 1) {
        context.lineTo(vertices[vv].x, vertices[vv].y);
      }
      context.closePath();
    }
    context.stroke();
  }

  // indicate if the sensor exceeded its maximum length by returning infinity
  if (rayLength > this.maxVal) {
    rayLength = Infinity;
  }
  else {
    // apply mild noise on the sensor reading, and clamp between valid values
    function gaussNoise(sigma=1) {
      const x0 = 1.0 - Math.random();
      const x1 = 1.0 - Math.random();
      return sigma * Math.sqrt(-2 * Math.log(x0)) * Math.cos(2 * Math.PI * x1);
    };
    rayLength = Math.floor(rayLength + gaussNoise(3));
    rayLength = Matter.Common.clamp(rayLength, this.minVal, this.maxVal);
  }

  this.value = rayLength;
};


function dragSensor(sensor, event) {
  const robotBay = document.getElementById('bayDidabot'),
        bCenter = {x: robotBay.width/2,
                   y: robotBay.height/2},
        rSize = simInfo.robotSize,
        bScale = simInfo.bayScale,
        sSize = sensor.getWidth(),
        mAngle = Math.atan2(  event.mouse.x - bCenter.x,
                            -(event.mouse.y - bCenter.y));
  sensor.info.attachAngle = mAngle;
  sensor.x = bCenter.x - sSize - bScale * rSize * Math.sin(-mAngle);
  sensor.y = bCenter.y - sSize - bScale * rSize * Math.cos( mAngle);
  repaintBay();
}

function loadSensor(sensor, event) {
  loadSensorInfo(sensor.sensor);
}

function loadSensorInfo(sensorInfo) {
  simInfo.baySensor = sensorInfo;
}

function loadBay(robot) {
  simInfo.bayRobot = robot;
  sensors = new Array();
  const robotBay = document.getElementById("bayDidabot");
  const bCenter = {x: robotBay.width/2,
                   y: robotBay.height/2},
        rSize = simInfo.robotSize,
        bScale = simInfo.bayScale;

  for (var ss = 0; ss < robot.info.sensors.length; ++ss) {
    const curSensor = robot.sensors[ss],
          attachAngle = curSensor.attachAngle;
    // put current sensor into global variable, make mouse-interactive
    sensors[ss] = makeInteractiveElement(new SensorGraphics(curSensor),
                                         document.getElementById("bayDidabot"));
    const sSize = sensors[ss].getWidth();
    sensors[ss].x = bCenter.x - sSize - bScale * rSize * Math.sin(-attachAngle);
    sensors[ss].y = bCenter.y - sSize - bScale * rSize * Math.cos( attachAngle);
    sensors[ss].onDragging = dragSensor;
    sensors[ss].onDrag = loadSensor;
  }
  repaintBay();
}

function SensorGraphics(sensorInfo) {
  this.info = sensorInfo;
  this.plotSensor = plotSensor;
  // add functions getWidth/getHeight for graphics.js & mouse.js,
  // to enable dragging the sensor in the robot bay
  this.getWidth = function() { return 6; };
  this.getHeight = function() { return 6; };
}

function InstantiateRobot(robotInfo) {
  // create robot's main physical body (simulated with MatterJS engine)
  const nSides = 20,
        circle = Matter.Bodies.circle;
  this.body = circle(robotInfo.init.x, robotInfo.init.y, simInfo.robotSize,
                     {frictionAir: simInfo.airDrag,
                       mass: simInfo.robotMass,
                       role: 'robot'}, nSides);
  Matter.World.add(simInfo.world, this.body);
  Matter.Body.setAngle(this.body, robotInfo.init.angle);

  // instantiate its sensors
  this.sensors = robotInfo.sensors;
  for (var ss = 0; ss < this.sensors.length; ++ss) {
    this.sensors[ss].parent = this;
  }

  // attach its helper functions
  this.rotate = rotate;
  this.drive = drive;
  this.info = robotInfo;
  this.plotRobot = plotRobot;

  // add functions getWidth/getHeight for graphics.js & mouse.js,
  // to enable selection by clicking the robot in the arena
  this.getWidth = function() { return 2 * simInfo.robotSize; };
  this.getHeight = function() { return 2 * simInfo.robotSize; };
}

function robotUpdateSensors(robot) {
  // update all sensors of robot; puts new values into sensor.value
  for (var ss = 0; ss < robot.sensors.length; ss++) {
    robot.sensors[ss].sense();
  }
};

function getSensorValById(robot, id) {
  for (var ss = 0; ss < robot.sensors.length; ss++) {
    if (robot.sensors[ss].id == id) {
      return robot.sensors[ss].value;
    }
  }
  return undefined;  // if not returned yet, id doesn't exist
};

function robotMove(robot) {
// This function is called each timestep and should be used to move the robots

};

function plotSensor(context, x = this.x, y = this.y) {
  context.beginPath();
  context.arc(x + this.getWidth()/2,
              y + this.getHeight()/2,
              this.getWidth()/2, 0, 2*Math.PI);
  context.closePath();
  context.fillStyle = 'black';
  context.strokeStyle = 'black';
  context.fill();
  context.stroke();
}

function plotRobot(context,
                     xTopLeft = this.body.position.x,
                     yTopLeft = this.body.position.y) {
  var x, y, scale, angle, i, half, full,
      rSize = simInfo.robotSize;

  if (context.canvas.id == "bayDidabot") {
    scale = simInfo.bayScale;
    half = Math.floor(rSize/2*scale);
    full = half * 2;
    x = xTopLeft + full;
    y = yTopLeft + full;
    angle = -Math.PI / 2;
  } else {
    scale = 1;
    half = Math.floor(rSize/2*scale);
    full = half * 2;
    x = xTopLeft;
    y = yTopLeft;
    angle = this.body.angle;
  }
  context.save();
  context.translate(x, y);
  context.rotate(angle);

  // Plot wheels as rectangles.
  context.strokeStyle = "black";
  if (context.canvas.id == "bayDidabot") {
    context.fillStyle = "grey";
    context.fillRect(-half, -full, full, full);
    context.fillRect(-half, 0, full, full);
  } else {
    context.fillStyle = "grey";
    context.fillRect(-half, -full, full, 2*full);
  }
  context.strokeRect(-half, -full, full, 2*full);

  // Plot circular base object.
  if (context.canvas.id == "bayDidabot") {
    context.beginPath();
    context.arc(0, 0, full, 0, 2*Math.PI);
    context.closePath();
    context.fillStyle = "lightgrey";
    context.fill();
    context.stroke();
  }
  else { // context.canvas.id == "arenaDidabot"
    // draw into world canvas without transformations,
    // because MatterJS thinks in world coords...
    context.restore();
    context.beginPath();
    var vertices = this.body.vertices;
    context.moveTo(vertices[0].x, vertices[0].y);
    for (var vv = 1; vv < vertices.length; vv += 1) {
      context.lineTo(vertices[vv].x, vertices[vv].y);
    }
    context.closePath();
    context.fillStyle = 'lightgrey';
    context.stroke();
    context.fill();
    // to draw the rest, rotate & translate again
    context.save();
    context.translate(x, y);
    context.rotate(angle);
  }

  // Plot a marker to distinguish robots and their orientation.
  context.beginPath();
  context.arc(0, 0, full * .4, -Math.PI/4, Math.PI/4);
  context.lineTo(full * Math.cos(Math.PI/4) * .8,
             full * Math.sin(Math.PI/4) * .8);
  context.arc(0, 0, full * .8, Math.PI/4, -Math.PI/4, true);
  context.closePath();
  context.fillStyle = this.info.color;
  context.fill();
  context.stroke();

  // Plot sensor positions into world canvas.
  if (context.canvas.id == "arenaDidabot") {
    for (ss = 0; ss < this.info.sensors.length; ++ss) {
      context.beginPath();
      context.arc(full * Math.cos(this.info.sensors[ss].attachAngle),
                  full * Math.sin(this.info.sensors[ss].attachAngle),
                  scale, 0, 2*Math.PI);
      context.closePath();
      context.fillStyle = 'black';
      context.strokeStyle = 'black';
      context.fill();
      context.stroke();
    }
  }
  context.restore();
}

function simStep() {
  // advance simulation by one step (except MatterJS engine's physics)
  if (simInfo.curSteps < simInfo.maxSteps) {
    repaintBay();
    drawBoard();
    for (var rr = 0; rr < robots.length; ++rr) {
      robotUpdateSensors(robots[rr]);
      robotMove(robots[rr]);
      // To enable selection by clicking (via mouse.js/graphics.js),
      // the position on the canvas needs to be defined in (x, y):
      const rSize = simInfo.robotSize;
      robots[rr].x = robots[rr].body.position.x - rSize;
      robots[rr].y = robots[rr].body.position.y - rSize;
    }
    // count and display number of steps
    simInfo.curSteps += 1;
    document.getElementById("SimStepLabel").innerHTML =
      padnumber(simInfo.curSteps, 5) +
      ' of ' +
      padnumber(simInfo.maxSteps, 5);
  }
  else {
    toggleSimulation();
  }
}

function drawBoard() {
  var context = document.getElementById('arenaDidabot').getContext('2d');
  context.fillStyle = "#444444";
  context.fillRect(0, 0, simInfo.width, simInfo.height);

  // draw objects within world
  const Composite = Matter.Composite,
        bodies = Composite.allBodies(simInfo.world);
  context.beginPath();
  for (var bb = 0; bb < bodies.length; bb += 1) {
    if (bodies[bb].role == 'robot') {
      // don't draw robot's circles; they're drawn below
      continue;
    }
    else {
      var vertices = bodies[bb].vertices;
      context.moveTo(vertices[0].x, vertices[0].y);
      for (var vv = 1; vv < vertices.length; vv += 1) {
        context.lineTo(vertices[vv].x, vertices[vv].y);
      }
      context.closePath();
    }
  }
  context.lineWidth = 1;
  context.strokeStyle = '#999';
  context.fillStyle = '#5559';
  context.fill();
  context.stroke();

  // draw all robots
  for (var rr = 0; rr < robots.length; ++rr) {
    robots[rr].plotRobot(context);
  }
}

function repaintBay() {
  // update inset canvas showing information about selected robot
  const robotBay = document.getElementById('bayDidabot'),
        context = robotBay.getContext('2d');
  context.clearRect(0, 0, robotBay.width, robotBay.height);
  simInfo.bayRobot.plotRobot(context, 10, 10);
  for (var ss = 0; ss < sensors.length; ss++) {
    sensors[ss].plotSensor(context);
  }

  // print sensor values of selected robot next to canvas
  if (!(simInfo.curSteps % 5)) {  // update slow enough to read
    var sensorString = '';
    const rsensors = simInfo.bayRobot.sensors;
    for (ss = 0; ss < rsensors.length; ss++) {
      sensorString += '<br> id \'' + rsensors[ss].id + '\': ' +
        padnumber(rsensors[ss].value, 2);
    }
    document.getElementById('SensorLabel').innerHTML = sensorString;
  }
}

function setRobotNumber(newValue) {
  var n;
  while (robots.length > newValue) {
    n = robots.length - 1;
    Matter.World.remove(simInfo.world, robots[n].body);
    robots[n] = null;
    robots.length = n;
  }

  while (robots.length < newValue) {
    if (newValue > RobotInfo.length) {
      console.warn('You request '+newValue+' robots, but only ' + RobotInfo.length +
                   ' are defined in RobotInfo!');
      toggleSimulation();
      return;
    }
    n = robots.length;
    robots[n] = makeInteractiveElement(new InstantiateRobot(RobotInfo[n]),
                                       document.getElementById("arenaDidabot"));

    robots[n].onDrop = function(robot, event) {
      robot.isDragged = false;
    };

    robots[n].onDrag = function(robot, event) {
      	robot.isDragged = true;
        loadBay(robot);
        return true;
    };
  }
}


function padnumber(number, size) {
  if (number == Infinity) {
    return 'inf';
  }
  const s = "000000" + number;
  return s.substr(s.length - size);
}

function format(number) {
  // prevent HTML elements to jump around at sign flips etc
  return (number >= 0 ? '+' : '−') + Math.abs(number).toFixed(1);
}

function toggleSimulation() {
  simInfo.doContinue = !simInfo.doContinue;
  if (simInfo.doContinue) {
    Matter.Runner.start(simInfo.runner, simInfo.engine);
  }
  else {
    Matter.Runner.stop(simInfo.runner);
  }
}
