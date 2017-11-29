/* Lemmings - robot and GUI script.
 *
 * Copyright 2016 Harmen de Weerd
 * Copyright 2017 Johannes Keyser, James Cooke, George Kachergis
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
  // First Robot.
  {body: null,  // for MatterJS body, added by InstantiateRobot()
   color: [255, 0, 0],  // color of the robot shape
   init: {x: 50, y: 50, angle: 0},  // initial position and orientation
   sensors: [  // define an array of sensors on the robot
     // define one distance sensor
     {sense: senseDistance, minVal: 0, maxVal: 50, attachAngle: Math.PI/30, 
      attachRadius: 27, lookAngle: 0, id: 'frontS_dist_all', color: [0, 0, 0, 50], 
      parent: null, value: null, valueStr: '' 
     },
     // define a gyroscope/angle sensor
     {sense: senseRobotAngle, id: 'gyro', parent: null, value: null, valueStr: ''},
     // define one color sensor
     {sense: senseColor, minVal: 0, maxVal: 15, attachAngle: -0.5585993153435624 ,
      attachRadius: 5, lookAngle: 71.003, id: 'color_gripper', color: [0, 255, 0, 50],
      parent: null, value: null, valueStr: ''
     },
	 // define one color sensor
	 {sense: senseColor, minVal: 0, maxVal: 50, attachAngle: Math.PI/30,
      attachRadius: 27, lookAngle: 0, id: 'frontS_color_all', color: [0, 255, 0, 50],
      parent: null, value: null, valueStr: ''
     }
   ]
  }, 

  // Second Robot.
  {body: null,  // for MatterJS body, added by InstantiateRobot()
   color: [0, 255, 0],  // color of the robot shape
   init: {x: 200, y: 50, angle: Math.PI/2},  // initial position and orientation
   sensors: [  // define an array of sensors on the robot
     // define one distance sensor
     {sense: senseDistance, minVal: 0, maxVal: 50, attachAngle: Math.PI/30, 
      attachRadius: 27, lookAngle: 0, id: 'frontS_dist_all', color: [0, 0, 0, 50], 
      parent: null, value: null, valueStr: '' 
     },
     // define a gyroscope/angle sensor
     {sense: senseRobotAngle, id: 'gyro', parent: null, value: null, valueStr: ''},
     // define one color sensor
     {sense: senseColor, minVal: 0, maxVal: 15, attachAngle: -0.5585993153435624 ,
      attachRadius: 5, lookAngle: 70.003, id: 'color_gripper', color: [0, 255, 0, 50],
      parent: null, value: null, valueStr: ''
     },
	 // define one color sensor
	 {sense: senseColor, minVal: 0, maxVal: 50, attachAngle: Math.PI/30,
      attachRadius: 27, lookAngle: 0, id: 'frontS_color_all', color: [0, 255, 0, 50],
      parent: null, value: null, valueStr: ''
     }
   ]
  }, 
  // Third Robot.
  {body: null,  // for MatterJS body, added by InstantiateRobot()
   color: [0, 0, 255],  // color of the robot shape
   init: {x: 400, y: 400, angle: -Math.PI},  // initial position and orientation
   sensors: [  // define an array of sensors on the robot
     // define one distance sensor
     {sense: senseDistance, minVal: 0, maxVal: 50, attachAngle: Math.PI/30, 
      attachRadius: 27, lookAngle: 0, id: 'frontS_dist_all', color: [0, 0, 0, 50], 
      parent: null, value: null, valueStr: '' 
     },
     // define a gyroscope/angle sensor
     {sense: senseRobotAngle, id: 'gyro', parent: null, value: null, valueStr: ''},
     // define one color sensor
     {sense: senseColor, minVal: 0, maxVal: 15, attachAngle: -0.5585993153435624 ,
      attachRadius: 5, lookAngle: 70.003, id: 'color_gripper', color: [0, 255, 0, 50],
      parent: null, value: null, valueStr: ''
     },
	 // define one color sensor
	 {sense: senseColor, minVal: 0, maxVal: 50, attachAngle: Math.PI/30,
      attachRadius: 27, lookAngle: 0, id: 'frontS_color_all', color: [0, 255, 0, 50],
      parent: null, value: null, valueStr: ''
     }
   ]
  }
];

// Simulation settings; please change anything that you think makes sense.
simInfo = {
  numRobots: 3,
  maxSteps: 50000,  // maximal number of simulation steps to run
  airDrag: 0.1,  // "air" friction of environment; 0 is vacuum, 0.9 is molasses
  boxFric: 0.005, // friction between boxes during collisions
  boxMass: 0.01,  // mass of boxes
  boxSize: 10,  // size of the boxes, in pixels
  robotSize: 15, // approximate robot radius, to select by clicking with mouse
  robotMass: 0.4, // robot mass (a.u)
  gravity: 0,  // constant acceleration in Y-direction
  bayRobot: null,  // currently selected robot
  baySensor: null,  // currently selected sensor
  bayScale: 3,  // scale within 2nd, inset canvas showing robot in it's "bay"
  doContinue: true,  // whether to continue simulation, set in HTML
  debugSensors: true,  // plot sensor rays and mark detected objects
  debugMouse: true,  // allow dragging any object with the mouse
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
  const robotBay = document.getElementById("bayLemming"),
        arena = document.getElementById("arenaLemming"),
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
                                   {isStatic: true, role: 'wall',
                                    color:[150, 150, 150]});
  };
  const wall_lo = getWall(width/2, height-5, width-5, 5),
        wall_hi = getWall(width/2, 5, width-5, 5),
        wall_le = getWall(5, height/2, 5, height-15),
        wall_ri = getWall(width-5, height/2, 5, height-15);
  Matter.World.add(simInfo.world, [wall_lo, wall_hi, wall_le, wall_ri]);

/* Add a bunch of boxes in a neat grid. */
  var nmbr = 1; 
 
  function getBox(x, y) {
    // flip coin for red vs blue and add rgb
    nmbr = nmbr+1;  
    if (nmbr % 2 == 0){
      color = [0, 0, 200];
    }
    else {
      color = [200, 0, 0];
    }
    box = Matter.Bodies.rectangle(x, y, simInfo.boxSize, simInfo.boxSize,
                                  {frictionAir: simInfo.airDrag,
                                   friction: simInfo.boxFric,
                                   mass: simInfo.boxMass,
                                   role: 'box',
                                   color: color});
    return box;
  };

  const startX = 60, startY = 60,
        nBoxX = 7, nBoxY = 7,
        gapX = 40, gapY = 40,
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
  simInfo.runner = Matter.Runner.create({fps: 60, isFixed: false});
  Matter.Runner.start(simInfo.runner, simInfo.engine);
  // register function simStep() as callback to MatterJS's engine events
  Matter.Events.on(simInfo.engine, 'tick', simStep);

  /* Create robot(s). */
  setRobotNumber(simInfo.numRobots);  // requires defined simInfo.world
  loadBay(robots[simInfo.numRobots-1]);

};

function convrgb(values) {
  return 'rgb(' + values.join(', ') + ')';
};


function rotate(robot, torque=0) {
  /* Apply a torque to the robot to rotate it.
   *
   * Parameters
   *   torque - rotational force to apply to the body.
   *            Try values around +/- 0.005.
   *            Don't pass Infinity, or robot will leave the arena.
   */
  robot.body.torque = torque;
 };

function drive(robot, force=0) {
  /* Apply a force to the robot to move it.
   *
   * Parameters
   *   force - force to apply to the body.
   *           Try values around +/- 0.0005.
   *           Don't pass Infinity, or robot will leave the arena.
   */
  const orientation = robot.body.angle,
        force_vec = Matter.Vector.create(force, 0),
        move_vec = Matter.Vector.rotate(force_vec, orientation);
  Matter.Body.applyForce(robot.body, robot.body.position , move_vec);
};

function senseDistance_noBox(sensor) {
  senseDistance(sensor, true);
}

function senseDistance(sensor, noBoxes=false) {
  /* Distance sensor simulation based on ray casting. Called from sensor
   * object, returns nothing, updates a new reading into sensor.value.
   *
   * Parameter noBoxes - whether to ignore boxes in the world.
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

  const context = document.getElementById('arenaLemming').getContext('2d');
  var bodies = Matter.Composite.allBodies(simInfo.engine.world),
      bb;

  if (noBoxes) {
    const bodies_noBoxes = [];
    for (bb = 0; bb < bodies.length; bb++) {
      if (bodies[bb].role != 'box') {
        bodies_noBoxes.push(bodies[bb]);
      }
    }
    bodies = bodies_noBoxes;
  }

  const robotAngle = sensor.parent.body.angle,
        attachAngle = sensor.attachAngle,
        attachRadius = sensor.attachRadius,
        rayAngle = robotAngle + attachAngle + sensor.lookAngle;

  const rPos = sensor.parent.body.position,
        startPoint = {x: rPos.x + attachRadius*Math.cos(robotAngle+attachAngle),
                      y: rPos.y + attachRadius*Math.sin(robotAngle+attachAngle)};

  function getEndpoint(rayLength) {
    return {x: rPos.x + (attachRadius + rayLength) * Math.cos(rayAngle),
            y: rPos.y + (attachRadius + rayLength) * Math.sin(rayAngle)};
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
  var rayLength = sensor.maxVal;
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
    context.strokeStyle = convrgb(sensor.color);
    context.lineWidth = 1.0;
    context.stroke();
    // mark all objects's lines intersecting with the ray
    for (bb = 0; bb < bodies.length; bb++) {
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
  if (rayLength > sensor.maxVal) {
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
    rayLength = Matter.Common.clamp(rayLength, sensor.minVal, sensor.maxVal);
  }

  sensor.value = rayLength;  // for using in program, e.g. robotMove()
  sensor.valueStr = padnumber(rayLength, 2);  // for printing to screen
};

function senseColor_noBox(sensor) {
  senseColor(sensor,true);
}

function senseColor(sensor, noBoxes=false) {
  /* Color sensor simulation based on ray casting. Called from sensor
   * object, returns nothing, updates a new reading into sensor.value.
   *
   * Parameter noBoxes - whether to ignore boxes in the world.
   */
   
  const context = document.getElementById('arenaLemming').getContext('2d');
  var bodies = Matter.Composite.allBodies(simInfo.engine.world),
      bb;

  if (noBoxes) {
    const bodies_noBoxes = [];
    for (bb = 0; bb < bodies.length; bb++) {
      if (bodies[bb].role != 'box') {
        bodies_noBoxes.push(bodies[bb]);
      }
    }
    bodies = bodies_noBoxes;
  }

  const robotAngle = sensor.parent.body.angle,
        attachAngle = sensor.attachAngle,
        attachRadius = sensor.attachRadius,
        rayAngle = robotAngle + attachAngle + sensor.lookAngle;

  const rPos = sensor.parent.body.position,
        startPoint = {x: rPos.x + attachRadius*Math.cos(robotAngle+attachAngle),
                      y: rPos.y + attachRadius*Math.sin(robotAngle+attachAngle)};

  function getEndpoint(rayLength) {
    return {x: rPos.x + (attachRadius + rayLength) * Math.cos(rayAngle),
            y: rPos.y + (attachRadius + rayLength) * Math.sin(rayAngle)};
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
  var rayLength = sensor.maxVal;
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
    context.strokeStyle = convrgb(sensor.color);
    context.lineWidth = 1.0;
    context.stroke();
    // mark all objects's lines intersecting with the ray
    for (bb = 0; bb < bodies.length; bb++) {
      var vertices = bodies[bb].vertices;
      context.moveTo(vertices[0].x, vertices[0].y);
      for (var vv = 1; vv < vertices.length; vv += 1) {
        context.lineTo(vertices[vv].x, vertices[vv].y);
      }
      context.closePath();
    }
    context.stroke();
  }

var color = [0,0,0];
  // indicate if there is a body in front of the robot
  if (bodies.length > 0) {
    var body = bodies[0];
	// apply mild noise on the sensor reading, and clamp between valid values
    function gaussNoise(sigma=1) {
      const x0 = 1.0 - Math.random();
      const x1 = 1.0 - Math.random();
      return sigma * Math.sqrt(-2 * Math.log(x0)) * Math.cos(2 * Math.PI * x1);
    };
	for(var i = 0; i < 3; i++) {
		color[i] = Math.floor(body.color[i] + gaussNoise(3));
	};
	function between(value, minValue, maxValue) {
		return value >= minValue && value <= maxValue;
	}
	function isWhite(color) {
		return between(color[0], 140,160) && between(color[1],140,160) && between(color[2],140,160);
	}
	function isRed(color) {
		return between(color[0],190,210) && between(color[1],-10,10) && between(color[2],-10,10);
	}
	function isBlue(color) {
		return between(color[0],-10,10) && between(color[1],-10,10) && between(color[2],190,210);
	}
	if(isWhite(color)){
		this.value = "w";
	}
	else if (isRed(color)) {
		this.value = "r";
	}
	else if (isBlue(color)) {
		this.value = "b";
	}
	else {
		this.value = "n";
	}
  }
  else {
	  // no bodies in front of the sensor, return nothing.
	  this.value = "n";
  }
  sensor.valueStr = this.value;  // for printing to screen
};

function rad2deg(x) {
  return 180 * x / Math.PI;
};

function deg2rad(x) {
  return Math.PI * x / 180;
};

function boundRadians(x) {
  return x % (2*Math.PI);
};

function senseRobotAngle(sensor) {
  /* Gyroscope sensor based on real angle, compromised with noise. Called with
   * sensor object, returns nothing, updates a new reading into sensor.value.
   *
   * Note that the Radian sensor values are not bounded; it simplifies some
   * operations, like rotating a given amount, and MatterJS is fine with it.
   * However, to print the values in an understandable format, they are bounded
   * between -2*PI and +2*PI, and converted to degrees.
   */

  const realRobotAngle = sensor.parent.body.angle;  // angle in radians

  // add some noise to the actual values to simulate an unbiased gyroscope
  function gaussNoise(sigma=Math.PI/45) {
    const x0 = 1.0 - Math.random();
    const x1 = 1.0 - Math.random();
    return sigma * Math.sqrt(-2 * Math.log(x0)) * Math.cos(2 * Math.PI * x1);
  };

  const noisyRobotAngle = realRobotAngle + gaussNoise();
  sensor.value = noisyRobotAngle;
  sensor.valueStr = format(rad2deg(boundRadians(noisyRobotAngle)));
};


function dragSensor(sensor, event) {
  const robotBay = document.getElementById('bayLemming'),
        bCenter = {x: robotBay.width/2,
                   y: robotBay.height/2},
        attachRadius = sensor.info.attachRadius,
        bScale = simInfo.bayScale,
        sSize = sensor.getWidth(),
        mAngle = Math.atan2(  event.mouse.x - bCenter.x,
                            -(event.mouse.y - bCenter.y));
  sensor.info.attachAngle = mAngle;
  sensor.x = bCenter.x - sSize - bScale * attachRadius * Math.sin(-mAngle);
  sensor.y = bCenter.y - sSize - bScale * attachRadius * Math.cos( mAngle);
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
  const robotBay = document.getElementById("bayLemming");
  const bCenter = {x: robotBay.width/2,
                   y: robotBay.height/2},
        bScale = simInfo.bayScale;

  for (var ss = 0; ss < robot.info.sensors.length; ++ss) {
    const curSensor = robot.sensors[ss],
          attachAngle = curSensor.attachAngle,
          attachRadius = curSensor.attachRadius;
    // put current sensor into global variable, make mouse-interactive
    sensors[ss] = makeInteractiveElement(new SensorGraphics(curSensor),
                                         document.getElementById("bayLemming"));
    const sSize = sensors[ss].getWidth();
    sensors[ss].x = bCenter.x -sSize -bScale*attachRadius*Math.sin(-attachAngle);
    sensors[ss].y = bCenter.y -sSize -bScale*attachRadius*Math.cos( attachAngle);
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

function loadFromSVG() {
  var vertexSets = [];
  const svg = document.getElementById('robotbodySVG'),
        data = svg.contentDocument;

  jQuery(data).find('path').each(function(_, path) {
    var points = Matter.Svg.pathToVertices(path, 30);
    vertexSets.push(Matter.Vertices.scale(points, 0.2, 0.2));
  });

  return vertexSets;
};

function InstantiateRobot(robotInfo) {
  // load robot's body shape from SVG file
  const bodySVGpoints = loadFromSVG();
  this.body = Matter.Bodies.fromVertices(robotInfo.init.x,
                                         robotInfo.init.y,
                                         bodySVGpoints,
                                         {frictionAir: simInfo.airDrag,
                                          mass: simInfo.robotMass,
                                          color: [255, 255, 255],
                                          role: 'robot'}, true);

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
  this.move = robotMove;

  // add functions getWidth/getHeight for graphics.js & mouse.js,
  // to enable selection by clicking the robot in the arena
  this.getWidth = function() { return 2 * simInfo.robotSize; };
  this.getHeight = function() { return 2 * simInfo.robotSize; };
}

function robotUpdateSensors(robot) {
  // update all sensors of robot; puts new values into sensor.value
  for (var ss = 0; ss < robot.sensors.length; ss++) {
    var sensor = robot.sensors[ss];
    sensor.sense(sensor);
  }
};

function getSensorValById(robot, sensor_id) {
  for (var ss = 0; ss < robot.sensors.length; ss++) {
    if (robot.sensors[ss].id == sensor_id) {
      return robot.sensors[ss].value;
    }
  }
  return undefined;  // if not returned yet, sensor_id doesn't exist
};

function rotateBySetRadians(robot,
                            set_radians=Math.PI/2,
                            abs_torque=0.01) {
  /* Rotate the robot for the given amount in parameter set_radians;
   * abs_torque is the absolute amount of torque to apply per sim step.
   * Note that robotMove() is not executed until the rotation is done.
   *
   * Note how this is implemented by creating a closure to save the requested
   * goal angle when rotateBySetRadians() is called in robotMove().
   * The closure is temporarily inserted in robot.move (called each sim step).
   */

  abs_torque = Math.abs(abs_torque);  // never trust users :)

  // compute values to define and monitor the rotation progress
  const robot_angle = getSensorValById(robot, 'gyro'),  // current robot angle
        goal_angle = robot_angle + set_radians,  // goal angle after rotation
        error_angle = goal_angle - robot_angle,  // signed error angle
        sgn_torque = Math.sign(error_angle);  // which direction to rotate

  function rotateUntilDone() {
    const error_angle = goal_angle - getSensorValById(robot, 'gyro');
    // anything left to rotate, or did we already overshoot (if < 0)?
    const error_remaining = error_angle * sgn_torque;

    if (error_remaining > 0) {  // not done yet, rotate a bit further
      robot.rotate(robot, sgn_torque*abs_torque);
    }
    else {  // done with rotation, call robotMove() again in next sim step
      robot.move = robotMove;
    }
  };

  return rotateUntilDone;
}

function robotMove(robot) {
// TODO: Program Lemming behavior.
/* o By default, the Lemming should wander around in a slight right curve.
 * o If it senses a block its subsequent behavior depends on whether a block is
 *   in the gripper and the block color.
 *   – If it carries no block it should drive straight towards it, and thus get
 *     the block into the gripper.
 *   – If it carries a blue block, it keeps wandering and ignores the detected block.
 *   – If it carries a red block it turns left to leave the block.
 * o If it senses a wall its behavior should depend on whether a block is in the
 *   gripper and the block color.
 *   – If it carries no block it should turn either left or right.
 *   – If it carries a blue block it should turn left to leave the block.
 *   – If it carries a red block it should turn right to keep the block.
 */

  const frontS_dist_all = getSensorValById(robot, 'frontS_dist_all'),
        frontS_dist_noBox = getSensorValById(robot, 'frontS_dist_noBox'),
		color_gripper = getSensorValById(robot, 'color_gripper'),
		frontS_color_all = getSensorValById(robot, 'frontS_color_all'),
		frontS_color_noBox = getSensorValById(robot, 'frontS_color_noBox');
		defaultRotate = 0.0001;

  robot.drive(robot, 0.0002);
  robot.rotate(robot, defaultRotate);
  if (frontS_color_all == "r" || frontS_color_all == "b"){
	  if (color_gripper == "n") {
		  robot.rotate(robot,0);
	  }
	  else if (color_gripper == "b") {
		  robot.rotate(robot, defaultRotate);
	  }
	  else if (color_gripper == "r") {
		  robot.rotate(robot, -40*defaultRotate);
	  }
  }
  else if (frontS_color_all == "w" && frontS_dist_all < 20) {
	  robot.drive(robot,-0.0015);
	  if (color_gripper == "n") {
		  robot.rotate(robot,-140*defaultRotate);
	  }
	  else if (color_gripper == "b" ){
		  robot.rotate(robot, -10*defaultRotate);
	  }
	  else if (color_gripper == "r" ){
		  robot.rotate(robot,defaultRotate);
	  }
  }
 
  // A demonstration of a turn every 200th step (as an example of some condition)
  //if (!(simInfo.curSteps % 200)) {
    // Attach new closure to robot.move (the function called every sim step),
    // to prevent entering robotMove() until the requested rotation is done:
    //robot.move = rotateBySetRadians(robot, Math.PI/1.5);
  //}

};

function plotSensor(context, x=this.x, y=this.y) {
  var color = 'black';
  if (this.info.color) {
    color = convrgb(this.info.color);
  }
  context.beginPath();
  context.arc(x + this.getWidth()/2,
              y + this.getHeight()/2,
              this.getWidth()/2, 0, 2*Math.PI);
  context.closePath();
  context.fillStyle = color;
  context.strokeStyle = color;
  context.fill();
  context.stroke();
}

function plotRobot(context,
                   xTopLeft = this.body.position.x,
                   yTopLeft = this.body.position.y) {
  var x, y, scale, angle, i, half, full,
      rSize = simInfo.robotSize;
  const showInternalEdges = false;

  if (context.canvas.id == "bayLemming") {
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

  if (context.canvas.id == "arenaLemming") {
    // draw into world canvas without transformations,
    // because MatterJS thinks in world coords...
    context.restore();

    const body = this.body;
    // handle compound parts

    context.beginPath();
    for (k = body.parts.length > 1 ? 1 : 0; k < body.parts.length; k++) {
      part = body.parts[k];
      context.moveTo(part.vertices[0].x,
                     part.vertices[0].y);
      for (j = 1; j < part.vertices.length; j++) {
        if (!part.vertices[j - 1].isInternal || showInternalEdges) {
          context.lineTo(part.vertices[j].x,
                         part.vertices[j].y);
        } else {
          context.moveTo(part.vertices[j].x,
                         part.vertices[j].y);
        }

        if (part.vertices[j].isInternal && !showInternalEdges) {
          context.moveTo(part.vertices[(j + 1) % part.vertices.length].x,
                         part.vertices[(j + 1) % part.vertices.length].y);
        }
      }
      context.lineTo(part.vertices[0].x,
                     part.vertices[0].y);
    }
    context.strokeStyle = convrgb(body.color);
    context.lineWidth = 1.5;
    context.stroke();

    // to draw the rest, rotate & translate again
    context.save();
    context.translate(x, y);
    context.rotate(angle);
  }

  // Plot sensor positions into world canvas.
  if (context.canvas.id == "arenaLemming") {
    for (ss = 0; ss < this.info.sensors.length; ++ss) {
      const attachAngle = this.info.sensors[ss].attachAngle,
            attachRadius = this.info.sensors[ss].attachRadius;
      var color = 'black';

      if (this.info.sensors[ss].color) {
        color = convrgb(this.info.sensors[ss].color);
      }

      context.beginPath();
      context.arc(attachRadius * Math.cos(attachAngle),
                  attachRadius * Math.sin(attachAngle),
                  scale, 0, 2*Math.PI);
      context.closePath();
      context.fillStyle = color;
      context.strokeStyle = color;
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
      var robot = robots[rr];
      robotUpdateSensors(robot);
      robot.move(robot);
      // To enable selection by clicking (via mouse.js/graphics.js),
      // the position on the canvas needs to be defined in (x, y):
      const rSize = simInfo.robotSize;
      robot.x = robot.body.position.x - rSize;
      robot.y = robot.body.position.y - rSize;
    }
    // count and display number of steps
    simInfo.curSteps += 1;
    document.getElementById("SimStepLabel").innerHTML =
      padnumber(simInfo.curSteps, 5) +
      ' of ' + padnumber(simInfo.maxSteps, 5);
  }
  else {
    toggleSimulation();
  }
}

function drawBoard() {
  var context = document.getElementById('arenaLemming').getContext('2d');
  context.fillStyle = "#444444";
  context.fillRect(0, 0, simInfo.width, simInfo.height);

  // draw objects within world
  const Composite = Matter.Composite,
        bodies = Composite.allBodies(simInfo.world);

  for (var bb = 0; bb < bodies.length; bb += 1) {
    var vertices = bodies[bb].vertices,
        vv;

    // draw all non-robot bodies here (walls and boxes)
    // don't draw robot's bodies here; they're drawn in plotRobot()
    if (bodies[bb].role != 'robot') {
      context.beginPath();
      context.moveTo(vertices[0].x, vertices[0].y);
      for (vv = 1; vv < vertices.length; vv += 1) {
        context.lineTo(vertices[vv].x, vertices[vv].y);
      }
      if (bodies[bb].color) {
        context.strokeStyle = convrgb(bodies[bb].color);
        context.closePath();
        context.stroke();
      }
    }
  }
  context.lineWidth = 1;

  // draw all robots
  for (var rr = 0; rr < robots.length; ++rr) {
    robots[rr].plotRobot(context);
  }
}

function repaintBay() {
  // update inset canvas showing information about selected robot
  const robotBay = document.getElementById('bayLemming'),
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
      var sensor = rsensors[ss],
          colorStr = 'black';
      if (sensor.color) {
        colorStr = convrgb(sensor.color);
      }
      sensorString += '<br> <span style="color:' + colorStr + ';">'
        + 'id \'' + sensor.id + '\': ' + sensor.valueStr;
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
                                       document.getElementById("arenaLemming"));

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

function format(number, numFix=1) {
  // prevent HTML elements to jump around at sign flips etc
  return (number >= 0 ? '+' : '−') + Math.abs(number).toFixed(numFix);
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

