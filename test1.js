var rk4 = require('ode-rk4')


//massive objects in the system....each has a position, s, determined by
//a function 
var massives = {
    "earth": { 
        mass: 1000.0, 
        rad: 400.0, 
        pos: (t) => [0.0,0.0]
    },
    "moon": { 
        mass: 200.0, 
        rad: 100.0, 
        pos: (t) => [20000.0*Math.cos(t*2.0*Math.PI/30.0), 
                     20000.0*Math.sin(t*2.0*Math.PI/30.0)] 
    }
}

var player = {
    mass: 1.0,
    pos: [ 4000.0, 0.0 ],
    v: [0.0, 0.0],
    thrust: 10
}


function distance(a, b) {
    var dist = Math.sqrt(Math.pow(Math.abs(a[0]-b[0]),2) + Math.pow(Math.abs(a[1]-b[1]),2));
    return dist;
}

function vcopyTo(v, b) {
    b[0] = v[0]
    b[1] = v[1]
}

function vadd(a,b) {
    v = [0,0]
    v[0] = a[0] + b[0]
    v[1] = a[1] + b[1]
    return v;
}

function vsub(a,b) {
    v = [a[0]-b[0], a[1]-b[1]];
    return v;
}

function vmag(v) {
    return Math.sqrt(Math.pow(v[0],2) + Math.pow(v[1],2));
}

function vmul(v, s) {
    return [v[0]*s, v[1]*s];
}

function vnorm(v) {
    mag = vmag(v)
    if(mag === 0) { return [0,0]; }
    return vmul(v, 1/vmag(v));
}

// calculates force on object a due to massive body b
function force_flat(a_mass, a_pos, b_mass, b_pos) {
    G = 50;
    fmag = G*a_mass*b_mass/Math.pow(distance(a_pos,b_pos),2);
    let acc = vmul(vnorm(vsub(b_pos, a_pos)), fmag); // a -> b
    return acc;
}


var deriv = function(dydt, y, t) {
    //initialize acceleration to player's thrust vector
    //oriented orthogonal to earth
    var acc = vmul(vnorm([-y[1], y[0]]), player.thrust);

    //enumerate over gravity terms from massive bodies
    //massive bodies can have position change due to time (e.g. moon)
    for(body in massives) {
        let accbody = force_flat(player.mass, y, 
            massives[body].mass, massives[body].pos(t) )
        acc = vadd(acc, accbody)
    }

    // ds/dt = v(t) + a(t)*dt
    dydt[0] = player.v[0] + dt*acc[0]
    dydt[1] = player.v[1] + dt*acc[1]
}

var dt = 1.0/100;

var t0 = 0;
var steps = 0;
var y = [player.pos[0], player.pos[1]];
var integrator = rk4( y, deriv, t0, dt )
while(steps<1000) {

 integrator.steps(1);
 steps += 1;
 t0 += dt;
 
 var pos2 = integrator.y;
 player.v = vmul(vsub(pos2,player.pos),1/dt);  //  v = (x2-x1)/dt
 vcopyTo(pos2, player.pos);

 console.log(steps, t0, player.pos, player.v)
}
