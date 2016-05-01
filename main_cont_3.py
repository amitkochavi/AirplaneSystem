from visual import *

import random

# Airplane Simulation

##

# all variables with the prefix a_ are in the world frame

# when looking at the screen at the start,

# world x­axis is right

# world y­axis is up

# world z­axis is out of the screen

# all variables with the prefix air_ are in the airplane's frame

# world x­axis is lateral (right +ve)

# world y­axis is longitudinal (forward +ve)

# world z­axis is up

##

print("""Airplane Simulation""")

scene_size = 200;

global pause;

global camera_view;

pause = 1;

camera_view = 3;

## initial airplane state variables

a_pos = vector(­10,1000,0);

a_vel = vector(280,0,0);

a_pos = vector(­10,6000,0);

a_vel = vector(210,0,0);

autopilot = False

# initial values

init_alpha = 15.070*pi/180; # initial angle of attack

init_beta = 0;

air_u1 = 22*pi/180; # angle of actuator 1 in radians

air_u2 = 22*pi/180; # angle of actuator 2 in radians

air_u3 = 0; # angle of actuator 3 in degrees

####################################################################

# pilot and control parameters: do not touch anything below this

####################################################################

## pilot input

global pilot_u1;

global pilot_u2;

global pilot_u3;

global pilot_t0;

# pilot input variables

pilot_u1 = 0;

pilot_u2 = 0;

pilot_u3 = 0;

pilot_t0 = 0.7;

# define functions for keyboard control

def keyPress(event):

global camera_view;

global pause;

global pilot_u1;

global pilot_u2;

global pilot_u3;

global pilot_t0;

K_pitch = 0.05;

K_roll = 0.05;

K_yaw = 0.05;

K_thrust = 0.01;

if event.key == 'c':

camera_view = (camera_view+1)%4;

if event.key == 'e':

pause = (pause+1)%2;

if event.key == 'up':

pilot_u1 = (1­K_pitch)*pilot_u1 ­ K_pitch;

pilot_u2 = (1­K_pitch)*pilot_u2 ­ K_pitch;

if event.key == 'down':

pilot_u1 = (1­K_pitch)*pilot_u1 + K_pitch;

pilot_u2 = (1­K_pitch)*pilot_u2 + K_pitch;

if event.key == 'left':

pilot_u1 = (1­K_roll)*pilot_u1 ­ K_roll;

pilot_u2 = (1­K_roll)*pilot_u2 + K_roll;

if event.key == 'right':

pilot_u1 = (1­K_roll)*pilot_u1 + K_roll;

pilot_u2 = (1­K_roll)*pilot_u2 ­ K_roll;

if event.key == 'a':

pilot_u3 = (1­K_yaw)*pilot_u3 ­ K_yaw;

if event.key == 'd':

pilot_u3 = (1­K_yaw)*pilot_u3 + K_yaw;

if event.key == 'w':

pilot_t0 = pilot_t0 + K_thrust;

if event.key == 's':

pilot_t0 = pilot_t0 ­ K_thrust;

if pilot_u1 > 1:

pilot_u1 = 1;

if pilot_u1 < ­1:

pilot_u1 = ­1;

if pilot_u2 > 1:

pilot_u2 = 1;

if pilot_u2 < ­1:

pilot_u2 = ­1;

if pilot_u3 > 1:

pilot_u3 = 1;

if pilot_u3 < ­1:

pilot_u3 = ­1;

if pilot_t0 > 1:

pilot_t0 = 1;

if pilot_t0 < 0:

pilot_t0 = 0;

def stick_decay():

global pilot_u1;

global pilot_u2;

global pilot_u3;

K_u1 = 0.25;

K_u2 = 0.25;

K_u3 = 0.25;

mu = 0;

sigma = 0.0000005;

pilot_u1 = pilot_u1 ­ K_u1*pilot_u1 + random.gauss(mu, sigma);

pilot_u2 = pilot_u2 ­ K_u2*pilot_u2 + random.gauss(mu, sigma);

pilot_u3 = pilot_u3 ­ K_u3*pilot_u3 + random.gauss(mu, sigma);

## actuator parameters

# limits

air_wga_max = vector(10,1,10);

air_t0_max = 25000;

air_t0_min = 0;

air_u_max = pi/6;

air_u_min = ­pi/6;

# controller gains

K_roll = 21.15;

K_pitch = 20.25;

K_yaw = 0.7;

##########################################################

# airplane parameters: do not touch anything below this

##########################################################

a_x = vector(0,0,1);

a_y = rotate(vector(1,0,0), angle=init_alpha, axis=a_x);

a_z = rotate(vector(0,1,0), angle=init_alpha, axis=a_x);

a_vel1 = a_vel;

## in airplane frame

air_vel = a_vel.mag;

air_wga = vector(0,0,0);

air_wga1 = air_wga;

air_alpha = diff_angle(vector(dot(a_vel,a_x),dot(a_vel,a_y),0),\

vector(dot(a_vel,a_x),dot(a_vel,a_y),dot(a_vel,a_z)));

if dot(a_vel,a_z) >= 0:

air_alpha = ­air_alpha;

air_beta = diff_angle(vector(dot(a_vel,a_x),0,dot(a_vel,a_z)),\

vector(dot(a_vel,a_x),dot(a_vel,a_y),dot(a_vel,a_z)));

if dot(a_vel,a_x) >= 0:

air_beta = ­air_beta;

air_wga = vector(0,0,0);

# airframe geometry

air_area = 15;

air_area_ac1 = 2;

air_area_ac2 = 2;

air_area_ac3 = 1.5;

air_area_ac4 = 2.5;

air_rc0 = vector(0,­0.5,0);

air_rc1 = vector(­2,­2,0);

air_rc2 = vector(2,­2,0);

air_rc3 = vector(0,­3,2);

air_rc4 = vector(0,­2,2);

# airframe inertia

air_m = 3000;

air_I = 20000;

# air forces

air_acc_ang = vector(0,0,0);

air_tau = vector(0,0,0);

air_force = vector(0,0,0);

air_acc = vector(0,0,0);

air_fc0 = vector(0,0,0);

air_fc1 = vector(0,0,0);

air_fc2 = vector(0,0,0);

air_fc3 = vector(0,0,0);

air_fc4 = vector(0,0,0);

##########################################################

# environment and graphics

##########################################################

# environment variables

rho = 1.225; # air density

b = 1.983; # air vilocity

g = 9.81; # gravity

air_rho = rho/pow(a_pos.y+6400000,2)*pow(6400000,2);

# scene create

scene = display(title='Airplane Simulation',

x=0, y=0, width=1024, height=768,

center=(0,10,0), background=(0,0.5,1),range=scene_size );

scene.visisble = 0;

scene.autoscale = 1;

scene.center = a_pos;

scene.bind('keydown', keyPress);

# ground

ground_box = box(size=(300*scene_size,0.1,300*scene_size),\

material=materials.wood);

ground_box.pos = (a_pos.x,ground_box.y,a_pos.z);

### sun

##scene.lights = [];

##sun_sphere = sphere(pos=2000*scene_size*vector(1,1,1), radius=30000,\

## color=(1,1,0.5), material=materials.emissive);

##lamp = local_light(pos=2000*scene_size*vector(1,1,1), color=(1,1,0.5));

# draw airplane

a_plane_fus = cone(make_trail=True, radius=0.2, color=color.red);

a_plane_wing = box(size=(3,0.1,2), color=color.red);

a_plane_tail = box(make_trail=True, size=(2,0.5,0.01) , color=color.blue);

a_plane_fus.pos = a_pos;

a_plane_fus.axis = a_y;

a_plane_fus.up = a_z;

a_plane_wing.pos = a_pos;

a_plane_wing.axis = a_y;

a_plane_wing.up = a_z;

a_plane_tail.pos = a_pos+0.1*a_z;

a_plane_tail.axis = a_y;

a_plane_tail.up = a_z;

# data readout

frame_correctness = 0;

data_text = "";

crash_warning = "";

crash_correction = "";

# draw crash curve

global traj;

traj = curve( color = color.red );

traj_point = a_pos;

traj1 = curve( color = color.cyan );

traj1_point = a_pos;

traj2 = curve( color = color.cyan );

traj2_point = a_pos;

traj3 = curve( color = color.cyan );

traj3_point = a_pos;

traj4 = curve( color = color.cyan );

traj4_point = a_pos;

t_amt = 500;

t_scale = 10;

##########################################################

# main loop of simulation

##########################################################

K_aoa = air_m/g/1.1753;

t = 0;

dt = 0.02;

while True:

rate(50)

# controller

air_u1 = K_aoa/pow(air_vel,2)/air_rho*(air_acc.z+g)\

+ K_pitch*((pilot_u1+pilot_u2)/2*air_wga_max.x)\

­ K_roll*((pilot_u1­pilot_u2)/2*air_wga_max.y);

air_u2 = K_aoa/pow(air_vel,2)/air_rho*(air_acc.z+g)\

+ K_pitch*((pilot_u1+pilot_u2)/2*air_wga_max.x)\

­ K_roll*((pilot_u2­pilot_u1)/2*air_wga_max.y);

air_u3 = K_yaw*((pilot_u3)*air_wga_max.z)\

­ cross(air_rc3, air_fc3).z*0.0000005;

air_t0 = air_t0_max*air_rho/rho*pilot_t0;

if air_u1 > air_u_max:

air_u1 = air_u_max;

if air_u1 < air_u_min:

air_u1 = air_u_min;

if air_u2 > air_u_max:

air_u2 = air_u_max;

if air_u2 < air_u_min:

air_u2 = air_u_min;

if air_u3 > air_u_max:

air_u3 = air_u_max;

if air_u3 < air_u_min:

air_u3 = air_u_min;

if air_t0 > air_t0_max:

air_t0 = air_t0_max;

if air_t0 < air_t0_min:

air_t0 = air_t0_min;

## iterate airplane in airplane frame

# find forces acting on plane

air_rho = rho/pow(a_pos.y+6400000,2)*pow(6400000,2);

air_alpha = diff_angle(vector(dot(a_vel,a_x),dot(a_vel,a_y),0),\

if dot(a_vel,a_z) >= 0:

air_alpha = ­air_alpha;

air_beta = diff_angle(vector(0,dot(a_vel,a_y),dot(a_vel,a_z)),\

vector(dot(a_vel,a_x),dot(a_vel,a_y),dot(a_vel,a_z)));

if dot(a_vel,a_x) >= 0:

air_beta = ­air_beta;

air_vel = a_vel.mag;

air_fc0 = air_area*air_rho*pow(air_vel,2)*sin(air_alpha)*\

vector(0,0,1);

air_fc1 = air_area_ac1*air_rho*pow(air_vel,2)*sin(air_alpha­air_u1)*\

vector(0, sin(air_u1), cos(air_u1));

air_fc2 = air_area_ac2*air_rho*pow(air_vel,2)*sin(air_alpha­air_u2)*\

vector(0, sin(air_u2), cos(air_u2));

air_fc3 = air_area_ac3*air_rho*pow(air_vel,2)*sin(air_beta­air_u3)*\

vector(cos(air_u3), sin(air_u3),0);

air_fc4 = air_area_ac4*air_rho*pow(air_vel,2)*sin(air_beta)*\

vector(1,0,0);

air_force = air_fc0 + air_fc1 + air_fc2 + air_fc3 + air_fc4 +\

air_t0*vector(0,1,0)\

­ 20*air_rho*b*air_vel*vector(0,1,0);

air_tau = cross(air_rc0, air_fc0) + cross(air_rc1, air_fc1)\

+ cross(air_rc2, air_fc2) + cross(air_rc3, air_fc3)\

+ cross(air_rc4, air_fc4)\

­ 300*air_rho*b*air_vel*air_wga;

air_acc = air_force/air_m;

air_acc_ang = air_tau/air_I;

# all forces evaluated at this point

## finding velocities and distance, integration, ode solver

# velocity from acceleration, zero order

air_wga = air_wga + air_acc_ang*dt;

a_acc = vector(\

dot(air_acc,vector(a_x.x,a_y.x,a_z.x)),\

dot(air_acc,vector(a_x.y,a_y.y,a_z.y)),\

dot(air_acc,vector(a_x.z,a_y.z,a_z.z)));

a_vel = a_vel + a_acc*dt + vector(0,­g,0)*dt;

# position from velocity, 1st order trapezoidal

a_pos = a_pos + (a_vel+a_vel1)/2*dt;

a_y = rotate(a_y, axis=norm(a_x), angle=(air_wga.x+air_wga1.x)/2*dt);

a_z = rotate(a_z, axis=norm(a_x), angle=(air_wga.x+air_wga1.x)/2*dt);

a_x = rotate(a_x, axis=norm(a_z), angle=(air_wga.z+air_wga1.z)/2*dt);

a_y = rotate(a_y, axis=norm(a_z), angle=(air_wga.z+air_wga1.z)/2*dt);

a_x = rotate(a_x, axis=norm(a_y), angle=(air_wga.y+air_wga1.y)/2*dt);

a_z = rotate(a_z, axis=norm(a_y), angle=(air_wga.y+air_wga1.y)/2*dt);

# store old variables

a_vel1 = a_vel;

air_wga1 = air_wga;

# check for crash

if (a_pos.y <= 0):

a_pos.y = 0;

a_vel = vector(0,0,0);

air_wga = vector(0,0,0);

air_t0_max = 0;

rho = 0;

#g = 0;

frame_correctness = dot(a_x,a_y)+dot(a_z,a_y)+dot(a_x,a_z) + \

# draw airplane

a_plane_fus.pos = a_pos;

a_plane_fus.axis = a_y;

a_plane_fus.up = a_z;

a_plane_wing.pos = a_pos;

a_plane_wing.axis = a_y;

a_plane_wing.up = a_z;

a_plane_tail.pos = a_pos+0.1*a_z;

a_plane_tail.axis = a_y;

a_plane_tail.up = a_z;

# update data_readout

data_text = "position:(%10.3f"%a_pos.x+"%10.3f"%a_pos.y+" %10.3f"%a_pos.z+ ")\n"\

"velocity: %10.3f" % air_vel+ "(%10.3f"%norm(a_vel).x+\

" %10.3f"%norm(a_vel).y+ " %10.3f"%norm(a_vel).z+ ")\n"\

+"altitude: %10.3f" % a_pos.y+"\n"\

+"g­force: %10.3f"%(air_acc.x/g)\

+ " %10.3f"%(air_acc.y/g) + " %10.3f"%(air_acc.z/g) +"\n"\

+"g­torque: %10.3f"%(air_acc_ang.x/g)\

+ " %10.3f"%(air_acc_ang.y/g) + "% 10.3f"%(air_acc_ang.z/g) +"\n"\

+"omega: %10.3f"%air_wga.x\

+ " %10.3f"%air_wga.y + " %10.3f"%air_wga.z +"\n"\

+ "aoa: %10.3f" %(air_alpha*180/pi) +"\n"\

+ "beta: %10.3f" %(air_beta*180/pi) +"\n"\

+ "actuators: %10.3f"%(air_u1*180/pi) + " % 10.3f"%(air_u2*180/pi)\

+ " % 10.3f"%(air_u3*180/pi) + " %10.3f"%air_t0 +"\n"\

+ "stick: %10.3f"%pilot_u1 + " % 10.3f"%pilot_u2\

+ " % 10.3f"%pilot_u3 + " %10.3f"%pilot_t0+"\n"\

"frame correctness: %10.3f"%frame_correctness+"\n"\

"autopilot: "+str(autopilot)+"\n"\

+ "camera view: %i"%camera_view +"\n\r\n" ;

#print "\r\n" * 100

if (a_pos.y <= 0):

break;

#################################################################

## find limit trajectories

# K_z = 0.00093187*pow(air_vel,2);

K_z = 0.00129*pow(air_vel,2);

K_y = ­0.00017*pow(air_vel,2)+air_t0/air_m\

­ 20*air_rho*b*air_vel/air_m;

K_t = air_t0_max/air_m/g;

crash_pos = vector(0,0,0);

c0_pos = a_pos;

c0_vel = a_vel;

c0_acc = a_acc+vector(0,­g,0);

c1_pos = a_pos;

c1_vel = a_vel;

c1_acc = K_z*a_z+vector(0,­g,0)+K_y*a_y;

c2_pos = a_pos;

c2_vel = a_vel;

c2_acc = ­K_z*a_z+vector(0,­g,0)+K_y*a_y;

c3_pos = a_pos;

c3_vel = a_vel;

c3_acc = ­K_z*a_x+vector(0,­g,0)+K_y*a_y;

c4_pos = a_pos;

c4_vel = a_vel;

c4_acc = K_z*a_x+vector(0,­g,0)+K_y*a_y;

traj.pos = a_pos;

traj1.pos = a_pos; # up

traj2.pos = a_pos; # down

traj3.pos = a_pos; # port

traj4.pos = a_pos; # starboard

crash_0 = False; # 0 true, 1 false

crash_1 = False;

crash_2 = False;

crash_3 = False;

crash_4 = False;

for i in range(0, int(20*t_amt/air_vel)):

c0_pos = c0_pos + (c0_vel)*t_scale*dt;

c0_vel = c0_vel + (c0_acc)*t_scale*dt;

c0_acc = rotate(c0_acc, axis=a_x, angle=air_wga.x*dt);

c1_pos = c1_pos + (c1_vel)*t_scale*dt;

c1_vel = c1_vel + (c1_acc)*t_scale*dt;

c1_acc = rotate(c1_acc, axis=a_x, angle= ­air_wga.x*dt);

c2_pos = c2_pos + (c2_vel)*t_scale*dt;

c2_vel = c2_vel + (c2_acc)*t_scale*dt;

c2_acc = rotate(c2_acc, axis=a_x, angle= air_wga.x*dt);

c3_pos = c3_pos + (c3_vel)*t_scale*dt;

c3_vel = c3_vel + (c3_acc)*t_scale*dt;

c3_acc = rotate(c3_acc, axis=a_x, angle= air_wga.x*dt);

c4_pos = c4_pos + (c4_vel)*t_scale*dt;

c4_vel = c4_vel + (c4_acc)*t_scale*dt;

c4_acc = rotate(c4_acc, axis=a_x, angle= air_wga.x*dt);

traj.append( pos= c0_pos);

traj1.append( pos= c1_pos);

traj2.append( pos= c2_pos);

traj3.append( pos= c3_pos);

traj4.append( pos= c4_pos);

if (crash_0 == False)&(c0_pos.y <= 0):

crash0_pos = c0_pos;

crash_0 = True;

if (crash_4 == False)&(c4_pos.y <= 0):

crash_4 = True;

crash4_pos = c4_pos;

if (crash_3 == False)&(c3_pos.y <= 0):

crash_3 = True;

crash3_pos = c3_pos;

if (crash_2 == False)&(c2_pos.y <= 0):

crash_2 = True;

crash2_pos = c2_pos;

if (crash_1 == False)&(c1_pos.y <= 0):

crash_1 = True;

crash1_pos = c1_pos;

# crash decision

#K_pitch+=100

#a_vel.y+=1

#print K_pitch

crash_warning = "";

if (crash_0==True):

crash_warning = "on crash course, "

autopilot = True

if autopilot:

crash_warning+= "autopilot, "

if (crash_4==True)&(crash_3==False):

crash_correction = "roll left, pull up";

if (crash_4==False)&(crash_3==True):

crash_correction = "roll right, pull up";

if (crash_1==True)&(crash_2==False):

crash_correction = "roll full, pull up";

if (crash_1==False)&(crash_2==True):

crash_correction = "pull up";

if (crash_4==True)&(crash_3==True)&(crash_1==True)&(crash_2==True):

if mag(a_pos­crash0_pos) > \

1000*(K_t­g/air_m*(a_vel.y)/mag(a_vel)): # min safe altitude

if mag(a_pos­crash3_pos) > mag(a_pos­crash4_pos):

if mag(a_pos­crash4_pos) > mag(a_pos­crash3_pos):

if mag(a_pos­crash2_pos) > mag(a_pos­crash1_pos):

if mag(a_pos­crash1_pos) > mag(a_pos­crash2_pos):

if (mag(a_pos­crash1_pos) == mag(a_pos­crash2_pos))&\

else:

crash_correction = "eject";

else:

if (crash_4==True):

crash_warning = "safe, crash if right, ";

crash_correction = "";

if (crash_3==True):

crash_warning = "safe, crash if left, ";

crash_correction = "";

if (crash_2==True):

crash_warning = "safe, crash if down, ";

#pilot_ul = 1

crash_correction = "";

if (crash_1==True):

crash_warning = "safe, crash if up, ";

crash_correction = "";

if (crash_4==False)&(crash_3==False)&(crash_1==False)&(crash_2==False):

crash_warning = "safe";

crash_correction = "";

#autopilot

#print a_vel.y

if autopilot:

#print a_x, a_y, a_z

#print a_z[1]

if a_pos.y < 2000 and a_vel.y < 100:

#a_vel.y+=1

pilot_t0= 1.

if (crash_4==True)&(crash_0==True):

print "roll left"

pilot_u1 =1.

if (crash_3==False): pilot_u2= ­1.

if (crash_4==False)&(crash_0==True):

print "roll right"

if (crash_3==True): pilot_u1 =­1.

pilot_u2= 1.

if (crash_1==True)&(crash_2==False):

print "full roll right"

pilot_u1 =­1.

pilot_u2= 1.

else:

if a_vel.y < 0:

elif a_vel.y < 50:

if a_pos.y > 2000:

if a_vel.y < 2 and a_vel.y > ­2:

pilot_u1 =0.

pilot_u2= 0.

#a_vel.y=0

autopilot = false

elif a_vel.y>0:

#a_vel.y­=1

#print "alt"+str(­a_vel.y/800)

if a_vel.y < 50:

else:

elif a_vel.y<0:

#a_vel.y+=1

# pilot_u1 =1.­a_vel.y/100

# pilot_u2= 1.­a_vel.y/100

pilot_u1 =.7

pilot_u2= .7

#print a_vel.y/500

if a_vel.y > ­50:

#if a_vel.y > ­200:

# pilot_u1 = .05­a_vel.y/500

# pilot_u2 = .05­a_vel.y/500

#################################################################

if mod(t,0.5) < dt:

print(data_text);

print(crash_warning+crash_correction+"\r\n");

stick_decay();

# update scene

scene.center = a_plane_fus.pos;

# set camera view based on value in camera_view

if camera_view == 0:

scene.forward = 0.1*a_y + 1*norm(a_vel);

scene.up = a_z;

scene.autoscale = False;

if camera_view == 1:

scene.forward = ­0.1*a_y ­ 1*norm(a_vel);

scene.up = a_z;

scene.autoscale = False;

if camera_view == 2:

scene.forward = 1*a_y + 0*norm(a_vel) ­ 0.2*a_z;

scene.up = a_z;

scene.autoscale = False;

if camera_view == 3:

scene.autoscale = False;

scene.up = vector(0,1,0);

# pause

while (pause == 0):

rate(50);

if pause == 1:

break;

if t == 0.: # make sure everything is set up before first visible display

scene.visible = 1;

t = t+dt;
