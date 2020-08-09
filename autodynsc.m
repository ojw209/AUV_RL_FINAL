function qd = autodynsc(t,Y)
% written by wrk/jleonard fall 97/spring 98
% status: 2/12/98 - adding closed loop control
% re-worked by SC Tang; Fall 1998
% sol = [u v w p q r x y z phi theta psi Tstbd Taft Tport Tfore]
% =[yl y2 y3 y4 y5 y6 y7 y8 y9 y10 yl l y12 yl3 yl4 yl5 yl6]

global m Ixx Iyy Izz
global xpfore xpaft yp zp zb B W rho
global rm rml
global rh lh Ah Cdh Vh ma_h
global span chord Ast Cdst ma_st
global rph lph Aph Cdph Vph ma_ph
global Xuu Xud
global Yvd Yrd Yvv
global Zwd Zww Zqd
global Kpp Kpd
global Mqd Mqq Mwd Mww
global Nrr Nvd Nrd Nvv
global Thrust cur_step max_steps
global Command
global controller_type
global OPEN_LOOP_FORWARD OPEN_LOOP_DOWN DEPTH_CONTROL DEPTH_PITCH_CONTROL
global OPEN_LOOP
global Tforce % for thruster force

radtodeg = 180/pi;
degtorad = pi/180;



u =Y(1);
v= Y(2);
w =Y(3);
p =Y(4);
q =Y(5);
r =Y(6);
x =Y(7);
y =Y(8);
z =Y(9);
phi = Y(10);
theta = Y(11);
psi = Y(12);

xG = 0.0053;        % center of mass is fwd of vehicle reference origin in x-axis (m)
yG = 0;             % assume no heeling (m)
zG = 0.0083;        % center of mass is below the origin in +ve z-axis (m)
% see spreadsheet (Table 4-1) for xG and zG
Tforce =1.75;       % thruster force in N, usage is optional

max_steps = 100000;

%%%%%%Determine control inputs %%%%%%
commanded_depth = 10.0;
commanded_speed = 1;
commanded_heading = 0.0;
commanded_pitch = 0.0;
depth_tolerance = 0.1;
speed_tolerance = 0.1;
heading_tolerance = 0.1;
pitch_tolerance = 2.0*degtorad;

% switch controller_type
%     case OPEN_LOOP_FORWARD
%         if (t==0)
%             disp('open loop forward selected')
%         end
%         Tstbd = 1.6; % thruster force
%         Taft = 0;
%         Tport =-1.5;
%         Tfore = 0;
%         
%     case OPEN_LOOP_DOWN
%         if (t==0)
%             disp('open loop down selected')
%         end
%         Tstbd = 0;
%         Taft = 0.6;
%         Tport=0;
%         Tfore =0.75;
%         
%     case OPEN_LOOP
%         if (t==O)
%             disp('open loop forward and down selected')
%         end
%         Tstbd = Tforce;
%         Taft = Tforce;
%         Tport = Tforce;
%         Tfore = Tforce;
%         
%     case DEPTH_CONTROL
%         if (t==O)
%             disp('depth control selected -- linear controller')
%         end
%         Tport = commanded_speed;
%         Tstbd = commanded_speed;
%         
%         kdp = 0.1; % proportional depth gain
%         depth_error = commanded_depth - z;
%         depth_cmd = depth_error * kdp;
%         if (depth_cmd > 1) depth_cmd = 1; end
%         if (depth_cmd < -1) depth_cmd = - 1; end
%         kpp = 1; % proportional pitch gain
%         kpd = 100; % derivative pitch gain
%         
%         pitch_error = commanded_pitch-z;
%         
%         % q is pitch rate
%         pitch_cmd = pitch_error * kpp + kpd*q;
%         
%         % Do
%         Taft = depth_cmd + pitch_cmd;
%         Tfore = depth_cmd - pitch_cmd;
%         if (Taft > Tforce) Taft=Tforce; end
%         if (Taft < -Tforce) Taft=-Tforce; end
%         if (Tfore > Tforce) Tfore=Tforce; end
%         if (Tfore < -Tforce) Tfore=-Tforce; end
%         
%     case DEPTH_PITCH_CONTROL
%         if (t==0)
%             disp('depth pitch control selected')
%         end
%         Tport = commanded_speed;
%         Tstbd = commanded_speed;
%         
%         if (z<(commanded_depth - depth_tolerance))
%             
%             Taft =Tforce;
%             
%         elseif (z>(commanded_depth + depth_tolerance))
%             
%             Taft = -Tforce;
%             
%         else
%             Taft =0;
%         end
%         if (theta<(commanded_pitch - pitch_tolerance))
%             
%             Tfore = Tforce;
%             
%         elseif (theta>(commanded_pitch + pitch_tolerance))
%             
%             Tfore = -Tforce;
%             
%         else
%             
%             Tfore = 0;
%             
%         end
%         
%     otherwise
%         disp('Unknown controller type in autodyn');
%         
%         Taft = 0;
%         Tfore = 0;
%         Tport = 0;
%         Tstbd = 0;
%         
% end


% Surge Equation
udot = (1/(m-Xud))*( Zwd*w*q + Zqd*q^2 -Yvd*v*r - Yrd*r^2 +...
    Xuu*u*abs(u) + Tport + Tstbd - (W-B)*sin(theta) + ...
    m*v*r - m*w*q + m*xG*(q^2 + r^2) - m*zG*(p*r));

% Motions in Vertical Plane
% Heave Equation
wdot = (1/(m-Zwd))*( Zqd- Xud*u*q + Yvd*v*p + Yrd*r*p + ...
    Zww*w*abs(w) + Taft + Tfore + (W-B)*cos(theta)*cos(phi) + ...
    m*u*q - m*v*p + m*zG*(p^2+q^2) - m*xG*(r*p));

% Pitch Equation
qdot = (1/(Iyy-Mqd))*( Zqd*(wdot-u*q) - (Zwd-Xud)*w*u - Yrd*v*p +...
    (Kpd-Nrd)*r*p + Mqq*q*abs(q) + Mww*w*abs(w) + ...
    (Tport+Tstbd)*zp - Tfore*xpfore + Taft*xpaft - ...
    (zG*W-zb*B)*sin(theta) - xG*(W-B)*cos(theta)*cos(phi) - ...
    (Ixx-Izz)*r*p - m*( zG*(udot-v*r+w*q) - xG*(wdot-u*q+v*p)));

vertical_plane_only=0;
if (vertical_plane_only== 1)
    vdot =0;
    pdot = 0;
    rdot = 0;
else
    
    % Roll Equation, Thruster reaction torque is ignored
    pdot = (1/(Ixx-Kpd))*( -(Yvd-Zwd)*v*w - (Yrd+Zqd)*w*r + (Yrd+Zqd)*v*q - ...
    (Mqd-Nrd)*q*r + Kpp*p*abs(p) - (zG*W-zb*B)*cos(theta)*sin(phi) - ...
    (Izz - Iyy)*q*r + m*zG*(w*p+u*r) ) ; 

    % Motions in Horizontal Plane
    % Sway Equation
    vdot =(1/(m-Yvd))*( Yrd + Xud*u*r - Zwd*w*p - Zqd*p*q + ...
    Yvv*v*abs(v) + (W-B)*cos(theta)*sin(phi) + ...
        m*p*w - m*u*r - m*zG*(q*r-pdot) -m*xG*(q*p));       
    
    % Yaw Equation, Thruster reaction torque is ignored
    rdot = (1/(Izz-Nrd))*( Yrd*vdot - (Xud-Yvd)*u*v + Yrd*u*r + Zqd*w*p - ...
    (Kpd-Mqd)*p*q + Nrr*r*abs(r) + Nvv*v*abs(v) - Tstbd*yp + Tport*yp +...
    xG*(W-B)*cos(theta)*sin(phi) - (Iyy - Ixx)*p*q - m*xG*(vdot-w*p+u*r));


end


%% Calculate the rate of change of the Euler angles phi, theta, psi
phidot = p + (sin(phi)*tan(theta))*q + (cos(phi)*tan(theta))*r;
thetadot = (cos(phi)*q) - (sin(phi))*r ;
psidot = (sin(phi)*sec(theta))*q + (cos(phi)*sec(theta))*r;


%% Calculate transformation matrix T
%% and use it to calculate xdot, ydot, and zdot

cphi = cos(phi) ;
ctheta = cos(theta);
cpsi = cos(psi) ;
sphi = sin(phi) ;
stheta = sin(theta);
spsi = sin(psi);

T = zeros(6,6);

T(1,1) = cpsi*ctheta;
T(1,2) = cpsi*stheta*sphi - spsi*cphi;
T(1,3) = spsi*sphi + cpsi*stheta*cphi;
T(2,1) = spsi*ctheta;
T(2,2) = spsi*stheta*sphi + cpsi*cphi;
T(2,3) = -cpsi*sphi + spsi*stheta*cphi;
T(3,1) = -stheta;
T(3,2) = ctheta*sphi;
T(3,3) = ctheta*cphi;

xdot = T(1,1)*u + T(1,2)*v + T(1,3)*w;
ydot = T(2,1)*u + T(2,2)*v + T(2,3)*w;
zdot = T(3,1)*u + T(3,2)*v + T(3,3)*w;

% save thruster values for plotting
Thrust(cur_step,1)= t;
Thrust(cur_step,2) = Tstbd;
Thrust(cur_step,3) = Taft;
Thrust(cur_step,4) = Tport;
Thrust(cur_step,5) = Tfore;

% save commanded values for plotting
Command(cur_step, 1)= t;
Command(cur_step,2) = commanded_depth;
Command(cur_step,3) = commanded_speed;
Command(cur_step,4) = commanded_heading;
Command(cur_step,5) = commanded_pitch;

if (cur_step<max_steps)
    cur_step = cur_step + 1;
else
disp(['max_steps value of ' num2str(max_steps) 'exceeded']);
end

qd = [udot; vdot; wdot; pdot; qdot; rdot; xdot; ydot; zdot; phidot;
thetadot; psidot; Tstbd; Taft; Tport; Tfore];

