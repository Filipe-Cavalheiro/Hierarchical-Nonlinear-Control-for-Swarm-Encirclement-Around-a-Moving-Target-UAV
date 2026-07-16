% Project Capture
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)

% inicializations

clear all;

% Model and simulation parameters
Param.Tend = 30;
Param.dTi = 0.01;  % inner-loop and simulation sampling period
Nsim = round(Param.Tend/Param.dTi)+1;
Param.Nsim = Nsim;
Param.g = 9.81;     % earth gravity
Param.nD = 3; % number of drones
Param.sphere_radius = 2;  % sphere radius
Param.arena = 'free_world'; % simulation arena: 'lab', 'free_world'

Param.k_rad = 0.3; % radius reduction gain
Param.k_geometric = 5;
Param.k_apf = 1;

t = linspace(0, Param.Tend, 200);
Param.ref_target_traj  = [ ...
    2*(t<=8).*sin(t*pi/8) + (t>8).*0; ...
    2*(t<=8).*cos(t*pi/8) + (t>8).*(-2); ...
    1.5*ones(size(t)) ...
];
Param.initial_target_pos = Param.ref_target_traj(:,1);

% === Animation ===
Param.verbose = false;
Param.animation.animate = true;
Param.animation.trail_length = 0;
Param.animation.step = 50;
% === Create PDF ===
Param.animation.composed = true;
Param.animation.radial_hist = true;
Param.animation.create_pdf = true;
% === Video settings ===
Param.animation.save_to_mp4 = false;
Param.animation.frameStep = 10;
Param.animation.sim_dt = Param.dTi * Param.animation.frameStep;
Param.animation.videoFPS = 1 / Param.animation.sim_dt;   

% M690B drone 
% (guessing parameters! needs identification or estimation from geometry)
% from https://doi.org/10.23919/ACC45564.2020.9147948 we have: "The quadrotor’s mass is M = 1.73Kg and its arms length is L = 0.2m. The moments of inertia matrix used in the nonlinear model (3) is considered diagonal with values: Ixx0=Iyy0=0.03Kg.m2 and Izz0=0.04Kg.m2. "
% from https://dspace.mit.edu/handle/1721.1/106777 we have as the result of a bifilar pendulum experiment: "mass 𝑚 = 1.28 kg; inertia (𝐽𝑥𝑥, 𝐽𝑦𝑦, 𝐽𝑧𝑧) = (6.9𝐸−3, 7.0𝐸−3, 12.4𝐸−3) Ns2; propeller diameter 0.165 m; propeller positions [±0.117 ±0.117 −0.012] m
% from https://doi.org/10.1016/j.conengprac.2010.02.008 we have: m = 4.34±5×10−3 kg; IXX = 0.0820±0.0025 kg m2, IYY = 0.0845±0.0029 kg m2, IZZ = 0.1377±0.0059 kg m2; rotor radius r = 0.165 ±0.5×10−3   m; arm length 0.157 m, height 0.035 m.
% from https://doi.org/10.1109/CEIT.2018.8751873 we have: m = 7.293 kg; Ixx = 0.13, Iyy = 0.25, Izz = 0.55 kg m^2; propeller radius 0.41 or 0.2 m; arm radius 0.55 or 0.45 m.
Param.m = 5;        % drone mass
Param.I = diag([2e-2,2e-2,3e-2]);  % inertia tensor
Param.D = 0.00;     % frame drag coeficient
Param.kp = diag([150,150,150]);
Param.kv = diag([15,15,15]);
Param.ki = diag([0,0,0]);
Param.kR = diag([8,8,8]);
Param.kom= diag([0.5,0.5,0.5]);

% Encirclement position
if strcmp(Param.arena, 'free_world')
    if Param.nD <= 9
        theta0   = [0.175 0.698 1.222 1.745 2.269 2.793 3.491 4.189 4.712];
        phi0     = [1.484 1.222 1.396 1.047 1.484 1.222 0.698 1.396 0.960];
        Param.r0 = [6 8 7 20 15 30 25 20 15];
    else
        theta0   = 2*pi*rand(1,Param.nD);
        phi0     = 2*pi*rand(1,Param.nD);
        Param.r0 = 15*rand(1, Param.nD);
    end 
    
    initial_pos = Param.r0.* [sin(theta0).*cos(phi0);
                    sin(theta0).*sin(phi0);
                    cos(theta0)] + Param.initial_target_pos;
elseif strcmp(Param.arena, 'lab')
    initial_pos = [-1, -1, -1, 0, 0, 0, 1, 1, 1;...
                   -1, 0, 1, -1, 0, 1, -1, 0, 1;...
                   0, 0, 0, 0, 0, 0, 0, 0, 0;];
    
    initial_pos = initial_pos(:, 1:Param.nD);
    
    Param.r0 = sqrt(sum((initial_pos - repmat(target_ref(0),1,Param.nD)) .^ 2));
else
    error("One of the arenas must be chosen either: Param.arena='lab' or Param.arena='free_world'");
end

% initialize variables for all drones:
t = 0:Param.dTi:Param.Tend;
nt = length(t);
nx = 18;
nu = 4;
for iD = 1:Param.nD
    % set initial conditions
    p0{iD} = initial_pos(:, iD);
    v0{iD} = [0;0;0];
    psi0{iD} = pi/20*((Param.nD-1)/2-iD+1);
    R0{iD} = Euler2R([0;0;psi0{iD}]);
    om0{iD} = [0;0;0];
    chaser{iD} = zeros(nx,Nsim+1);
    xiep{iD} = zeros(3,Nsim+1);
    T{iD} = zeros(1,Nsim);
    tau{iD} = zeros(3,Nsim);
    lbd{iD} = zeros(3,Nsim);
    chaser{iD}(:,1) = [p0{iD};v0{iD};reshape(R0{iD},[],1);om0{iD}];
end
