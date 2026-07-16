% Project Capture
% Filipe Cavalheiro (fs.cavalheiro@campus.fct.unl.pt)
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)

clear; clc; close all;
addpath(genpath(pwd)); % load all sub folders
drone_init;            % initialize variables

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fibonacci sphere values
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fib_ref_points = get_fibonacci_points(Param.nD, 1, [0;0;0]);

% safe distance is half of the average distance between fib_ref_points 
min_safe_dist = get_min_safe_distance(fib_ref_points, Param)*0.5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Assign each agent to nearest ref point
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute geodesic distances between all pairs
pos_on_sphere = projectToSphere(initial_pos, Param.sphere_radius, Param.initial_target_pos);
geo_distances_all = zeros(Param.nD, Param.nD);
for i = 1:Param.nD
    for j = 1:Param.nD
        a = fib_ref_points(:,i) * Param.sphere_radius;
        b = pos_on_sphere(:,j);
        dot_product = dot(a, b-Param.initial_target_pos);
        geo_distances_all(i, j) = Param.sphere_radius * acos(dot_product / (Param.sphere_radius^2));
    end
end

%the theoretical maximum two points will be apart is pi so 10pi is just to
%force full one-to-one assignment
costofnonassignment = 10*pi*Param.sphere_radius;
[assignments, unassignedrows, unassignedcolumns] = ...
    assignmunkres(geo_distances_all, costofnonassignment);

if ~isempty(unassignedrows) || ~isempty(unassignedcolumns)
    disp("ERROR: impossible to assing 1-to-1");
    exit;
end

organized_initial_pos = zeros(3,Param.nD);
organized_Rad = zeros(1,Param.nD);
for i = 1:Param.nD
    organized_initial_pos(:,i) = initial_pos(:,assignments(i,1));
    organized_Rad(i) = Param.r0(assignments(i,1));
end

pair_idx = nchoosek(1:Param.nD,2);
nPairs = size(pair_idx,1);

geo_distances_hist = zeros(nPairs,Nsim);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation initialization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for iD = 1:Param.nD
    % initial values
    Rad{iD} = organized_Rad(iD);
    chaser{iD}(1:3,1) = organized_initial_pos(:,iD);
    q{iD} = organized_initial_pos(:,iD);
    omega{iD} = [0; 0; 0];
    
    % desired values
    q_d{iD} = fib_ref_points(:,iD);
    
    omega_d{iD} = [0; 0; 0];
end

% Storage
for iD = 1:Param.nD
    q_hist{iD} = zeros(3, Nsim);
    Rad_hist{iD} = zeros(1, Nsim);
end
c_sphere = zeros(3,Nsim);
target_hist = zeros(18,Nsim);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Target drone initialization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial target state
target.x = zeros(18,1);
target.x(1:3)   = Param.initial_target_pos;    % position
target.x(4:6)   = [0;0;0];                     % velocity
target.x(7:15)  = reshape(eye(3),[],1);        % rotation matrix
target.x(16:18) = [0;0;0];                     % angular velocity

% Integral error (if you want to reuse Mellinger)
target.xiep = zeros(3,1);

current_pos = zeros(3, Param.nD);
for iD = 1:Param.nD
        q_s  = q{iD} - c_sphere(:,1);
        q_hat  = q_s/ norm(q_s);
        current_pos(:,iD) = q_hat;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main time loop for simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k = 1:Nsim
    fprintf("Computing simulation interation: %04d\n", k);
    
    %move target
    p_d_target = target_ref(Param.ref_target_traj, target.x(1:3));
    [target, c_sphere(:,k)] = target_motion(target, p_d_target, Param);
    target_hist(:,k) = target.x;

    % Collect current drone positions
    current_points = zeros(3, Param.nD);
    for ii = 1:Param.nD
        current_points(:,ii) = (chaser{ii}(1:3,k)-c_sphere(:,k))/norm(chaser{ii}(1:3,k)-c_sphere(:,k));
    end

    % Compute geodesic distance matrix
    geo_matrix = geo_distances(current_points, 1);

    % Store only unique pair distances
    for p = 1:nPairs
        i = pair_idx(p,1);
        j = pair_idx(p,2);

        geo_distances_hist(p,k) = geo_matrix(i,j);
    end
    
    for iD = 1:Param.nD
        % get state vector
        pos{iD} = chaser{iD}(1:3,k);
        v{iD} = chaser{iD}(4:6,k);
        R = reshape(chaser{iD}(7:15,k),3,3);

        om = chaser{iD}(16:18,k);

        % ---------- Shift to sphere frame (back to origin)----------
        q_s  = pos{iD} - c_sphere(:,k);
        q_hat  = q_s/ norm(q_s);

        % Position reference generation
        [q_hat, e_q] = linear_motion_ref(q_hat, q_d{iD}, Param);

        % Position reference shift to safe guard collisions
        current_pos(:, iD) = q_hat;
        q_hat = APF_safe_guard(iD, current_pos, min_safe_dist, Param);

        Rad{iD} = update_radius(Rad{iD}, e_q, Param);

        % ---------- Enforce S² constraint (local) ----------
        q{iD} = Rad{iD} * q_hat + c_sphere(:,k);
            
        % --- Store ---
        Rad_hist{iD}(k) = Rad{iD};
        q_hist{iD}(:,k) = q{iD};

        % get reference
        p_d =  q{iD};
        v_d = zeros(3,1);
        a_d = zeros(3,1);
        j_d = zeros(3,1);
        psi_d = 0;
        dpsi_d = 0;
    
        % Mellinger controller
        [T{iD}(:,k),tau{iD}(:,k),e_p] = drone_mellinger_ctrl(pos{iD},...
            v{iD},R,om,Param,p_d,psi_d,xiep{iD}(:,k),v_d,dpsi_d,a_d,j_d);
        
        % Integrate position error
        xiep{iD}(:,k+1) = xiep{iD}(:,k) + Param.dTi*e_p;
        
        % nonlinear drone model (continuous time)
        [dot_p,dot_v,dot_R,dot_om] = drone_3dfull_dyn(v{iD},R,om,T{iD}(:,k),tau{iD}(:,k),Param);
        
        % discretization
        pp = pos{iD} + Param.dTi*dot_p;
        vp = v{iD} + Param.dTi*dot_v;
        Rp = rot_integrate(R,om,Param.dTi);
        omp = om + Param.dTi*dot_om;
        chaser{iD}(:,k+1) = [pp;vp;reshape(Rp,[],1);omp];
    
        % auxiliary drone attitude computation from rotation matrix
        lbd{iD}(:,k) = R2Euler(R);
    
        if abs(1-norm(Rp'*Rp))>1e-4
            warning(['Problems with rotation matrix integration... stoping simulation: t = ' num2str(k*Param.dTi) ' s.']);
            break;
        end
    end
end

%% animations PDF and mp4
close all; clc;
create_figures(chaser, target_hist(1:3,:),...
     cell2mat(q_d), Param);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot pairwise distances
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

time = (0:Nsim-1) * Param.dTi;

figure;
hold on;
grid on;

for p = 1:nPairs
    i = pair_idx(p,1);
    j = pair_idx(p,2);

    plot(time, geo_distances_hist(p,:));
end

yline(min_safe_dist, '--r', 'Minimum Safe Distance', ...
    'LineWidth', 2);
ylim([0 pi])

xlabel('Time [s]');
ylabel('Geodesic Distance [m]');
print2pdf(sprintf("intra_agent_dist_apf_%d", Param.nD), 1);

% clean path after it runs so that I can modify folders
rmpath(genpath(pwd))