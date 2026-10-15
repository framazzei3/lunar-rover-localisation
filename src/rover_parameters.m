function p = rover_parameters()
%ROVER_PARAMETERS Problem data and tuning parameters, in one place.
%   Units: km, h, rad, unless stated otherwise.
%   Problem data come from the assignment (docs/assignment.pdf); the
%   planning, control and simulation settings are design choices.

%% Rover and mission (assignment)
p.L        = 1e-3;              % wheel base [km] (1 m)
p.v_max    = 2;                 % maximum speed [km/h] (Lunokhod)
p.start    = [-6; 21; 0.41];    % initial pose [x; y; theta] [km, km, rad]
p.target1  = [2.8; -14];        % first target [km]
p.target2  = [-1.6; -29.7];     % second target [km]

% Ridges as horizontal segments, one per row: [x_start, x_end, y] [km]
p.ridges = [ -5  2   11
             11 25    9
             -9  8   -3
             10 25  -14];

% Known landmarks [km], first row x, second row y
p.landmarks = [ -1.4  4.88 7.58 14.74 12.76  5.65   8.9   7.4  -1.2 -14 -6.4 -8.2 -14.73 -22.03  -3.8 11.6 -23.8
                15.77 12.23 6.41  0.85 -7.9 -13.8 -18.8 -26.5 -31.3  14  0   -7.8 -22.1  -13.37 -14.7 21.5   4.3];

% Lunar map image and the area it covers [km]
p.map_file = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'data', 'lunarmap.jpg');
p.map_x    = [-27 30];
p.map_y    = [-55 37];

%% Path planning (design)
% Via points on landmarks between the hazards, one set per leg [km]
p.via1 = p.landmarks(:, 1:6);   % start to target 1
p.via2 = p.landmarks(:, 7:9);   % target 1 to target 2

p.speed_fraction = 0.95;        % reference speed as a fraction of v_max
p.blend_time     = 0.5;         % duration of each parabolic blend [h]
p.ref_dt         = 1e-3;        % time step of the reference trajectory [h]

%% Guidance and control (design)
p.lead_distance   = 0.5;        % distance kept behind the moving reference [km]
p.Kv              = 2;          % proportional gain on speed [1/h]
p.Ki              = 0.1;        % integral gain on speed [1/h^2]
p.Kh              = 0.01;       % proportional gain on steering [-]
p.approach_radius = 0.8;        % switch to move to a point below this distance [km]
p.arrival_tol     = 0.01;       % target reached below this distance [km] (10 m)
p.max_extra_time  = 2.5;        % time allowed after the reference ends [h]
p.dwell_time      = 0.5;        % stop at target 1 before the second leg [h]

%% Odometry (assignment noise; sampling is a design choice)
p.odo_dt   = 0.08;              % odometry sampling period [h] (4.8 min)
p.sigma_d  = 3e-5;              % noise on the travelled distance [km] (3 cm)
p.sigma_ha = deg2rad(0.8);      % noise on the heading angle [rad]

% Initial covariance (assignment)
p.P0 = diag([5e-5, 5e-5, deg2rad(0.001)].^2);

%% Lidar terrain mapper (assignment)
p.sigma_r     = 2e-5;           % range noise [km] (2 cm)
p.sigma_b     = deg2rad(0.3);   % bearing noise [rad]
p.lidar_range = 3;              % maximum detection distance [km]

%% Filter tuning (design)
p.filter      = 'oc2';          % Task 3 filter, see ekf_localisation
p.underweight = 1;              % measurement underweighting factor (0 = off);
                                % 1 gave the best accuracy with a median
                                % NEES at the expected value (monte_carlo.m)

%% Analysis
p.confidence = 0.95;            % probability inside the uncertainty ellipses
p.seed       = 1000;            % random seed, for reproducible noise
end
