clear
clc
load("inputs.mat")
Sc = [Sx; Sy];
% Combines X and Y coordinates
XY = [X; Y];
% Number of joints and members
[J, M] = size(C);
% Creates matrix A
A = zeros(2*J, M);
fprintf('\nMember Lengths:\n');
member_lengths = zeros(1, M);  % store lengths for later
for m = 1:M
    joints = find(C(:,m));
    if length(joints) ~= 2
        fprintf('m%2d: Invalid connection (%d joints)\n', m, length(joints));
    else
        j1 = joints(1);
        j2 = joints(2);
        dx = X(j2) - X(j1);
        dy = Y(j2) - Y(j1);
        L_m = sqrt(dx^2 + dy^2);
        member_lengths(m) = L_m;
        fprintf('m%2d: %.2f inches\n', m, L_m);
    end
end
% Assemble equilibrium matrix A
for j = 1:J
    xforces = zeros(1, M);
    yforces = zeros(1, M);
    members = find(C(j,:));
    for m = members
        joints = find(C(:,m));
        j2 = joints(joints ~= j);  % the other joint
        Jc = XY(:, j);
        j2c = XY(:, j2);
        r = norm(j2c - Jc);
        xforces(m) = (j2c(1) - Jc(1)) / r;
        yforces(m) = (j2c(2) - Jc(2)) / r;
    end
    A(j, :) = xforces;
    A(j + J, :) = yforces;
end
% Add support constraints
A = [A Sc];
% Solve system
T = A \ L;
member_forces = T(1:M);
reactions = T(M+1:end);
% Compute max load each member can hold
max_loads = zeros(1, M);
Wj = find(L);
W = L(Wj);  % original applied load
for m = 1:M
    if member_forces(m) >= 0
        Wf = inf;  % in tension or ZFM
    else
        R = member_forces(m) / W;
        L_m = member_lengths(m);
        P = 2390.012 * (L_m^-1.811);
        Wf = -P / R;
    end
    max_loads(m) = Wf;
end
% Find max load before first member buckles
max_load = min(max_loads);
fail_member = find(max_loads == max_load);
scale_factor = max_load / W;
% Display results
fprintf('\nMember forces:\n');
for m = 1:M
    if member_forces(m) == 0
        fprintf('m%2d: %d oz (ZFM)', m, member_forces(m));
    elseif member_forces(m) > 0
        fprintf('m%2d: %.2f oz (T)', m, member_forces(m));
    else
        fprintf('m%2d: %.2f oz (C)', m, -member_forces(m));
    end
    fprintf(' – Breaking Load: %.2f oz\n', max_loads(m));
end
fprintf('\nReaction forces:\n');
fprintf('Sx1: %.2f oz\n', reactions(1));
fprintf('Sy1: %.2f oz\n', reactions(2));
fprintf('Sy2: %.2f oz\n', reactions(3));
% Cost
total_length = sum(member_lengths);
cost = 10*J + total_length;
fprintf('\nTruss Cost: $%.2f\n', cost);
fprintf('\nMaximal Load: %.2f oz\n', max_load);
fprintf('First Member to Buckle: Member(s) %s\n', mat2str(fail_member));
fprintf('\nLoad:Cost Ratio = %.2f oz/$\n', max_load / cost);
% Additional: Member forces at max load
fprintf('\nMember forces at Max Truss Load (%.2f oz):\n', max_load);
for m = 1:M
    F_max = member_forces(m) * scale_factor;
    if abs(F_max) < 1e-6
        force_type = 'ZFM';
    elseif F_max > 0
        force_type = 'T';
    else
        force_type = 'C';
    end
    fprintf('m%2d: %7.2f oz (%s)\n', m, abs(F_max), force_type);
end
% Buckling strength + uncertainty (for compressive members only)
fprintf('\nBuckling Strengths (Compressive Members Only):\n');
for m = 1:M
    if member_forces(m) < 0
        L_m = member_lengths(m);
        Pcrit = 2390.012 * (L_m^-1.811);
        uncertainty = 0.10 * Pcrit;  % 10% assumed uncertainty
        fprintf('m%2d: Pcrit = %.2f oz ± %.2f oz (Length = %.2f in)\n', ...
            m, Pcrit, uncertainty, L_m);
    end
end