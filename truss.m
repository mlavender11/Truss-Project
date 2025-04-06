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

% Loops over the joints
for j = 1:J
    fprintf('Joint: %d\n', j)

    % Sets up the row vector to be applied to row k of A
    xforces = zeros(1, M);
    yforces = zeros(1, M);

    % Finds the members connected to this joint
    members = find(C(j,:));
    disp(members)

    % Loops for each member connected to this joint
    for m = members
        disp(m)

        % Finding what other joint this member is connected to
        joints = find(C(:,m));

        if joints(1) == j
            j2 = joints(2);
        else
            j2 = joints(1);
        end

        % X and Y coordinates of the joints
        Jc = XY(:, j);
        j2c = XY(:, j2);

        % Distance between joints
        r = sqrt((Jc(1) - j2c(1))^2 + (Jc(2) - j2c(2))^2);

        xforces(m) = (j2c(1)-Jc(1))/r;
        yforces(m) = (j2c(2)-Jc(2))/r;
    end

    A(j, :) = xforces;
    A(j + J, :) = yforces;
end


% Truncate Sx and Sy onto A
A = [A Sc];

T = A\L;


member_forces = T(1:M);
reactions = T(M+1:end);

% Display results
fprintf('\nMember forces:\n');
for m = 1:M
    if member_forces(m) >= 0
        fprintf('m%d: %.2f oz (T)\n', m, member_forces(m));
    else
        fprintf('m%d: %.2f oz (C)\n', m, -member_forces(m));
    end
end

fprintf('\nReaction forces:\n');
fprintf('Sx1: %.2f oz\n', reactions(1));
fprintf('Sy1: %.2f oz\n', reactions(2));
fprintf('Sy2: %.2f oz\n', reactions(3));

% Cost
total_length = 0;
for m = 1:M
    joints = find(C(:,m));
    L_m = sqrt((X(joints(2))-X(joints(1)))^2 + (Y(joints(2))-Y(joints(1)))^2);
    total_length = total_length + L_m;
end
cost = 10*J + 1*total_length;
fprintf('\nTruss cost: $%.2f\n', cost);
