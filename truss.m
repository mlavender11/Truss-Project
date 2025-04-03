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
for k = 1:J

    % Sets up the row vector to be applied to row k of A
    xforces = zeros(1, M);
    yforces = zeros(1, M);

    % Finds the members connected to this joint
    members = find(C(k,:));

    % Loops for each member connected to this joint
    for z = members

        % Finding what other joint this member is connected to
        joints = find(C(:,z));

        if joints(1) == J
            j2 = joints(2);
        else
            j2 = joints(1);
        end

        % X and Y coordinates of the joints
        Jc = XY(:, J);
        j2c = XY(:, j2);

        % Distance between joints
        r = sqrt((Jc(1) - j2c(1))^2 + (Jc(2) - j2c(2))^2);

        xforces(z) = (j2c(1)-Jc(1))/r;
        yforces(z) = (j2c(2)-Jc(2))/r;
    end

    A(k, :) = xforces;
    A(k + J, :) = yforces;
end


% Truncate Sx and Sy onto A
A = [A Sc];

T = A\L;