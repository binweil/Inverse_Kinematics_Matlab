%FORWARD KINEMATICS

function [x] = forward_kinematics(u)
% [x] = forward_kinematics(u,th)
% Calculates the forward kinematics for a given th and link lengths

% 	Stefan Schaal, Jan 99
n_joints = length(u)/2;
th = u(1:n_joints);
links = u(n_joints+1:end);

sum_ths = cumsum(th);
links_sin_sum_ths = links.*sin(sum_ths);
links_cos_sum_ths = links.*cos(sum_ths);

x = zeros(2,1);

for i=1:n_joints,
  x = x + [sum(links_cos_sum_ths(1:i));sum(links_sin_sum_ths(1:i))];
end

x = x/n_joints;