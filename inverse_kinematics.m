function thetad = inverse_kinematics(u)
% performs inverse kinematics computations with various methods

% Stefan Schaal, March 2009

% split the input u into meaningful quantities

n = (length(u)-2)/2;

theta = u(1:n);
xd    = u(n+1:n+2);
links = u(n+3:end);

n = length(theta);
m = length(xd);


% NOTE: insert your Jacobian calculation here

sumtheta=zeros(1,n);
for i=1:n
    sumtheta(1,i+1)=theta(i)+sumtheta(1,i);
end
sumtheta(:,1)=[];
pn=zeros(3,n);
for j =1:n
    pn(1,j+1)=links(j)*cos(sumtheta(j))+pn(1,j);
    pn(2,j+1)=links(j)*sin(sumtheta(j))+pn(2,j);
end
pn(:,1)=[];
z=[0;0;1];
J=zeros(3,n,n);
for x =1:n
    J(:,1,x) = cross(z,pn(:,x)-[0;0;0]);
    for y =2:x
        J(:,y,x) = cross(z, pn(:,x)-pn(:,y-1));
    end
end
J=1/n*sum(J,3);
J(3,:)=[];
  
% convert cartesian velocities into joint velocities

% NOTE: insert the required inverse kinematics methods at this
%       location

%Part g)
%a=1;
%thetad = a*J'*xd;

%Part h)
%a=0.01
%thetad = a*J'*inv(J*J')*xd;

%Part i)
%a=0.01
% theta_o = 0.1*ones(n,1);
% J_pinv = J' * inv(J*J');
% thetad = a * J_pinv*xd +(eye(n) - J_pinv*J)*(theta_o -theta);

%Part j)
%a=0.01
%w=[1.0 0.5 0.1 0.01];
%w=diag(w);
%thetad=a*inv(w)*J'*inv(J*inv(w)*J')*xd;

%Part k) 
%a=0.01 
% theta_o=0.1*ones(n,1);
% J_wpinv = inv(w)* J'*inv(J*inv(w)*J');
% thetad = a*J_wpinv*xd +(eye(n)-J_wpinv*J)*(theta_o-theta);
