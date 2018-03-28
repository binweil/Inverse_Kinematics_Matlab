function [sys,x0]=ArmAnimation(t,x,u,flag,ts,theta0,links);
% S-function for animating the motion of a planar arm.
%
%   Gaurav & Stefan, Jan. 1999
%

global ArmAnimation;

n_joints = length(theta0);
theta=u;

if flag==2, % update the animation
	if any(get(0,'Children')==ArmAnimation),
		if strcmp(get(ArmAnimation,'Name'),'Arm Animation'),
			set(0,'currentfigure',ArmAnimation);
			hndlList=get(gca,'UserData');
			
			n_joints = length(links);
			
			sum_thetas = 0;
      sum_x = 0;
      sum_y = 0;
			X=zeros(n_joints+1,1);
			Y=zeros(n_joints+1,1);
			for i=1:n_joints,
				sum_thetas = sum_thetas + theta(i);
				X(i+1) = X(i)+cos(sum_thetas)*links(i);
        sum_x = sum_x + X(i+1);
				Y(i+1) = Y(i)+sin(sum_thetas)*links(i);
        sum_y = sum_y + Y(i+1);
				if mod(t,ts)==0,
					set(hndlList(i),'XData',X(i:i+1),'YData',Y(i:i+1));
				end;
			end;
			if mod(t,ts)==0,
			  set(hndlList(n_joints+1),'XData',sum_x/n_joints,'YData',sum_y/n_joints);
      end
			if mod(t,ts)==0,
				drawnow;
			end;
		end
	end
  sys=[];
	
elseif flag == 3 % Return output

	sys=[];
	
elseif flag == 4 % Return next sample hit

	sys = [];
	
elseif flag==0,

  % Initialize the figure for use with this simulation
  ArmAnimation = animinit('Arm Animation');
  set(ArmAnimation,'HandleVisibility','on');
  axis([-.7 0.7 -.7 0.7]);
  hold on;
	theta = theta0;
	
	sum_thetas = 0;
  sum_x = 0;
  sum_y = 0;
	X=zeros(n_joints+1,1);
	Y=zeros(n_joints+1,1);
	for i=1:n_joints,
		sum_thetas = sum_thetas + theta(i);
		X(i+1) = X(i)+cos(sum_thetas)*links(i);
    sum_x = sum_x + X(i+1);
		Y(i+1) = Y(i)+sin(sum_thetas)*links(i);
    sum_y = sum_y + Y(i+1);
		hndlList(i)=plot(X(i:i+1),Y(i:i+1),'LineWidth',5,'Color',[1,1,1]*0.1*i,'EraseMode','background');
	end;
  
  hndlList(n_joints+1) = plot(sum_x/n_joints, sum_y/n_joints,'LineWidth',10,'Color',[1,0,0],'EraseMode','background');

  set(gca,'DataAspectRatio',[1 1 1]);
  set(gca,'UserData',hndlList);
	
  sys=[0 0 0 n_joints 0 0]; % the non-zero number indicates the number of inputs
  x0=[];
  
  hold off;
  drawnow
  flag
	
end
