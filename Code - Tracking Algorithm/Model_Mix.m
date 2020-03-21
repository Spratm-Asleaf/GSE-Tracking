% Source: https://github.com/Spratm-Asleaf/GSE-Tracking
% Author: WANG Shixiong (s.wang@u.nus.edu)
% Affiliate: Department of Industrial SystemsEngineering and Management, National University of Singapore, Singapore 117576


function [x_pro,P]=Model_Mix(x1,x2,x3,x4,P1,P2,P3,P4,u)
	x_pro=x1*u(1)+x2*u(2)+x3*u(3)+x4*u(4); 
	P=(P1+[x1-x_pro]*[x1-x_pro]')*u(1)+...
	(P2+[x2-x_pro]*[x2-x_pro]')*u(2)+...
	(P3+[x3-x_pro]*[x3-x_pro]')*u(3)+...
	(P4+[x4-x_pro]*[x4-x_pro]')*u(4);
end