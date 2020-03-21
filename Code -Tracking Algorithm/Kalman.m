% Source: https://github.com/Spratm-Asleaf/GSE-Tracking
% Author: WANG Shixiong (s.wang@u.nus.edu)
% Affiliate: Department of Industrial SystemsEngineering and Management, National University of Singapore, Singapore 117576


%Kalman Filtering
function [X,P,e,S,a_avrg]=Kalman(X_Forward,P_Forward,Z,A,G,Q,H,R,isCS,G2,a_avrg,a,T)
%Inputs Parameters
%       Z--Y in Eq. (2) and (3)
%       A--\Phi in Eq. (2) and (3)
%       G--G in Eq. (2) and (3)
%       Q--Covirance of W in Eq. (2) and (3)
%       H--H in Eq. (2) and (3)
%       R--Covirance of V in Eq. (2) and (3)
%       X_Forward--X^t in Eq. (2) and (3)
%       P_Forward--X^t in Eq. (2) and (3)
%Outputs Parameters
%       X--estimate of X^t in Eq. (2) and (3)
%       P--estimate error covariance of X^t
%       e--innovation
%       S--innovation covariance
 
	% Time Update
    if isCS
        a_avrg = (1 - exp(-a*T))*a_avrg + exp(-a*T)*X_Forward([3;6]);
        X_Pre=A*X_Forward + (G2-G)* a_avrg;
    else
        X_Pre=A*X_Forward;
    end
    
    P_Pre=A*P_Forward*A'+G*Q*G';

    % Kalman gain
    K=P_Pre*H'*inv(H*P_Pre*H'+R)';  
	
	% innovation and its covirance
    e = Z-H*X_Pre;	
    S=H*P_Pre*H'+R;

    % Measurement Update
    X=X_Pre+K*(Z-H*X_Pre);

    M=K*H; 
    n=size(M);
    I=eye(n); 
    P=(I-K*H)*P_Pre*(I-K*H)'+ K*R*K';
end