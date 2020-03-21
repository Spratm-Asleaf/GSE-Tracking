% Source: https://github.com/Spratm-Asleaf/GSE-Tracking
% Author: WANG Shixiong (s.wang@u.nus.edu)
% Affiliate: Department of Industrial SystemsEngineering and Management, National University of Singapore, Singapore 117576


% 2. Update Model Probability
function [u]=Model_P_Update(r1,r2,r3,r4,S1,S2,S3,S4,c_j)
	%u  Model Probability
	%r1 Prediction Error of Model 1
	%r2 Prediction Error of Model 2
	%r3 Prediction Error of Model 3
	%r4 Prediction Error of Model 4

	%S1 Prediction Error Covariance of Model 1
	%S2 Prediction Error Covariance of Model 2
	%S3 Prediction Error Covariance of Model 3
	%S3 Prediction Error Covariance of Model 4
	%c_j  Model Probability
	 
	%Likelihood of Each Model
	Lfun1=(1/sqrt(abs(2*pi*(det(S1)))))*exp((-1/2)*(r1'*inv(S1)*r1));    %Lfun1=1/(r1'*inv(S1)*r1); 
	Lfun2=(1/sqrt(abs(2*pi*(det(S2)))))*exp((-1/2)*(r2'*inv(S2)*r2));    %Lfun2=1/(r2'*inv(S2)*r2); 
	Lfun3=(1/sqrt(abs(2*pi*(det(S3)))))*exp((-1/2)*(r3'*inv(S3)*r3));    %Lfun3=1/(r3'*inv(S3)*r3);
	Lfun4=(1/sqrt(abs(2*pi*(det(S4)))))*exp((-1/2)*(r4'*inv(S4)*r4));    %Lfun4=1/(r4'*inv(S4)*r4);
	 
	%Normized
	Lfun11=Lfun1/(Lfun1+Lfun2+Lfun3+Lfun4);
	Lfun21=Lfun2/(Lfun1+Lfun2+Lfun3+Lfun4);
	Lfun31=Lfun3/(Lfun1+Lfun2+Lfun3+Lfun4); 
	Lfun41=Lfun4/(Lfun1+Lfun2+Lfun3+Lfun4); 

	%Update Model Probability
	c=[Lfun11,Lfun21,Lfun31,Lfun41]*c_j;
	u=(1/c).*[Lfun11,Lfun21,Lfun31,Lfun41]'.*c_j; 
end