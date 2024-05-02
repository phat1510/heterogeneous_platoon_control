function cacc_CL0 = cacc_modeling(L, sys, K, We_e, We_v, Wu, Wz)
% cacc_modeling - Generate an interconnected system for a platoon of
% vehicle
%
% Syntax:  caccCL0 = cacc_modeling(L, sys, K, We_e, We_v, Wu, Wz)
%
% Inputs:
%    L - Description
%    sys - Description
%
% Outputs:
%    output1 - Description
%    output2 - Description
%
% Example: 
%    Line 1 of example
%    Line 2 of example
%    Line 3 of example
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also:

% Author:
% 
% email address: 
% Website: 
% 

%------------- BEGIN CODE --------------

% Input Laplacian matrix
arguments
    L
end

% Inputs plants
arguments (Repeating)
    sys 
    K
    We_e
    We_v
    Wu
    Wz
end

% Check some conditions

% Names assignment
plants = sys;
Gs = {};
for i = 1:length(plants)
    
    Gi = plants{i};
    if i==1
        Gi.u = {'v0','u1'};
        Gi.y = 'x1';
    else
        input_name_1 = 'x';
        input_name_1 = strcat(input_name_1,num2str(i-1),'(2)');
        input_name_2 = 'u';
        input_name_2 = strcat(input_name_2,num2str(i));
    
        output_name = 'x';
        output_name = strcat(output_name,num2str(i));
        
        input_name = {input_name_1; input_name_2};
        Gi.u = input_name;
        Gi.y = output_name;
    end

    Gs{1,i} = Gi;
end

% Build sum blocks in the form of static gain transfer functions 
% using Laplacian
no_of_states = 2; % x = [e v]^T
L_tf =  kron(L(2:length(L),:),eye(no_of_states))*tf(1);
L_tf = L_tf(:,2:size(L_tf,2));

% Sum blocks names assignment
for i = 1:size(L,2)
   if i==1 % For the leading vehicle, only use its velocity
       L_tf(:,1).u = 'v0';
   else
       L_tf(:,no_of_states*i-2:no_of_states*i-1).u = strcat('x',num2str(i-1));
   end
end

for i = 1:(size(L,2)-1) % check the communication with other agent
    L_tf(no_of_states*i-1:no_of_states*i,:).y = strcat('y',num2str(i));
end
L_tf(1,2)=tf(1);

% Controllers names assignment
controllers = K;
Ks = {};
for i = 1:length(controllers)
    Ki = controllers{i};
    
    input_name = 'y';
    input_name = append(input_name,string(i));

    output_name = 'u';
    output_name = append(output_name,string(i));

    Ki.u = input_name;
    Ki.y = output_name;

    Ks{1,i} = Ki;
end

% Names assignment for weighting functions on errors
weigting_we_e = We_e;
weigting_we_v = We_v;
We_s = {};
for i = 1:length(weigting_we_e)
    We_ei = weigting_we_e{i};
    We_vi = weigting_we_v{i};
    We_i  = blkdiag(We_ei,We_vi);
    
    input_name = 'y';
    input_name = append(input_name,num2str(i));

    output_name = 'z';
    output_name = append(output_name,num2str(i));

    We_i.u = input_name;
    We_i.y = output_name;

    We_s{1,i} = We_i;
end

% Names assignment for weighting function on controllers
weighting_wu = Wu;
input_name = {};
for i = 1:length(weighting_wu)
    input_name_i = 'u';
    input_name_i = strcat(input_name_i,num2str(i));
    input_name{i,1} = input_name_i;
end
Wu_s = blkdiag(weighting_wu{1,:});

Wu_s.u = input_name;
Wu_s.y = 'zu';

% Names assignment for weighting function on controllers
weighting_wz = Wz;
input_name = {};
for i = 1:length(weighting_wz)
    input_name_i = 'x';
    input_name_i = strcat(input_name_i,num2str(i),'(2)');
    input_name{i,1} = input_name_i;
end
Wz_s = blkdiag(weighting_wz{1,:});

Wz_s.u = input_name;
Wz_s.y = 'zs';


% Build the interconnected system
T0 = connect(Gs{1,:},Ks{1,:},We_s{1,:},Wu_s,Wz_s,L_tf,{'v0'},{'z1','z2','z3','zu','zs'});
cacc_CL0 = T0;

end

