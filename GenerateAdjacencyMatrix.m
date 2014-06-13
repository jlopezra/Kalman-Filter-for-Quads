function Adj=GenerateAdjacencyMatrix(Adj,Zlm,Zq,Zr,qcount,Nq)
% This Function initially contains a 7 by 7 matrix full of zeros, and it adds a 1 to each element of the matrix when measurements have been taken from the other quadcopters or landmarks.
% Exp: 
% Quad Making measurements	Q1	Q2	Q3 	Q4	Q5	L1	L2
%                       Q1	1	0	0	1	0	1	1
%                       Q2	1	1	0	0	0	0	1
%                       Q3	0	0	1	0	1	1	0
%                       Q4	0	1	0	1	0	0	0
%                       Q5	0	0	1	0	1	0	1
% 	                        0	0	0	0	0	0	0
% 	                        0	0	0	0	0	0	0
% 
% This Adjacency matrix means that Quadrotor 1 is getting measurements from Quad4 and both landmarks and Quad4 is getting measurements from Q2 and no landmarks.

if(isempty(Zq)==0)
    [m,n]=size(Zq);
    for i=1:m
        Adj(qcount,Zq(i,n))=1;
    end
end
if(isempty(Zlm)==0)
    [m,n]=size(Zlm);
    for i=1:m
        Adj(qcount,Nq+Zlm(i,n))=1;
    end
end