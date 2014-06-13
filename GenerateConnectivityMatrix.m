function Conn=GenerateConnectivityMatrix(Adj,k)
[m,n]=size(Adj);
Conn=eye(m);
for i=1:k
    Conn=Conn+Adj^i;
end
