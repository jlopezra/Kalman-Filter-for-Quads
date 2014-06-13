function Conn_vec=GenerateConnVec(Conn,Nq,N)
Conn_vec=zeros(Nq,1);
for i=1:Nq
    for j=1:N
        if(Conn(i,Nq+j)>0)
          Conn_vec(i,1)=Conn_vec(i,1)+1;
        end
    end
end
