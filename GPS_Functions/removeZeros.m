function [newPsr,newCno,newDop,newCarr,newPrn] = removeZeros(psr,cno,dop,carr,prn)
%REMOVEZEROS Summary of this function goes here
%   Detailed explanation goes here


newPsr = [];
newCno = [];
newDop = [];
newCarr = [];
newPrn = [];
    for i = 1:length(prn)
        if((prn(i) ~= 0) && prn(i) ~= 23)
            if(i == 1)
                newPsr = [newPsr;psr(i)];
                newCno = [newCno;cno(i)];
                newDop = [newDop;dop(i)];
                newCarr = [newCarr;carr(i)];
                newPrn = [newPrn;prn(i)];
            elseif(prn(i) ~= prn(i-1))
                newPsr = [newPsr;psr(i)];
                newCno = [newCno;cno(i)];
                newDop = [newDop;dop(i)];
                newCarr = [newCarr;carr(i)];
                newPrn = [newPrn;prn(i)];
            end
        end
    end
end

