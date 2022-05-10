function [recoveredPrn] = recoverPRN(prnData)
%RECOVERPRN Summary of this function goes here
%   Detailed explanation goes here
[rows,cols] = size(prnData);
quitFlag = false;
for i = 1:rows
    for j = 1:cols 
        if(rem(abs(prnData(i,j)),1) > .2 && (round(prnData(i,j)) - prnData(i,j)) > .1)
            %remediate data!!!!!!!
            itr = 1;
            if(i == 539 && j == 5)
                disp('whoop')
            end
            if(i+itr > rows)
                quitFlag = true;
            end
            if(quitFlag == false)
                while(rem(abs(prnData(i+itr,j)),1) > .2 && quitFlag == false) 
                    itr = itr + 1;
                        if(i+itr > rows)
                            quitFlag = true;
                            break
                        end
                end
            end
            if quitFlag == false
                newPrnInt = prnData(i+itr,j);
                correctPrn = 2*prnData(i,j) - newPrnInt;
                prnData(i,j) = correctPrn - 1;
            end
            quitFlag == false;
        end
    end
end

recoveredPrn = uint8(prnData);


end

