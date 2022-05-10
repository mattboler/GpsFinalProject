function [shifts,values] = autoCorrelation(random_vec_1,random_vec_2)
%AUTOCORRELATION this function will perform an autocorrelation of
% a vector with another vector

if(length(random_vec_1) ~= length(random_vec_2))
    error('VECTOR INPUTS MUST BE OF EQUAL LENGTH!')
end
shiftedVec = [];
j = 1;
for i = 0:(length(random_vec_1)-1)
   if(i == 0)
       values(j) = sum(dot(random_vec_1,random_vec_2));
       shifts(j) = i;
       j= j+1;
   else
       shiftedVec = [random_vec_2(i+1:length(random_vec_2)),random_vec_2(1:(i))];
       values(j) = sum(dot(random_vec_1,shiftedVec));
       shifts(j) = i;
       j = j+1;
   end
end

end

