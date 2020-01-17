function kill_figures(fig_arr)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


for i=1:length(fig_arr)
   try
       clf(fig_arr(i));
   catch
       fprintf('Figure %d not opened\n',fig_arr(i));
   end
end
end

