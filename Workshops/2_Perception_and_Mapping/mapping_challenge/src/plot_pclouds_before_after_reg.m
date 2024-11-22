function plot_pclouds_before_after_reg(pc_prev,pc_curr,pc_curr_tformed,h_pc_compare)
% Plot point clouds before and after registration

figure(h_pc_compare)
subplot(1,2,1)
pcshowpair(pc_prev,pc_curr)
title('Before Registration','Color','w')

subplot(1,2,2)
pcshowpair(pc_prev,pc_curr_tformed)
title('After Registration','Color','w')

end