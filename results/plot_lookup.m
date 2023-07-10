load CB_param_lookup.mat

figure()
hold on;
yyaxis left
plot(CB_param_lookup.phi(1:11), CB_param_lookup.lambda(1:11), '-b', 'LineWidth', 1.2);
xlabel("phi/°（转接角）");
ylabel("最优设计参数lambda");

yyaxis right
plot(CB_param_lookup.phi(1:11), CB_param_lookup.kappa_overshoot(1:11)*100, '-r', 'LineWidth', 1.2);
ylabel("最大超出曲率百分比/%");