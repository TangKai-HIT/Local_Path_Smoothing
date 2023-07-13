function [fitresult, gof] = createFit(phi_data, lambda_data)
%CREATEFIT(PHI_DATA,LAMBDA_DATA)
%  创建一个拟合。
%
%  要进行 '无标题拟合 1' 拟合的数据:
%      X 输入: phi_data
%      Y 输出: lambda_data
%  输出:
%      fitresult: 表示拟合的拟合对象。
%      gof: 带有拟合优度信息的结构体。
%
%  另请参阅 FIT, CFIT, SFIT.

%  由 MATLAB 于 13-Jul-2023 12:49:34 自动生成


%% 拟合: '无标题拟合 1'。
[xData, yData] = prepareCurveData( phi_data, lambda_data );

% 设置 fittype 和选项。
ft = fittype( 'a*exp(b*x)+c', 'independent', 'x', 'dependent', 'y' );
opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts.Display = 'Off';
opts.StartPoint = [0.381558457093008 0.765516788149002 0.795199901137063];

% 对数据进行模型拟合。
[fitresult, gof] = fit( xData, yData, ft, opts );

% 绘制数据拟合图。
figure( 'Name', '无标题拟合 1' );
h = plot( fitresult, xData, yData );
legend( h, 'lambda_data vs. phi_data', '无标题拟合 1', 'Location', 'NorthEast', 'Interpreter', 'none' );
% 为坐标区加标签
xlabel( 'phi_data', 'Interpreter', 'none' );
ylabel( 'lambda_data', 'Interpreter', 'none' );
grid on


