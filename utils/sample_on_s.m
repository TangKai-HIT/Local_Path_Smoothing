function [s_sample, u_sample] = sample_on_s(params, arc_lens, num_samples, plotFlag)
%SAMPLE_ON_S Uniform arc-length sampling on a parametric curve
%   此处显示详细说明

%resample on s
s_sample = linspace(arc_lens(1), arc_lens(end), num_samples);

u_sample = zeros(1, num_samples);
u_sample(1) = params(1);
u_sample(end) = params(end);

%s-u
index = 2;
i = 2;
while i<=length(arc_lens) && index<=num_samples
    if arc_lens(i) > s_sample(index)
        del_u = params(i) - params(i-1); 
        del_arc = arc_lens(i) - arc_lens(i-1);
        u_sample(index) = params(i-1) + del_u * (s_sample(index) - arc_lens(i-1))/del_arc;

        index = index + 1;
        continue;
    end
    
    i = i + 1;
end

%plot resampled s-u relation
if plotFlag
    figure()
    plot(s_sample, u_sample, '-b', 'LineWidth', 1);
    xlabel("s"); ylabel("u"); title("resampled")
end

end

