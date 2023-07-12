function [arc_len, param] = arclength_reparam(derivativeFcn, interval, num_samples)
%ARCLENGTH_REPARAM reparameterize a curve by arc length
%   Inputs:
%       derivativeFcn
%       interval: [0, u_end]
%       num_samples

[arc_len, param] = simpson_interg(derivativeFcn, interval, num_samples-1);

end

