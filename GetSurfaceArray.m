function [ac_s] = GetSurfaceArray(t,ac_surfaces)
% Inputs: time vector and aircraft control surfaces vector
% Output: Aircrsft control surfaces over time, assuming that the
% control surfaces stay constant over time

ammount = length(t);

ac_s = ones(ammount,1) * ac_surfaces';

end

