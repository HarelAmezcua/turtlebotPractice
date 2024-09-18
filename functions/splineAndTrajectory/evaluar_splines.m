function [qk, qpk] = evaluar_splines(S, t_array, time)
    n = numel(t_array);
    if time > t_array(end)
        k = n - 1;
    else
        k = find(t_array(1:end-1) <= time & time <= t_array(2:end), 1);
    end
    if isempty(k)
        k = 1;
    end
    h = time - t_array(k);
    qk = S(k, :) * [h^3; h^2; h; 1];
    qpk = S(k, :) * [3*h^2; 2*h; 1; 0];
end