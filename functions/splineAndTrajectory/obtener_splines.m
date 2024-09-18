function S = obtener_splines(t, q)
    % Natural cubic spline computation
    n = numel(t);
    h = diff(t);
    diag_princ = 2 * (h(1:end-1) + h(2:end));
    diag_sup = h(2:end-1);
    diag_inf = h(2:end-1);
    M = diag(diag_princ) + diag(diag_sup, 1) + diag(diag_inf, -1);
    delta = diff(q) ./ h;
    b = 6 * diff(delta);
    g_interior = M \ b';
    g = [0; g_interior; 0];
    a = (g(2:end) - g(1:end-1)) ./ (6 * h');
    b_coef = g(1:end-1) / 2;
    c = delta' - h' .* (2 * g(1:end-1) + g(2:end)) / 6;
    d = q(1:end-1)';
    S = [a b_coef c d];
end