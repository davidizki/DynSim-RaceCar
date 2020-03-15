function c = controlMap_eval(c,T,mode)

c.delta_t = NaN(size(T));
c.delta_b = NaN(size(T));
c.theta_s = NaN(size(T));
c_ii = struct('delta_t',[],'delta_b',[],'theta_s',[]);

for ii = 1:numel(T)
    c_ii = controlMap(c_ii,T(ii),mode);
    c.delta_t(ii) = c_ii.delta_t;
    c.delta_b(ii) = c_ii.delta_b;
    c.theta_s(ii) = c_ii.theta_s;
end

end
