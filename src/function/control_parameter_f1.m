function f1 = control_parameter_f1(d_0, a_m, b_m)
if d_0 > b_m
    f1 = a_m;
else
    f1 = a_m * (d_0 / b_m);
end
end
