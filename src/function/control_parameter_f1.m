function f1 = control_parameter_f1(d_m, a_m, b_m)
if d_m > b_m
    f1 = a_m;
else
    f1 = a_m * (d_m / b_m);
end
end
