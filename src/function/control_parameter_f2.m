function f2 = control_parameter_f2(d_0, a_0, b_0, b_f)
if d_0 > b_0 || d_0 < b_f
    f2 = 0;
else
    f2 = a_0 * (d_0 / (b_f - b_0) + b_0 / (b_0 - b_f));
end
end
