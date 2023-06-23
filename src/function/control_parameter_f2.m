function f2 = control_parameter_f2(d_0, a_0, b_0, b_f)
fprintf("\n In f2 function \n");
fprintf("D0: %d \n", d_0);
fprintf("B0: %d \n", b_0);
fprintf("Case 1: %d \n", d_0 > b_0);
fprintf("Case 2: %d \n", d_0 < b_f);

if d_0 > b_0 || d_0 < b_f
    f2 = 0;
else
fprintf("\n In f2 function activated \n");

    f2 = a_0 * (d_0 / (b_f - b_0) + b_0 / (b_0 - b_f));
end
end
