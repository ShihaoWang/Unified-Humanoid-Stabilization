function bspline_basis_test()
knot = [ 0, 0.1, 0.2, 0.3, 0.5, 0.5, 0.6, 1, 1, 1 ];
p = 3;
x = 1;
y = bspline_basis(6,p,knot,x);

end

