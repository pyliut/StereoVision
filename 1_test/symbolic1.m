% Practice functions in the symbolic toolbox
%% defining symbols
a = sym('x');                   % no need for 'x', can also you just x
b1 = sym(1/4);
b2 = sym(2.7)
c = sym(4*pi); 
d1 = a + b1 + c                 % using symbolic allows for more accuracy as results are not stored as doubles
d2 = b1 + b2                    % allows addition of fractions to be returned in fractional form

%% solve
syms x y z                      % allows definition of more than one variable at a time
eqn1 = x + 3*z == 1;            % remember '==' sign, not '='
eqn2 = x^2 + x*y + z == 3;
eqn3 = subs(eqn1,'x',10)        % subs() substitutes in a value for the variables
eqn4 = simplify(eqn3)           % simplify() simplifies, collect() gathers terms, expand() expands, factor() factorises

[solx,soly] = solve(eqn1,eqn2,x,y)     % the 'x,y' is optional but it doesn't hurt to be specific
ezplot(eqn1)

%% matrices
syms x y A B C
A = [x y 0;
    0 x 2*y;
    0 1 x];
B = [1 2 3; 
    4 5 x; 
    y 6 7];
C = A*B

%% polynomials
coef = [1 2 3 4 5];
polynomial = poly2sym(coef)                 % poly2sym() takes a matrix of coefficients (from most to least significant) and turns it into a polynomial
coef2 = sym2poly(polynomial)                % sym2poly() takes symbolic equation and returns a 1D matrix of coefficients
formatted = pretty(polynomial)              % pretty() formats the polynomial to be more like written

%% plotting
syms x y z
y = x^2 + 2*x + 3;
ezplot(y)

