from sympy import symbols, diff, solve

# Define the variable
x = symbols('x')

# Define the polynomial
poly = 119.9 * x**5 + 1.0928 * x**4 - 29.224 * x**3 - 0.0388 * x**2 - 0.4588 * x

# Compute the first and second derivatives
first_derivative = diff(poly, x)
second_derivative = diff(first_derivative, x)

# Solve for roots of the second derivative
roots_second_derivative = solve(second_derivative, x)

# Print results
print("First derivative:", first_derivative)
print("Second derivative:", second_derivative)
print("Roots of the second derivative:", roots_second_derivative)