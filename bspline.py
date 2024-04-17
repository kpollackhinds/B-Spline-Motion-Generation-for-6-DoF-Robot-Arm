def gen_knot_vector(degree, n):
    """Returns a uniform clamped knot vector"""
    knot_vector_length = degree + n + 1 + 1  # m+1
    knot_vector = [0] * knot_vector_length

    x = 1 / (knot_vector_length + 1 - 2 * (degree + 1))
    for i in range(knot_vector_length):
        if i < degree + 1:
            continue
        elif i > knot_vector_length - 1 - degree - 1:
            knot_vector[i] = 1
        else:
            knot_vector[i] = knot_vector[i - 1] + x

    return knot_vector

def find_span(degree, n, u, knot_vector):
    p = degree
    m = len(knot_vector) - 1

    if u == knot_vector[n + 1]:
        return n

    low = p
    high = n + 1
    mid = (low + high) // 2

    while u < knot_vector[mid] or u >= knot_vector[mid + 1]:
        if u < knot_vector[mid]:
            high = mid
        else:
            low = mid
        mid = (low + high) // 2

    return mid

def basis_funcs(index, u, degree, knot_vector):
    N = [0.0] * (degree + 1)
    left = [0.0] * (degree + 1)
    right = [0.0] * (degree + 1)

    p = degree
    i = index

    N[0] = 1.0
    for j in range(1, p + 1):
        left[j] = u - knot_vector[i + 1 - j]
        right[j] = knot_vector[i + j] - u
        saved = 0.0

        for r in range(j):
            temp = N[r] / (right[r + 1] + left[j - r])
            N[r] = saved + right[r + 1] * temp
            saved = left[j - r] * temp
        N[j] = saved

    return N

def valid_knot_vector(knot_vector, degree, control_points, normalize=False):
    # Check if knot vector has enough numbers (m = p + n + 1)
    if len(knot_vector) != degree + len(control_points) + 1 or \
            min(knot_vector) < 0 or \
            knot_vector[-1] != knot_vector[-1] or \
            knot_vector[-1] == float("inf"):
        return False

    # Check if knot vector values are increasing
    for i in range(len(knot_vector) - 1):
        if knot_vector[i] > knot_vector[i + 1]:
            return False

    if normalize:
        max_value = max(knot_vector)
        if max_value > 1:
            knot_vector = [val / max_value for val in knot_vector]
        return knot_vector

    return True

def b_spline_curve(knot_vector, degree, control_positions, resolution=100):
    bspline_points = []
    u_min = knot_vector[degree]
    u_max = knot_vector[len(knot_vector) -degree - 1]

    for sample_num in range(resolution + 1):
        u = u_min + (sample_num * (u_max - u_min)) / resolution
        index = find_span(degree, len(control_positions) - 1, u, knot_vector)
        basis_functions = basis_funcs(index, u, degree, knot_vector)

        for j in range(degree + 1):
            if index - degree + j < 0 or index - degree + j >= len(control_positions):
                continue
            Nip = basis_functions[j]
            P_i = control_positions[index - degree + j]
            if j == 0:
                C_u = Nip * P_i
            else:
                C_u += Nip * P_i


        bspline_points.append(C_u)

    return bspline_points