import numpy as np
from helper_functions import get_translation
from dual_quaternions import DualQuaternion
def gen_knot_vector(degree, n,style="Clamped"):
    """Returns a uniform clamped knot vector"""
    knot_vector_length = degree + n + 1 + 1  # m+1
    knot_vector = [0] * knot_vector_length
    if style=="Clamped":
        x = 1 / (knot_vector_length + 1 - 2 * (degree + 1))
        for i in range(knot_vector_length):
            if i < degree + 1:
                continue
            elif i > knot_vector_length - 1 - degree - 1:
                knot_vector[i] = 1
            else:
                knot_vector[i] = knot_vector[i - 1] + x
    elif style=="Closed":
        for i in range(knot_vector_length):
            knot_vector[i]=i/knot_vector_length
    return knot_vector

def find_span(degree, n, u, knot_vector):
    p = degree
    m = len(knot_vector) - 1

    if abs(u-knot_vector[n + 1])<.001:
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

def parameterize(dq_list,style="Uniform",dist="Quaternion"):
    parameters=[]
    if style=="Uniform":
        for i in range(len(dq_list)):
            parameters.append(i/(len(dq_list)-1))
    elif style=="Chord":    
        L=0
        for i in range(len(dq_list)-1):
            if dist=="Quaternion":
                L+=(dq_list[i+1].q_r-dq_list[i].q_r).norm
            if dist=="Cartesian":
                d1=get_translation(dq_list[i+1])
                d2=get_translation(dq_list[i])
                L+=((d1[0]-d2[0])**2+(d1[1]-d2[1])**2+(d1[2]-d2[2])**2)**.5
        for i in range(len(dq_list)):
            k=0
            for j in range(1,i+1):
                if dist=="Quaternion":
                    k+=(dq_list[j].q_r-dq_list[j-1].q_r).norm
                if dist=="Cartesian":
                    d1=get_translation(dq_list[j])
                    d2=get_translation(dq_list[j-1])
                    k+=((d1[0]-d2[0])**2+(d1[1]-d2[1])**2+(d1[2]-d2[2])**2)**.5
            parameters.append(k/L)
    elif style=="Centripetal":
        L=0
        for i in range(len(dq_list)-1):
            if dist=="Quaternion":
                L+=((dq_list[i+1].q_r-dq_list[i].q_r).norm)**.5
            if dist=="Cartesian":
                d1=get_translation(dq_list[i+1])
                d2=get_translation(dq_list[i])
                L+=((d1[0]-d2[0])**2+(d1[1]-d2[1])**2+(d1[2]-d2[2])**2)**.25
        for i in range(len(dq_list)):
            k=0
            for j in range(1,i+1):
                if dist=="Quaternion":
                    k+=((dq_list[j].q_r-dq_list[j-1].q_r).norm)**.5
                if dist=="Cartesian":
                    d1=get_translation(dq_list[j])
                    d2=get_translation(dq_list[j-1])
                    k+=((d1[0]-d2[0])**2+(d1[1]-d2[1])**2+(d1[2]-d2[2])**2)**.25
            parameters.append(k/L)
    print("p",parameters)
    return parameters

def get_control_points(dq_list,parameter,degree,h=None):
    if h==None:
        h=len(dq_list)-1
    if h==len(dq_list)-1 and degree==1:
        return dq_list
    points=[dq_list[0]]
    knot_vector=interpolation_knot_vector(len(dq_list)-1,h,degree,parameter)
    print(knot_vector)
    Q=[]
    for i in range(len(dq_list)):
        print(better_basis_function(0,degree,parameter[i],knot_vector))
        print(better_basis_function(h,degree,parameter[i],knot_vector))
        Q.append(dq_list[i]+(-1)*better_basis_function(0,degree,parameter[i],knot_vector)*dq_list[0]+(-1)*better_basis_function(h,degree,parameter[i],knot_vector)*dq_list[-1])
    Q1=np.zeros([len(dq_list)-2,8])
    for i in range(1,h):
        Q1[i-1,:]=Q[i].dq_array()
    #Q_arr=np.zeros([h-1,8])
    #for i in range(1,h):
    #   value =DualQuaternion.from_dq_array([0,0,0,0,0,0,0,0])
    #    for j in range(1,len(dq_list)-1):
    #        value+=better_basis_function(i,degree,parameter[j],knot_vector)*Q[j]
    #    Q_arr[i-1,:]=value.dq_array()
    N_arr=np.zeros([len(dq_list)-2,h-1])
    for i in range(1,len(dq_list)-1):
        for j in range(1,h):
            N_arr[i-1,j-1]=better_basis_function(j,degree,parameter[i],knot_vector)
    print(N_arr)
    Q_arr=np.matmul(N_arr.transpose(),Q1)
    P=np.matmul(np.linalg.inv(np.matmul(N_arr.transpose(),N_arr)),Q_arr)
    #print("P",P)
    #P=np.matmul(np.linalg.pinv(N_arr),Q1)
    #print(P)
    for i in range(h-1):
        points.append(DualQuaternion.from_dq_array(P[i,:]))
    points.append(dq_list[-1])
    return points
    
def interpolation_knot_vector(n,h,degree,parameter):
    if h==None:
        h=n
    d=(n+1)/(h-degree+1)
    knot_vector_length=h+degree+1+1
    vector = [0] * knot_vector_length
    for i in range(knot_vector_length):
            if i < degree + 1:
                continue
            elif i > knot_vector_length - 1 - degree - 1:
                vector[i] = 1
            else:
                if h==n:
                    value=0
                    for j in range(i,i+degree):
                        value+=parameter[j-degree]
                    vector[i]=(1/degree)*value
                else:
                    index=int((i-degree)*d)
                    alpha=(i-degree)*d-index
                    vector[i] = (1-alpha)*parameter[index-1]+alpha*parameter[index]
    return vector


def better_basis_function(index,degree,t,knot_vector):
    prevArray=[]
    for i in range(len(knot_vector)-1):
        if t>=knot_vector[i] and t<knot_vector[i+1]:
            prevArray.append(1)
        else:
            prevArray.append(0)
    for j in range(1,degree+1):
        current_array=[]
        for i in range(len(prevArray)-1):
            if knot_vector[i+j]==knot_vector[i]and knot_vector[i+j+1]==knot_vector[i+1]:
                current_array.append(0)
            elif knot_vector[i+j]==knot_vector[i]:
                current_array.append(prevArray[i+1]*(knot_vector[i+j+1]-t)/(knot_vector[i+j+1]-knot_vector[i+1]))
            elif knot_vector[i+j+1]==knot_vector[i+1]:
                current_array.append(prevArray[i]*(t-knot_vector[i])/ (knot_vector[i+j]-knot_vector[i]))
            else:
                current_array.append(prevArray[i]*(t-knot_vector[i])/ (knot_vector[i+j]-knot_vector[i]) + prevArray[i+1]* (knot_vector[i+j+1]-t)/(knot_vector[i+j+1]-knot_vector[i+1]))
        prevArray=current_array
    return prevArray[index]
