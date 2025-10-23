def angular_diff(θ1, θ2):
    no_circle = abs(θ1 - θ2)
    circle = abs((360 - max(θ1, θ2)) + min(θ1, θ2))

    if no_circle < circle:
        return no_circle
    else: return -circle

