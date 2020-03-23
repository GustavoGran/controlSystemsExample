using ControlSystems;
using Plots

function simulatePlant()
    # Define helper laplace operator
    s = tf("s")

    # Define pendulum parameters and Transfer function
    g = 9.81
    l  = 0.5
    Gp = (-g/l) / (s^2 - g/l)

    # Define controller parameters
    ξ = 0.7
    ωn = 6.45
    zc = sqrt(g/l)
    pc = zc + 2*ξ*ωn
    s1 = -ξ*ωn +(ωn*sqrt(1-ξ^2))im
    kc = abs((s1 + pc)*(s1 - sqrt(g/l))/(g/l))
    Gc = -kc * (s + zc) / (s + pc)

    # Define Open Loop ft and Closed loop ft
    Gma = Gc * Gp
    Gmf = (Gc * Gp) / (1 + Gc * Gp)

    nyquistplot(Gma)
    savefig("~/Desktop/GmaPlot.png")
    nyquistplot(Gp)
    savefig("~/Desktop/GpPlot.png")
end
