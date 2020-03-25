using Plots;
using ControlSystems;

function simulatePlant()
    # Define helper laplace operator
    s = tf("s");

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

    rlocusplot(Gp,title = "Root Locus Gp(s)")
    savefig("~/Desktop/GpRLocusPlot.png")
    nyquistplot(Gp, title = "Nyquist Diagram Gp(s)")
    savefig("~/Desktop/GpNyquistPlot.png")

    rlocusplot(Gma,title = "Root Locus Gma(s)")
    savefig("~/Desktop/GmaRLocusPlot.png")
    nyquistplot(Gma, title = "Nyquist Diagram Gma(s)")
    savefig("~/Desktop/GmaNyquistPlot.png")

    setPlotScale("dB")
    w = range(1.0; stop=1e4,step=1e-1);
    bodeplot(Gma,w, title = "Bode Diagram Gma(s)")
    savefig("~/Desktop/GmaBodePlot.png")
    marginplot(Gma,w)
    savefig("~/Desktop/GmaMarginPlot.png")


end
