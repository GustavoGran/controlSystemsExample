using ControlSystems;

function delaySystemStep()
    sys = tf(1, [1,1])*delay(1)
    stepplot(sys,5); # Compilation time might be long 1st time
    # nyquistplot(sys)
end

function delaySystemNyquist()
    sys = tf(1, [1,1])*delay(1)
    # stepplot(sys,5) # Compilation time might be long 1st time
    nyquistplot(sys);
end

function motorSimulation()
    # Motor parameters
    J = 2.0
    b = 0.04
    K = 1.0
    R = 0.08
    L = 1e-4

    # Create the moel transfer function
    s = tf("s")
    P = K / (s*((J*s + b)*(L*s + R) + K^2))
    # This generates the system:
    # TransferFunction{ControlSystems.SisoRational{Float64}}
    #                 1.0
    # ------------------------------------
    # 0.0002*s^3 + 0.160004*s^2 + 1.0032*s
    #
    # Continuous-time transfer function model

    # Create an array of closed loop systems for different values of Kp
    CLs = TransferFunction[kp*P/(1 + kp*P) for kp = [1, 5, 15]];

    # Plot the step response of the controllers
    # Any Keyword arguments supported in Plots.jl can be supplied
    stepplot(CLs, label=["Kp = 1" "Kp = 5" "Kp = 15"])

end
