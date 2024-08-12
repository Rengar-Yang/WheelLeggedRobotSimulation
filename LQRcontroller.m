function Torque = LQRcontroller(State,K_LQR,TargetSpeed,X)
    
    % Desired state
    X_des = [0;0;TargetSpeed];

    % Display the LQR gain matrix
    Torque = K_LQR * (X_des-State) + 0.003*X + 0.001*State(3);
    % Torque = K_LQR * (X_des-State)
end