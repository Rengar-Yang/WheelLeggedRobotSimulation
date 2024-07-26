function State_hat = Observer(State,A,B,U)
    State_hat = State + (A*State + B*U)*0.001;
end