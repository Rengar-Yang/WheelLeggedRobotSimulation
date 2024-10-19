function Torque = MPCcontroller(State,U,A,B,U_weight,TargetSpeed,Phi,Lambda,InputNum,N,Q,S_Matrix,X,StateNum)
% ,Phi,Lambda,U_weight,InputNum,H,N,Q

        R = zeros(3*N,1); % Reference martrix [3N*1]
        for i = 1:length(R)
            if mod(i, 3) == 0
                R(i) = TargetSpeed;
            else
                R(i) = 0;
            end
        end

        % =============================Nonlinear term=============================
        o = (eye(3)-A)*State - B*U;
        O=zeros(N*StateNum,1);
        O(1:StateNum,:)=o + B*U;
        temp =eye(StateNum);

        for i= 2 : N
            rows =StateNum*i-StateNum+1 :StateNum*i;
            temp = temp * A; 
            O(rows,:)=O(rows-StateNum,:)+temp *(B*U + o);
        end
        NextState = State;

        % ================================Observer==============================
        % NextState = State + (A*State + B*U)*0.001;

        % =======================Calculate H and f matrix=======================
        E=[U_weight*U; zeros((N-1)*InputNum, 1)];    
        H = Phi'*Q*Phi + S_Matrix;
        H = (H+H')/2;
        f = Phi'*Q*(Lambda*NextState + O - R) - E;
        options = optimoptions('quadprog', 'Algorithm', 'active-set', 'Display', 'off', 'MaxIter', 1000);
        Torque = zeros(N, 1); 
        Torque = quadprog(H, f, [], [], [], [], [], [], zeros(size(f)),options);

        TorqueOuter = Torque(1);

        PositionP = 1;
        PositionD = 0;
        TorquePosition = PositionP*X + PositionD*State(3);

        Torque = TorqueOuter + TorquePosition;

end