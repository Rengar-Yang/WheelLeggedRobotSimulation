function Torque = MPCcontroller(State,U,A,B,U_weight,TargetSpeed,Phi,Lambda,InputNum,N,Q,S_Matrix,X,StateNum)
% ,Phi,Lambda,U_weight,InputNum,H,N,Q
        % N = 3;% Prediction length
        % StateNum = 3; % The number of the state
        % InputNum = 1; % The number of the input
        % OutputNum = 3; % The number of the output
        % U_weight = 1; % The weight of the U
        % Y_weight = 10*diag([1000 1000 1]); % The weight of the Y

        R = zeros(3*N,1); % Reference martrix [3N*1]
        for i = 1:length(R)
            if mod(i, 3) == 0
                R(i) = TargetSpeed;
            else
                R(i) = 0;
            end
        end
        % % =============Calculate the reference matrix during the process============
        % if k+N < size(Reference,1)
        %     for i = 0:N-1
        %          NowReference = [xr(k+i);yr(k+i);thetar(k+i)];
        %          R((3*i)+1:(3*i+3),1) = NowReference;
        %     end
        % % ===============Calculate the reference matrix near the end=============
        % else 
        %     for i = 0:N-1
        %         if k+i<size(my_alg('path'),1)
        %             NowReference = [xr(k+i);yr(k+i);thetar(k+i)];
        %         else
        %             NowReference = [xr(size(my_alg('path'),1));yr(size(my_alg('path'),1));thetar(size(my_alg('path'),1))];
        %         end
        %          R((3*i)+1:(3*i+3),1) = NowReference;
        %     end
        % end

        % % =======================Initial parameters for H and f matrix============================
        % Lambda = zeros(N*StateNum,StateNum);Lambda(1:StateNum,:)=eye(3);
        % Phi = zeros(N*StateNum, N*InputNum);
        % temp =eye(StateNum);
        % % =======================Calculate Phi, Lambda, C=======================
        % for i= 2 : N
        %     rows =StateNum*i-StateNum+1 :StateNum*i;
        %     Phi(rows,:)=[temp*B,Phi(rows-StateNum,1:end-InputNum)];
        %     temp = temp * A;
        %     Lambda(rows,:)=temp *A;
        % end
        % 
        % % =======================Calculate Q and S matrix=======================
        % q = Y_weight*eye(OutputNum); % Weights on output deviation from setpoint
        % Q = sparse(kron(eye(N),q));
        % s = U_weight*eye(InputNum); % Weights on input deviation from setpoint
        % S_Matrix = kron(diag([2*ones(1,N-1),1]),s);
        % S_Matrix = S_Matrix + kron(diag([-ones(1,N-1)],-1),s);
        % S_Matrix = S_Matrix + kron(diag([-ones(1,N-1)],1),s);

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
        % InnerP = 100;
        % InnerD  = 4;
        % TorqueInner = InnerP * State(1) + InnerD * State(2);

        PositionP = 1;
        PositionD = 0;
        TorquePosition = PositionP*X + PositionD*State(3);

        Torque = TorqueOuter + TorquePosition;

        % Torque = Torque(1);
end