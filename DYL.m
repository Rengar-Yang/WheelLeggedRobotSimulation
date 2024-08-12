        clc;
        clear all;
        A = [0 0 0 0 -192.34; 1 0 0 0 -178.04; 0 1 0 0 -77.14; 0 0 1 0 -27.24; 0 0 0 1 -5.12];
        B = [194.49;-60.46;9.72;-1.29;-0.012];
        C = [0 0 0 0 1];


        tA = 0.1 * [0:29]';
        Simulation_A = repmat(A,[1 1 length(tA)]);
        dataA.time=tA;
        dataA.signals.values = Simulation_A;
        dataA.signals.dimensions=[5 5];

        tB = 0.1 * [0:29]';
        Simulation_B = repmat(B,[1 1 length(tB)]);
        dataB.time=tB;
        dataB.signals.values = Simulation_B;
        dataB.signals.dimensions=[5 1];

        %% Compute the offline matrix
        State = [0;0;0;0;0];
        U = 0;

        N = 20;% Prediction length
        StateNum = 5; % The number of the state
        InputNum = 1; % The number of the input
        OutputNum = 1; % The number of the output
        U_weight = 1; % The weight of the U
        Y_weight = 1*diag([1 1 1 1 1]); % The weight of the Y


        tN = 0.1 * [0:29]';
        Simulation_N = repmat(N,[1 1 length(tN)]);
        dataN.time=tN;
        dataN.signals.values = Simulation_N;
        dataN.signals.dimensions=[1 1];

        tU_weight= 0.1 * [0:29]';
        Simulation_U_weight = repmat(U_weight,[1 1 length(tU_weight)]);
        dataU_weight.time=tU_weight;
        dataU_weight.signals.values = Simulation_U_weight;
        dataU_weight.signals.dimensions=[1 1];

        tInputNum = 0.1 * [0:29]';
        Simulation_InputNum = repmat(InputNum,[1 1 length(tInputNum)]);
        dataInputNum.time=tInputNum;
        dataInputNum.signals.values = Simulation_InputNum;
        dataInputNum.signals.dimensions=[1 1];
        
        tStateNum = 0.1 * [0:29]';
        Simulation_StateNum = repmat(StateNum,[1 1 length(tStateNum)]);
        dataStateNum.time=tStateNum;
        dataStateNum.signals.values = Simulation_StateNum;
        dataStateNum.signals.dimensions=[1 1];
       
        R = zeros(5*N,1); % Reference martrix [3N*1]
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

        % =======================Initial parameters for H and f matrix============================
        Lambda = zeros(N*StateNum,StateNum);
        Lambda(1:StateNum,:)=A;
        Phi = zeros(N*StateNum, N*InputNum);
        temp =eye(StateNum);

        % =======================Calculate Phi, Lambda, C=======================
        for i= 2 : N
            rows =StateNum*i-StateNum+1 :StateNum*i;
            Phi(rows,:)=[temp*B,Phi(rows-StateNum,1:end-InputNum)];
            temp = temp * A;
            Lambda(rows,:)=temp *A;
        end

        tPhi = 0.1 * [0:29]';
        Simulation_Phi = repmat(Phi,[1 1 length(tPhi)]);
        dataPhi.time=tPhi;
        dataPhi.signals.values = Simulation_Phi;
        dataPhi.signals.dimensions=[size(Phi, 1) size(Phi, 2)];

        tLambda = 0.1 * [0:29]';
        Simulation_Lambda = repmat(Lambda,[1 1 length(tLambda)]);
        dataLambda.time=tLambda;
        dataLambda.signals.values = Simulation_Lambda;
        dataLambda.signals.dimensions=[size(Lambda, 1) size(Lambda, 2)];

        % =======================Calculate Q and S matrix=======================
        q = Y_weight*eye(OutputNum); % Weights on output deviation from setpoint
        Q = sparse(kron(eye(N),q));
        Q = full(Q);
        s = U_weight*eye(InputNum); % Weights on input deviation from setpoint
        S_Matrix = kron(diag([2*ones(1,N-1),1]),s);
        S_Matrix = S_Matrix + kron(diag([-ones(1,N-1)],-1),s);
        S_Matrix = S_Matrix + kron(diag([-ones(1,N-1)],1),s);

        tQ = 0.1 * [0:29]';
        Simulation_Q = repmat(Q,[1 1 length(tQ)]);
        dataQ.time=tQ;
        dataQ.signals.values = Simulation_Q;
        dataQ.signals.dimensions=[size(Q, 1); size(Q, 2);];

        tS_Matrix = 0.1 * [0:29]';
        Simulation_S_Matrix = repmat(S_Matrix,[1 1 length(tS_Matrix)]);
        dataS_Matrix.time=tS_Matrix;
        dataS_Matrix.signals.values = Simulation_S_Matrix;
        dataS_Matrix.signals.dimensions=[size(S_Matrix, 1) size(S_Matrix, 2)];

       % % ================================Observer==============================
       %  NextState = State + (A*State + B*U)*0.001;

        % =======================Calculate H and f matrix=======================
        E=[U_weight*U; zeros((N-1)*InputNum, 1)];    
        H = Phi'*Q*Phi + S_Matrix;
        H = (H+H')/2;
        f = Phi'*Q*(Lambda*State - R) - E;
        options = optimoptions('quadprog', 'Algorithm', 'active-set', 'Display', 'off');
        Torque = quadprog(H, f, [], [], [], [], [], [], zeros(size(f)),options);
        % Torque = Torque(1);