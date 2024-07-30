%% Leg sturcture
global l1 l2 l3 l4 l0

l1=90;    
l2=120; 
l3=120; 
l4=90; 
l0=32;

%% Inverse dynanmic

LegInverseDynamic(16,56,90,120,120,90,32)

%% Jacobin Matrix
 
    syms l1 l2 l3 l4 l5 F Tp real;
        
    syms phi1 phi2 phi3 phi4;
    syms xc yc xb yb xd yd; % c,b,d三点的xy坐标
    
    %进行几何计算
    xb=l1*cos(phi1);
    yb=l1*sin(phi1);
    xd=l5+l4*cos(phi4);
    yd=l4*sin(phi4);
    
    A0=2*l2*(xd-xb);
    B0=2*l2*(yd-yb);
    C0=l2^2+(xd-xb)^2+(yd-yb)^2-l3^2;
    phi2=2*atan((B0+sqrt(A0^2+B0^2-C0^2))/(A0+C0));
    
    xc=xb+l2*cos(phi2);
    yc=yb+l2*sin(phi2);
    
    l0=sqrt((xc-l5/2)^2+yc^2);
    phi0=atan2(yc,(xc-l5/2));
    % 求得腿部姿态 [l0; phi0] = leg_pos(phi1, phi4)
    pos=[l0; phi0];
    matlabFunction(pos,'File','LegPosition');
    
    % 计算雅可比矩阵
    J11=diff(l0,phi1);
    J12=diff(l0,phi4);
    J21=diff(phi0,phi1);
    J22=diff(phi0,phi4);
    J=[J11 J12; J21 J22];
    
    T=J'*[F; Tp];
    
    matlabFunction(T, 'File', 'JointVMC', 'Vars', {F,Tp,phi1, phi4, l1, l2, l3, l4, l5});

%% LQR controller with VMC
    clc;
    clear all;
    
    L0s=0.06:0.01:0.15; % L0变化范围
    Ks=zeros(2,6,length(L0s)); % 存放不同L0对应的K
    
    for step=1:length(L0s)
        syms theta theta1 theta2; % theta1=dTheta, theta2=ddTheta
        syms x x1 x2;
        syms phi phi1 phi2;
        syms T Tp N P Nm Pm Nf t;
        
        % 机器人结构参数
        R=0.03; 
        L=L0s(step)/2; 
        Lm=L0s(step)/2; 
        l=0; 
        mw=0.15; 
        mp=0; 
        M=0.5; 
        Iw=0.5 * mw * R^2; 
        Ip=0; 
        h = 0.03;   % height of the body (m)
        w = 0.112;   % width of the body (m)
        Im=(1/12) * M * (h^2 + w^2);
        g=9.8;
        
        % 进行物理计算
        Nm=M*(x2+(L+Lm)*(theta2*cos(theta)-theta1^2*sin(theta))-l*(phi2*cos(phi)-phi1^2*sin(phi)));
        Pm=M*g+M*((L+Lm)*(-theta1^2*cos(theta)-theta2*sin(theta))-l*(phi1^2*cos(phi)+phi2*sin(phi)));
        N=Nm+mp*(x2+L*(theta2*cos(theta)-theta1^2*sin(theta)));
        P=Pm+mp*g+mp*L*(-theta1^2*cos(theta)-theta2*sin(theta));
        
        %二阶导数求解
        equ1=x2-(T-N*R)/(Iw/R+mw*R);% =0求解
        equ2=(P*L+Pm*Lm)*sin(theta)-(N*L+Nm*Lm)*cos(theta)-T+Tp-Ip*theta2;
        equ3=Tp+Nm*l*cos(phi)+Pm*l*sin(phi)-Im*phi2;
        [x2,theta2,phi2]=solve(equ1,equ2,equ3,x2,theta2,phi2);
        
        % 求得雅克比矩阵，然后得到状态空间方程
        Ja=jacobian([theta1;theta2;x1;x2;phi1;phi2],[theta theta1 x x1 phi phi1]);
        Jb=jacobian([theta1;theta2;x1;x2;phi1;phi2],[T Tp]);
        A=vpa(subs(Ja,[theta theta1 x x1  phi phi1],[0 0 0 0 0 0]));
        B=vpa(subs(Jb,[theta theta1 x x1  phi phi1],[0 0 0 0 0 0]));
        
        % 离散化
        [G,H]=c2d(eval(A),eval(B),0.005);
        
        % 定义权重矩阵Q, R
        Q=diag([1 1 500 100 5000 1]);
        R=diag([1 0.25]);
    
        % 求解反馈矩阵K
        Ks(:,:,step)=dlqr(G,H,Q,R);
        % tK_LQR_VMC = 0.1 * [0:29]';
        % Simulation_K_LQR_VMC = repmat(K_LQR_VMC,[1 1 length(tK_LQR_VMC)]);
        % dataK_LQR_VMC.time=tK_LQR_VMC;
        % dataK_LQR_VMC.signals.values = Simulation_K_LQR_VMC;
        % dataK_LQR_VMC.signals.dimensions=[size(K_LQR_VMC, 1) size(K_LQR_VMC, 2)];
        end
    
    % 对K的每个元素关于L0进行拟合
    K=sym('K',[2 6]);
    syms L0;
    for x=1:2
        for y=1:6
            p=polyfit(L0s,reshape(Ks(x,y,:),1,length(L0s)),3);
            K(x,y)=p(1)*L0^3+p(2)*L0^2+p(3)*L0+p(4);
        end
    end
    
    % 输出到m函数
    matlabFunction(K,'File','LQR_VMC');


%% LQR controller
    clc;
    clear all;
    
    % Example parameters
    mb = 0.5;  % mass of the float-based body (kg)
    mw = 0.15;  % mass of the wheel (kg)
    rw = 0.03;  % radius of the wheel (m)
    h = 0.03;   % height of the body (m)
    w = 0.112;   % width of the body (m)
    l = 0.106;   % length of the link (m)
    g = 9.81;  % gravitational acceleration (m/s²)
    
    % Calculate moments of inertia
    Iw = 0.5 * mw * rw^2;
    Ib = (1/12) * mb * (h^2 + w^2);
    
    % Compute matrices
    [A, B] = StateSpaceMatrix(mb, mw, Ib, Iw, rw, l, g);
    tA = 0.1 * [0:29]';
    Simulation_A = repmat(A,[1 1 length(tA)]);
    dataA.time=tA;
    dataA.signals.values = Simulation_A;
    dataA.signals.dimensions=[3 3];

    tB = 0.1 * [0:29]';
    Simulation_B = repmat(B,[1 1 length(tB)]);
    dataB.time=tB;
    dataB.signals.values = Simulation_B;
    dataB.signals.dimensions=[3 1];

    % Define weight matrices Q and R for LQR
    Q = diag([100, 1, 1]); % Adjust as needed
    R = 1000000; % Adjust as needed
    
    % Calculate LQR gain matrix K
    K_LQR= lqr(A, B, Q, R); 
    tK_LQR = 0.1 * [0:29]';
    Simulation_K_LQR = repmat(K_LQR,[1 1 length(tK_LQR)]);
    dataK_LQR.time=tK_LQR;
    dataK_LQR.signals.values = Simulation_K_LQR;
    dataK_LQR.signals.dimensions=[size(K_LQR, 1) size(K_LQR, 2)];

  %% MPC controller, state space get
        clc;
        clear all;
        % Example parameters
        mb = 0.5;  % mass of the float-based body (kg)
        mw = 0.15;  % mass of the wheel (kg)
        rw = 0.03;  % radius of the wheel (m)
        h = 0.03;   % height of the body (m)
        w = 0.112;   % width of the body (m)
        l = 0.56;   % length of the link (m)
        g = 9.81;  % gravitational acceleration (m/s²)
        
        % Calculate moments of inertia
        Iw = 0.5 * mw * rw^2;
        Ib = (1/12) * mb * (h^2 + w^2);
        
        % Compute matrices
        [A, B] = StateSpaceMatrix(mb, mw, Ib, Iw, rw, l, g);
        A = eye(3) + 0.001*A;
        B = 0.001*B;

        tA = 0.1 * [0:29]';
        Simulation_A = repmat(A,[1 1 length(tA)]);
        dataA.time=tA;
        dataA.signals.values = Simulation_A;
        dataA.signals.dimensions=[3 3];

        tB = 0.1 * [0:29]';
        Simulation_B = repmat(B,[1 1 length(tB)]);
        dataB.time=tB;
        dataB.signals.values = Simulation_B;
        dataB.signals.dimensions=[3 1];

        %% Compute the offline matrix
        % State = [0;0;0];
        % U = 0;

        N = 20;% Prediction length
        StateNum = 3; % The number of the state
        InputNum = 1; % The number of the input
        OutputNum = 3; % The number of the output
        U_weight = 1; % The weight of the U
        Y_weight = 1*diag([1000000 50 1]); % The weight of the Y


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
       
        R = zeros(3*N,1); % Reference martrix [3N*1]
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
        Lambda = zeros(N*StateNum,StateNum);Lambda(1:StateNum,:)=A;
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

        % % =======================Calculate H and f matrix=======================
        % E=[U_weight*U; zeros((N-1)*InputNum, 1)];    
        % H = Phi'*Q*Phi + S_Matrix;
        % H = (H+H')/2;
        % f = Phi'*Q*(Lambda*NextState - R) - E;
        % options = optimoptions('quadprog', 'Algorithm', 'active-set', 'Display', 'off');
        % Torque = quadprog(H, f, [], [], [], [], [], [], zeros(size(f)),options);
        % % Torque = Torque(1);
    
        %%
        MPCcontroller([1;0.1;0.2],0,A,B,U_weight,0,Phi,Lambda,InputNum,N,Q,S_Matrix,0,StateNum)
        
