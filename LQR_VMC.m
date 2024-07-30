function K = LQR_VMC(L0)
%LQR_VMC
%    K = LQR_VMC(L0)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    2024-07-30 20:08:02

t2 = L0.^2;
t3 = L0.^3;
mt1 = [L0.*(-1.575755058297929e+1)-t2.*6.210478712618976e+1+t3.*2.22949116315042e+2+2.927397943242606e-1];
mt2 = [L0.*4.971867387666016e-1-t2.*2.522596721610726+t3.*4.366970603500726-1.564400745187216e-2];
mt3 = [L0.*(-6.805000679210331e-1)-t2.*1.759843812287881e+1+t3.*3.551487371817518e+1+1.596805864955427e-2];
mt4 = [L0.*3.402007465273937e-2-t2.*8.977259910858454e-2+t3.*3.040511550219081e-2-7.321874485675998e-4];
mt5 = [L0.*(-2.935477484722629e+1)+t2.*1.062456862888647e+2-t3.*1.366503310491531e+2+5.268078446425215e-1];
mt6 = [L0.*4.867729844625689e-1-t2.*4.482066658922028+t3.*1.13533618620742e+1+1.590389629027031e-2];
mt7 = [L0.*(-1.829913562717524e+1)+t2.*5.182212503926323e+1-t3.*4.992117580257448e+1+3.099259036490254e-1];
mt8 = [L0.*3.875946940206277e-1-t2.*3.110430712079792+t3.*7.521916557857281+1.982015791297235e-3];
mt9 = [L0.*(-5.96467389792853e+1)+t2.*3.235012782841464e+2-t3.*6.813263171237346e+2+5.271882316963119];
mt10 = [L0.*2.483449480333306-t2.*1.671465589613474e+1+t3.*3.968745796683989e+1+6.574626919250577];
mt11 = [L0.*(-8.858192300360789e-1)+t2.*4.601616496369133-t3.*9.804666906730967+9.889985690318451e-2];
mt12 = [L0.*1.369298680551009e-2-t2.*7.851955468434604e-2+t3.*1.700005245169354e-1+1.275522044769929e-1];
K = reshape([mt1,mt2,mt3,mt4,mt5,mt6,mt7,mt8,mt9,mt10,mt11,mt12],2,6);
end
