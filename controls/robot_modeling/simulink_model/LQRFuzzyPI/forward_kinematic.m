function forward_kinematic(block)
%MSFUNTMPL A Template for a MATLAB S-Function
%   The MATLAB S-function is written as a MATLAB function with the
%   same name as the S-function. Replace 'msfuntmpl' with the name
%   of your S-function.  
%
%   It should be noted that the MATLAB S-function is very similar
%   to Level-2 C-Mex S-functions. You should be able to get more 
%   information for each of the block methods by referring to the
%   documentation for C-Mex S-functions.
%  
%   Copyright 2003-2010 The MathWorks, Inc.
  
%
% The setup method is used to setup the basic attributes of the
% S-function such as ports, parameters, etc. Do not add any other
% calls to the main body of the function.  
%   
setup(block);
  
%endfunction

% Function: setup ===================================================
% Abstract:
%   Set up the S-function block's basic characteristics such as:
%   - Input ports
%   - Output ports
%   - Dialog parameters
%   - Options
% 
%   Required         : Yes
%   C-Mex counterpart: mdlInitializeSizes
%
function setup(block)

  % Register the number of ports.
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 1;
  
  % Set up the port properties to be inherited or dynamic.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  % Override the input port properties.
  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';
  
  % Override the output port properties.
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';

  % Register the parameters.
  %block.NumDialogPrms     = 3;
  %block.DialogPrmsTunable = {'Tunable','Nontunable','SimOnlyTunable'};
  
  % Set up the continuous states.
  block.NumContStates = 4;

  % Register the sample times.
  %  [0 offset]            : Continuous sample time
  %  [positive_num offset] : Discrete sample time
  %
  %  [-1, 0]               : Inherited sample time
  %  [-2, 0]               : Variable sample time
  block.SampleTimes = [0 0];
  
  % -----------------------------------------------------------------
  % Options
  % -----------------------------------------------------------------
  % Specify if Accelerator should use TLC or call back to the 
  % MATLAB file
  block.SetAccelRunOnTLC(false);
  
  % Specify the block simStateCompliance. The allowed values are:
  %    'UnknownSimState', < The default setting; warn and assume DefaultSimState
  %    'DefaultSimState', < Same SimState as a built-in block
  %    'HasNoSimState',   < No SimState
  %    'CustomSimState',  < Has GetSimState and SetSimState methods
  %    'DisallowSimState' < Errors out when saving or restoring the SimState
  block.SimStateCompliance = 'DefaultSimState';
  
  % -----------------------------------------------------------------
  % The MATLAB S-function uses an internal registry for all
  % block methods. You should register all relevant methods
  % (optional and required) as illustrated below. You may choose
  % any suitable name for the methods and implement these methods
  % as local functions within the same file.
  % -----------------------------------------------------------------
   
  % -----------------------------------------------------------------
  % Register the methods called during update diagram/compilation.
  % -----------------------------------------------------------------
  
  % 
  % CheckParameters:
  %   Functionality    : Called in order to allow validation of the
  %                      block dialog parameters. You are 
  %                      responsible for calling this method
  %                      explicitly at the start of the setup method.
  %   C-Mex counterpart: mdlCheckParameters
  %
  block.RegBlockMethod('CheckParameters', @CheckPrms);

  %
  % SetInputPortSamplingMode:
  %   Functionality    : Check and set input and output port 
  %                      attributes and specify whether the port is operating 
  %                      in sample-based or frame-based mode
  %   C-Mex counterpart: mdlSetInputPortFrameData.
  %   (The DSP System Toolbox is required to set a port as frame-based)
  %
  block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
  
  %
  % SetInputPortDimensions:
  %   Functionality    : Check and set the input and optionally the output
  %                      port dimensions.
  %   C-Mex counterpart: mdlSetInputPortDimensionInfo
  %
  block.RegBlockMethod('SetInputPortDimensions', @SetInpPortDims);

  %
  % SetOutputPortDimensions:
  %   Functionality    : Check and set the output and optionally the input
  %                      port dimensions.
  %   C-Mex counterpart: mdlSetOutputPortDimensionInfo
  %
  block.RegBlockMethod('SetOutputPortDimensions', @SetOutPortDims);
  
  %
  % SetInputPortDatatype:
  %   Functionality    : Check and set the input and optionally the output
  %                      port datatypes.
  %   C-Mex counterpart: mdlSetInputPortDataType
  %
  block.RegBlockMethod('SetInputPortDataType', @SetInpPortDataType);
  
  %
  % SetOutputPortDatatype:
  %   Functionality    : Check and set the output and optionally the input
  %                      port datatypes.
  %   C-Mex counterpart: mdlSetOutputPortDataType
  %
  block.RegBlockMethod('SetOutputPortDataType', @SetOutPortDataType);
  
  %
  % SetInputPortComplexSignal:
  %   Functionality    : Check and set the input and optionally the output
  %                      port complexity attributes.
  %   C-Mex counterpart: mdlSetInputPortComplexSignal
  %
  block.RegBlockMethod('SetInputPortComplexSignal', @SetInpPortComplexSig);
  
  %
  % SetOutputPortComplexSignal:
  %   Functionality    : Check and set the output and optionally the input
  %                      port complexity attributes.
  %   C-Mex counterpart: mdlSetOutputPortComplexSignal
  %
  block.RegBlockMethod('SetOutputPortComplexSignal', @SetOutPortComplexSig);
  
  %
  % PostPropagationSetup:
  %   Functionality    : Set up the work areas and the state variables. You can
  %                      also register run-time methods here.
  %   C-Mex counterpart: mdlSetWorkWidths
  %
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

  % -----------------------------------------------------------------
  % Register methods called at run-time
  % -----------------------------------------------------------------
  
  % 
  % ProcessParameters:
  %   Functionality    : Call to allow an update of run-time parameters.
  %   C-Mex counterpart: mdlProcessParameters
  %  
  block.RegBlockMethod('ProcessParameters', @ProcessPrms);

  % 
  % InitializeConditions:
  %   Functionality    : Call to initialize the state and the work
  %                      area values.
  %   C-Mex counterpart: mdlInitializeConditions
  % 
  block.RegBlockMethod('InitializeConditions', @InitializeConditions);
  
  % 
  % Start:
  %   Functionality    : Call to initialize the state and the work
  %                      area values.
  %   C-Mex counterpart: mdlStart
  %
  block.RegBlockMethod('Start', @Start);

  % 
  % Outputs:
  %   Functionality    : Call to generate the block outputs during a
  %                      simulation step.
  %   C-Mex counterpart: mdlOutputs
  %
  block.RegBlockMethod('Outputs', @Outputs);

  % 
  % Update:
  %   Functionality    : Call to update the discrete states
  %                      during a simulation step.
  %   C-Mex counterpart: mdlUpdate
  %
  block.RegBlockMethod('Update', @Update);

  % 
  % Derivatives:
  %   Functionality    : Call to update the derivatives of the
  %                      continuous states during a simulation step.
  %   C-Mex counterpart: mdlDerivatives
  %
  block.RegBlockMethod('Derivatives', @Derivatives);
  
  % 
  % Projection:
  %   Functionality    : Call to update the projections during a
  %                      simulation step.
  %   C-Mex counterpart: mdlProjections
  %
  block.RegBlockMethod('Projection', @Projection);
  
  % 
  % SimStatusChange:
  %   Functionality    : Call when simulation enters pause mode
  %                      or leaves pause mode.
  %   C-Mex counterpart: mdlSimStatusChange
  %
  block.RegBlockMethod('SimStatusChange', @SimStatusChange);
  
  % 
  % Terminate:
  %   Functionality    : Call at the end of a simulation for cleanup.
  %   C-Mex counterpart: mdlTerminate
  %
  block.RegBlockMethod('Terminate', @Terminate);

  %
  % GetSimState:
  %   Functionality    : Return the SimState of the block.
  %   C-Mex counterpart: mdlGetSimState
  %
  block.RegBlockMethod('GetSimState', @GetSimState);
  
  %
  % SetSimState:
  %   Functionality    : Set the SimState of the block using a given value.
  %   C-Mex counterpart: mdlSetSimState
  %
  block.RegBlockMethod('SetSimState', @SetSimState);

  % -----------------------------------------------------------------
  % Register the methods called during code generation.
  % -----------------------------------------------------------------
  
  %
  % WriteRTW:
  %   Functionality    : Write specific information to model.rtw file.
  %   C-Mex counterpart: mdlRTW
  %
  block.RegBlockMethod('WriteRTW', @WriteRTW);
%endfunction

% -------------------------------------------------------------------
% The local functions below are provided to illustrate how you may implement
% the various block methods listed above.
% -------------------------------------------------------------------

function CheckPrms(block)
  
%  a = block.DialogPrm(1).Data;
%  if ~strcmp(class(a), 'double')
%    me = MSLException(block.BlockHandle, message('Simulink:blocks:invalidParameter'));
%    throw(me);
%  end
  
%endfunction

function ProcessPrms(block)

  block.AutoUpdateRuntimePrms;
 
%endfunction

function SetInpPortFrameData(block, idx, fd)
  
  block.InputPort(idx).SamplingMode = fd;
  block.OutputPort(1).SamplingMode  = fd;
  
%endfunction

function SetInpPortDims(block, idx, di)
  
  block.InputPort(idx).Dimensions = di;
  block.OutputPort(1).Dimensions  = di;

%endfunction

function SetOutPortDims(block, idx, di)
  
  block.OutputPort(idx).Dimensions = di;
  block.InputPort(1).Dimensions    = di;

%endfunction

function SetInpPortDataType(block, idx, dt)
  
  block.InputPort(idx).DataTypeID = dt;
  block.OutputPort(1).DataTypeID  = dt;

%endfunction
  
function SetOutPortDataType(block, idx, dt)

  block.OutputPort(idx).DataTypeID  = dt;
  block.InputPort(1).DataTypeID     = dt;

%endfunction  

function SetInpPortComplexSig(block, idx, c)
  
  block.InputPort(idx).Complexity = c;
  block.OutputPort(1).Complexity  = c;

%endfunction 
  
function SetOutPortComplexSig(block, idx, c)

  block.OutputPort(idx).Complexity = c;
  block.InputPort(1).Complexity    = c;

%endfunction 
    
function DoPostPropSetup(block)
  block.NumDworks = 1; %2
  
  block.Dwork(1).Name            = 'x1';
  block.Dwork(1).Dimensions      = 4; %1
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = false; %true
  
  %block.Dwork(2).Name            = 'numPause';
  %block.Dwork(2).Dimensions      = 1;
  %block.Dwork(2).DatatypeID      = 7;      % uint32
  %block.Dwork(2).Complexity      = 'Real'; % real
  %block.Dwork(2).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;

%endfunction

function InitializeConditions(block)

block.ContStates.Data = [0,0,0,0];%1

%endfunction

function Start(block)

  block.Dwork(1).Data = [0,0,0,0];%0
%  block.Dwork(2).Data = uint32(1); 
   
%endfunction

function WriteRTW(block)
  
   %block.WriteRTWParam('matrix', 'M',    [1 2; 3 4]);
   %block.WriteRTWParam('string', 'Mode', 'Auto');
   
%endfunction

function Outputs(block)
  
  block.OutputPort(1).Data = block.ContStates.Data; % + block.InputPort(1).Data
  
%endfunction

function Update(block)
  
  block.Dwork(1).Data = block.InputPort(1).Data;
  
%endfunction

function Derivatives(block)

Rt = 0.978;
k_m = 33.5 / 1000;
EM = (1/285)/60;
J_L = 2.158e-5;
J_m = 1.35e-5;
J = 0.013;
l = 0.0824;
m = 3.678;
n = 4.091;
r = 0.0285;
c_m = 0.0015;
c_L = 0;


k1 = J_m + J_L/n^2 + r^2/n^2*(0.2810*m + 11.75*J);
k2 = r^2/n^2*(0.03812*m + 11.75*J);
k3 = r^2/n^2*(-0.2619*m + 9.050*J);
k4 = r^2/n^2*(-0.05717*m+9.050*J);
k5 = J_m + J_L/n^2 + r^2/n^2*(0.2459*m + 6.9710*J);
k6 = r^2/n^2*(0.07323*m + 6.9710*J);
k7 = c_m + c_L/n^2;
k8 = 0.2784*r^2/n^2*m;
k9 = 0.02184*r^2/n^2*m;
k10 = 0.2566*r^2/n^2*m;
k11 = 0.2347*r^2/n^2*m;

theta = [0, pi/2, pi, 3*pi/2];
G = [-sin(theta(1)), -sin(theta(2)), -sin(theta(3)), -sin(theta(4));
      cos(theta(1)),  cos(theta(2)),  cos(theta(3)),  cos(theta(4));
                  l,              l,              l,              l];

omega_m = block.ContStates.Data;
E = block.InputPort(1).Data;
X_dot_b = pinv(G') * r*omega_m;
psi_dot = X_dot_b(3);

Z = [k1, k2, k3, k4;
     k2, k1, k4, k3;
     k3, k4, k5, k6;
     k4, k3, k6, k5];
 
V = [          k7, -k8*psi_dot,   k9*psi_dot,  k10*psi_dot;
       k8*psi_dot,          k7, -k10*psi_dot,  -k9*psi_dot;
      -k9*psi_dot, k10*psi_dot,           k7, -k11*psi_dot;
     -k10*psi_dot,  k9*psi_dot,  k11*psi_dot,           k7];

% E = (Rt/k_m Z) \omega_m_dot + (Rt/k_m V + EM.I_4x4)\omega_m
% E - (Rt/k_m V + EM.I_4x4)\omega_m = (Rt/k_m Z) \omega_m_dot
% omega_m_dot = (Rt/k_m Z)^-1 (E - (Rt/k_m V + EM.I_4x4) omega_m)
omega_m_dot = (Rt / k_m * Z) \ (E - (Rt / k_m * V + EM * eye(4,4)) * omega_m);

block.Derivatives.Data = omega_m_dot;

%block.Derivatives.Data = 2*block.ContStates.Data;

%endfunction

function Projection(block)

states = block.ContStates.Data;
block.ContStates.Data = states+eps; 

%endfunction

function SimStatusChange(block, s)
  
  %block.Dwork(2).Data = block.Dwork(2).Data+1;    

  %if s == 0
  %  disp('Pause in simulation.');
  %elseif s == 1
  %  disp('Resume simulation.');
  %end
  
%endfunction
    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);

%endfunction
 
function outSimState = GetSimState(block)

outSimState = block.Dwork(1).Data;

%endfunction

function SetSimState(block, inSimState)

block.Dwork(1).Data = inSimState;

%endfunction
