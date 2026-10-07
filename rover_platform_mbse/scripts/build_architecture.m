function build_architecture()
%BUILD_ARCHITECTURE Build the System Composer model of the rover_ros platform.
%   Reads data/platform_architecture.json and data/platform_parameters.json (the
%   reverse-engineered description, with rover_ros source lines) and generates:
%     architecture/RoverPlatformArch.slx         logical/software architecture
%     architecture/RoverPlatformInterfaces.sldd  port interfaces
%     architecture/RoverPlatformProfile.xml      RosPackage / RosTopic / ... stereotypes
%     architecture/RoverPlatformDeployment.slx   execution nodes
%     architecture/RoverPlatformAlloc.mldatx     software -> deployment allocation
%   Idempotent: every artifact is deleted and rebuilt. Never edit the outputs by
%   hand; change the JSON and re-run.

% Copyright 2026 Mechatronics Academy
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%     http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.

root = mbse.root();
outDir = fullfile(root, 'architecture');
def = mbse.readJson(fullfile('data', 'platform_architecture.json'));
params = mbse.readJson(fullfile('data', 'platform_parameters.json'));

mdlName = 'RoverPlatformArch';
depName = 'RoverPlatformDeployment';
ddFile = 'RoverPlatformInterfaces.sldd';
profName = 'RoverPlatformProfile';
allocName = 'RoverPlatformAlloc';

mbse.closeAll();
oldDir = cd(outDir);
restoreDir = onCleanup(@() cd(oldDir));
for f = {[mdlName '.slx'], [depName '.slx'], ddFile, [profName '.xml'], ...
         [allocName '.mldatx'], [mdlName '.slmx']}
    if isfile(f{1})
        delete(f{1});
    end
end

%% Interfaces
model = systemcomposer.createModel(mdlName);
dict = systemcomposer.createDictionary(ddFile);
ifaces = containers.Map();
for k = 1:numel(def.interfaces)
    d = def.interfaces(k);
    iface = dict.addInterface(d.name);
    for e = 1:numel(d.elements)
        el = d.elements(e);
        if isempty(el.units)
            iface.addElement(el.name, Type = 'double');
        else
            iface.addElement(el.name, Type = 'double', Units = el.units);
        end
    end
    ifaces(d.name) = iface;
end
linkDictionary(model, ddFile);

%% Profile
prof = systemcomposer.profile.Profile.createProfile(profName);
st = prof.addStereotype('RosPackage', AppliesTo = 'Component');
addStringProps(st, {'package', 'swrsPrefix', 'language', 'lifecycle', 'executables', 'behaviourModel'});
st = prof.addStereotype('FunctionalSubsystem', AppliesTo = 'Component');
addStringProps(st, {'label'});
st = prof.addStereotype('ExternalActor', AppliesTo = 'Component');
addStringProps(st, {'description'});
st = prof.addStereotype('ExecutionNode', AppliesTo = 'Component');
addStringProps(st, {'description', 'source'});
st = prof.addStereotype('RosTopic', AppliesTo = 'Connector');
addStringProps(st, {'topic', 'msgType', 'qos', 'rate', 'source'});
st = prof.addStereotype('RosService', AppliesTo = 'Connector');
addStringProps(st, {'service', 'srvType', 'rate', 'source'});
st = prof.addStereotype('Ros2Control', AppliesTo = 'Connector');
addStringProps(st, {'interfaces', 'rate', 'source'});
st = prof.addStereotype('HwBus', AppliesTo = 'Connector');
addStringProps(st, {'bus', 'address', 'rate', 'source'});
prof.save();
model.applyProfile(profName);

%% Components: functional subsystems (with package parts) and external actors
top = model.Architecture;
comps = containers.Map();   % name -> Component
where = containers.Map();   % name -> composite name ('' = top level)
for k = 1:numel(def.composites)
    c = def.composites(k);
    sub = top.addComponent(c.name);
    applyProps(sub, [profName '.FunctionalSubsystem'], struct('label', c.label));
    comps(c.name) = sub;
    members = cellstr(c.members);
    for m = 1:numel(members)
        pkg = members{m};
        info = def.components(strcmp({def.components.name}, pkg));
        part = sub.Architecture.addComponent(pkg);
        applyProps(part, [profName '.RosPackage'], struct( ...
            'package', pkg, 'swrsPrefix', info.prefix, 'language', info.language, ...
            'lifecycle', info.lifecycle, 'executables', info.executables, ...
            'behaviourModel', info.behaviour_model));
        comps(pkg) = part;
        where(pkg) = c.name;
    end
end
for k = 1:numel(def.actors)
    a = def.actors(k);
    act = top.addComponent(a.name);
    applyProps(act, [profName '.ExternalActor'], struct('description', a.description));
    comps(a.name) = act;
    where(a.name) = '';
end

% Numeric parameters, with their rover_ros source as the parameter description
for k = 1:numel(params.parameters)
    p = params.parameters(k);
    comp = comps(p.component);
    unit = p.unit;
    if strcmp(unit, '1')
        unit = '';
    end
    comp.Architecture.addParameter(p.name, Value = num2str(p.value, 10), Units = unit);
end

%% Connections, routed through subsystem boundary ports where needed
nConn = 0;
owners = containers.Map();
stereotyped = containers.Map();
connections = mbse.items(def.connections);
for k = 1:numel(connections)
    c = connections{k};
    iface = ifaces(c.interface);
    portName = mbse.sanitize(c.port);
    srcPort = getOrAddPort(comps(c.src), portName, 'out', iface);
    dstPort = getOrAddPort(comps(c.dst), uniqueInName(owners, comps(c.dst), portName, c.src), 'in', iface);
    srcLoc = where(c.src);
    dstLoc = where(c.dst);
    conns = {};
    if strcmp(srcLoc, dstLoc)
        conns{end+1} = connectOnce(archOf(comps, srcLoc, top), srcPort, dstPort); %#ok<AGROW>
    else
        % Leave the source subsystem through an out boundary port
        if ~isempty(srcLoc)
            outer = comps(srcLoc);
            bOut = getOrAddPort(outer, portName, 'out', iface);
            conns{end+1} = connectOnce(outer.Architecture, srcPort, ...
                outer.Architecture.getPort(portName)); %#ok<AGROW>
            srcPort = bOut;
        end
        % Enter the destination subsystem through an in boundary port
        if ~isempty(dstLoc)
            outer = comps(dstLoc);
            inName = uniqueInName(owners, outer, portName, c.src);
            bIn = getOrAddPort(outer, inName, 'in', iface);
            conns{end+1} = connectOnce(outer.Architecture, ...
                outer.Architecture.getPort(inName), dstPort); %#ok<AGROW>
            dstPort = bIn;
        end
        conns{end+1} = connectOnce(top, srcPort, dstPort); %#ok<AGROW>
    end
    for j = 1:numel(conns)
        stereotypeConnector(conns{j}, profName, c, stereotyped);
    end
    nConn = nConn + 1;
end

Simulink.BlockDiagram.arrangeSystem(mdlName);
for k = 1:numel(def.composites)
    Simulink.BlockDiagram.arrangeSystem([mdlName '/' def.composites(k).name]);
end

%% Deployment view and software -> node allocation
dep = systemcomposer.createModel(depName);
dep.applyProfile(profName);
nodes = struct( ...
    'name', {'PlatformContainer', 'SafetyPlcNode', 'PhidgetVintHub', 'Esp32BmsBridge', ...
             'LedControllers', 'OrchestratorContainer', 'SensorsContainer', 'OperatorPc', 'ElrsReceiver'}, ...
    'description', { ...
        'rover-a1-platform container: ARM64 rover computer, Ubuntu 26.04, ROS 2 Lyrical, rmw_zenoh_cpp', ...
        'Safety PLC (Modbus TCP 192.168.88.11:502)', ...
        'Phidget VINT hub: 4x DCC1000 + MOT0110 IMU (USB)', ...
        'ESP32 reading the Daly BMS, UDP sender', ...
        'SK9822 panel UDP controllers 192.168.77.201/202', ...
        'rover-a1-orchestrator container (Nav 2, missions)', ...
        'rover-a1-sensors container (GNSS, lidar)', ...
        'Operator PC / browser (Drive UI, Foxglove)', ...
        'ExpressLRS receiver, USB-UART on the RUTX11 router; Serial Utilities Over IP sends CRSF as UDP to PlatformContainer 192.168.1.201:10111 (source 192.168.1.1)'}, ...
    'source', { ...
        'rover_docker/rover_a1_platform/Dockerfile:1-5 (outside rover_ros)', ...
        'rover_description/urdf/rover_a1/rover_a1_macro.urdf.xacro:148-149', ...
        'rover_hardware_interface/src/rover_driver/rover_a1_driver.cpp:73-80', ...
        'rover_battery/config/rover_battery.yaml:6-7', ...
        'rover_led/config/rover_a1_udp_led_channel_1.yaml:3-4', ...
        'rover_twist_mux/config/rover_twist_mux.yaml:5-9', ...
        'rover_gps_heading/src/infrastructure/rover_gps_heading_node.cpp:104-105', ...
        'rover_bringup/launch/rover_web_bridges.launch.py:54-198', ...
        'rover_crsf_teleop/config/rover_crsf_teleop.yaml:4-24'});
depComps = containers.Map();
for k = 1:numel(nodes)
    n = dep.Architecture.addComponent(nodes(k).name);
    applyProps(n, [profName '.ExecutionNode'], rmfield(nodes(k), 'name'));
    depComps(nodes(k).name) = n;
end
Simulink.BlockDiagram.arrangeSystem(depName);

actorNode = containers.Map( ...
    {'SafetyPlc', 'DriveMotors', 'ImuSensor', 'BmsBridge', 'LedPanels', ...
     'Orchestrator', 'SensorPayload', 'OperatorStation', 'RcTransmitter'}, ...
    {'SafetyPlcNode', 'PhidgetVintHub', 'PhidgetVintHub', 'Esp32BmsBridge', 'LedControllers', ...
     'OrchestratorContainer', 'SensorsContainer', 'OperatorPc', 'ElrsReceiver'});
aset = systemcomposer.allocation.createAllocationSet(allocName, mdlName, depName);
scenario = aset.Scenarios(1);
scenario.Name = 'SoftwareToExecution';
for k = 1:numel(def.components)
    scenario.allocate(comps(def.components(k).name), depComps('PlatformContainer'));
end
keysA = actorNode.keys;
for k = 1:numel(keysA)
    scenario.allocate(comps(keysA{k}), depComps(actorNode(keysA{k})));
end

%% Save
dict.save();
model.save();
dep.save();
aset.save();
fprintf('build_architecture: %d subsystems, %d packages, %d actors, %d interfaces, %d connections, %d parameters, %d nodes\n', ...
    numel(def.composites), numel(def.components), numel(def.actors), numel(def.interfaces), ...
    nConn, numel(params.parameters), numel(nodes));
mbse.closeAll();
end

%% Helpers
function addStringProps(st, names)
for k = 1:numel(names)
    st.addProperty(names{k}, Type = 'string');
end
end

function applyProps(elem, stereo, values)
elem.applyStereotype(stereo);
f = fieldnames(values);
for k = 1:numel(f)
    v = char(string(values.(f{k})));
    elem.setProperty([stereo '.' f{k}], ['"' strrep(v, '"', '''') '"']);
end
end

function port = getOrAddPort(comp, name, direction, iface)
port = comp.getPort(name);
if isempty(port)
    port = comp.Architecture.addPort(name, direction);
    port.setInterface(iface);
    port = comp.getPort(name);
end
end

function name = uniqueInName(owners, comp, base, src)
% An in port name must be unique per source: two senders of 'diagnostics' get
% 'diagnostics' and 'diagnostics_from_<src>'. owners is a containers.Map handle.
key = [comp.getQualifiedName() '|' base];
if ~isKey(owners, key) || strcmp(owners(key), src)
    owners(key) = src; %#ok<NASGU> containers.Map is a handle
    name = base;
else
    name = [base '_from_' mbse.sanitize(src)];
end
end

function arch = archOf(comps, loc, top)
if isempty(loc)
    arch = top;
else
    arch = comps(loc).Architecture;
end
end

function conn = connectOnce(arch, srcPort, dstPort)
for k = 1:numel(arch.Connectors)
    c = arch.Connectors(k);
    if isequal(c.SourcePort, srcPort) && isequal(c.DestinationPort, dstPort)
        conn = c;
        return
    end
end
conn = connect(srcPort, dstPort);   % port-level form; arch.connect(...) returns empty
end

function stereotypeConnector(conn, profName, c, done)
% done: containers.Map of connector UUIDs already stereotyped (the first
% connection routed through a shared boundary segment owns its stereotype).
if isempty(conn)
    error('build_architecture:connect', 'No connector for %s -> %s (%s).', c.src, c.dst, c.port);
end
if isKey(done, conn.UUID)
    return
end
done(conn.UUID) = true; %#ok<NASGU> containers.Map is a handle
switch c.kind
    case 'RosTopic'
        applyProps(conn, [profName '.RosTopic'], struct('topic', c.topic, ...
            'msgType', c.msgType, 'qos', c.qos, 'rate', c.rate, 'source', c.source));
    case 'RosService'
        applyProps(conn, [profName '.RosService'], struct('service', c.topic, ...
            'srvType', c.msgType, 'rate', c.rate, 'source', c.source));
    case 'Ros2Control'
        applyProps(conn, [profName '.Ros2Control'], struct('interfaces', c.topic, ...
            'rate', c.rate, 'source', c.source));
    case 'HwBus'
        applyProps(conn, [profName '.HwBus'], struct('bus', c.bus, ...
            'address', c.address, 'rate', c.rate, 'source', c.source));
end
end
